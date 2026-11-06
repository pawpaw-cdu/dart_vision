#include "dart_vision/pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include <opencv2/imgproc.hpp>

#include "dart_vision/overlay.hpp"

namespace dart_vision {

void PipelineStats::note(const Detection& det, double process_ms_value, bool zoomed,
                         int64_t t_ms) {
    ++frames;
    sum_process_ms += process_ms_value;
    process_ms = process_ms_value;
    if (!has_first) {
        has_first = true;
        first_t_ms = t_ms;
    } else if (t_ms > last_t_ms) {
        interval_ms = static_cast<double>(t_ms - last_t_ms);
        sum_interval_ms += interval_ms;
    }
    last_t_ms = t_ms;
    if (zoomed) ++zoom_frames;

    if (det.found) {
        ++detected;
        ++det_streak;
        max_det_streak = std::max(max_det_streak, det_streak);
    } else {
        det_streak = 0;
    }
    const double elapsed = static_cast<double>(t_ms - first_t_ms);
    fps = elapsed > 0.0 ? (frames - 1) * 1000.0 / elapsed : 0.0;
}

std::string PipelineStats::summary() const {
    std::ostringstream oss;
    const double denom = frames > 0 ? static_cast<double>(frames) : 1.0;
    oss.setf(std::ios::fixed);
    oss.precision(2);
    oss << "帧数=" << frames << " 命中=" << detected << " (" << (100.0 * detected / denom)
        << "%)"
        << " 回发=" << sent << " 最长连续命中=" << max_det_streak << " 帧"
        << " 变焦帧=" << zoom_frames << "(窗口命中=" << zoom_hits << ",整幅回退=" << zoom_fallback
        << ")"
        << " 平均处理=" << (sum_process_ms / denom) << "ms"
        << " 平均帧间隔=" << (frames > 1 ? sum_interval_ms / (frames - 1) : 0.0) << "ms";
    return oss.str();
}

Pipeline::Pipeline(const AppConfig& cfg, CommandSender* sender, DartStateHolder* state_holder)
    : cfg_(cfg),
      detector_(cfg.detector),
      tracker_(cfg.tracker),
      solver_(cfg.solver, cfg.camera),
      guidance_(cfg.guidance),
      controller_(cfg.control),
      sender_(sender),
      state_holder_(state_holder) {}

void Pipeline::reset() {
    tracker_.reset();
    solver_.reset();
    stats_ = PipelineStats();
    last_detection_ = Detection();
    last_solution_ = AimSolution();
    last_from_fallback_ = false;
    guidance_armed_ = false;
    guidance_ready_t_ms_ = 0;
    have_acquire_angle_ = false;
}

void Pipeline::setArmed(bool armed) {
    tracker_.setArmed(armed);
    if (!armed) solver_.reset();
}

void Pipeline::setZoomEnabled(bool enabled) { cfg_.tracker.zoom_enable = enabled; }

void Pipeline::processFrame(const Frame& frame, cv::Mat* canvas) {
    const auto begin = std::chrono::steady_clock::now();
    if (frame.bgr.empty()) return;

    const cv::Size size = frame.bgr.size();
    Detection detection;
    last_from_fallback_ = false;

    const bool do_detect = tracker_.beginFrame(size, frame.t_ms, &window_);
    if (do_detect) {
        if (window_.zoomed) {
            cv::Mat processing;
            cv::resize(frame.bgr(window_.crop), processing, size, 0.0, 0.0, cv::INTER_LINEAR);
            const Detection zoomed = detector_.detect(processing, window_.detect_roi);
            if (zoomed.found) {
                detection.found = true;
                detection.blob = RoiTracker::toFrameCoords(zoomed.blob, window_);
                detection.score = zoomed.score;
                ++stats_.zoom_hits;
            } else {
                // 变焦窗口很窄，目标一旦跑出窗口就会丢；这里立刻用整幅图回退搜索一次，
                // 命中后 ROI 会被重新拉回目标，相当于第二阶段的兜底。
                const Detection wide =
                    detector_.detect(frame.bgr, cv::Rect(0, 0, size.width, size.height));
                if (wide.found) {
                    detection = wide;
                    last_from_fallback_ = true;
                    ++stats_.zoom_fallback;
                }
            }
        } else {
            const Detection raw = detector_.detect(frame.bgr, window_.detect_roi);
            if (raw.found) {
                detection.found = true;
                detection.blob = raw.blob;
                detection.score = raw.score;
            }
        }
    }

    tracker_.update(detection, size);

    // 目标连续丢失后重新出现时，重新走一遍稳定期
    if (!detection.found && tracker_.lostFrames() >= 3) {
        guidance_armed_ = false;
        have_acquire_angle_ = false;
    }

    AimSolution solution;
    if (detection.found) {
        solution = solver_.solve(detection.blob, frame.t_ms);
        // ---- 控制（含坐标变换）：按"仿真只管物理"的原则，律在这一侧 ----
        if (solution.valid && cfg_.control.enabled) {
            // 按"这一帧图像的时刻"取姿态。
            // 相机固连在镖头时，机身姿态抖动会直接进视线角；若拿"最新"姿态
            // 去配一帧几十毫秒前的图像，就引入 角速率 × 延迟 的视线角误差
            // （末端机体几十度/秒，20ms 错配 ≈ 1°，实测让散布翻倍）。
            // frame.timestamp_us 是仿真器写在图像帧头里的墙钟时间戳，
            // 与 STATE 尾部的墙钟字段是同一个钟，可以直接配对。
            last_state_ = state_holder_ ? state_holder_->getAtWallClock(frame.timestamp_us)
                                        : DartState();
            const DartState& state = last_state_;
            // 测距用的表观尺寸：用亚像素值，不要用整型 max(w,h)
            // （变焦反算时的取整会带来 20~25% 的距离跳变）
            const double blob_size = detection.blob.apparentSize();
            // 控制律内部的时间也用对齐后的仿真时刻：交战门限、斜坡、λ̇ 差分
            // 全部落在同一个时基上。
            const int64_t control_t_ms = state.valid ? state.t_ms : frame.t_ms;
            last_control_ = controller_.update(solution.yaw, solution.pitch, blob_size, state,
                                               control_t_ms);
            if (last_control_.active && cfg_.send && sender_) {
                sender_->sendServo(last_control_.pitch, last_control_.yaw, last_control_.roll);
                ++stats_.sent;
            }
        }
        // 制导律只作用于「角度域」协议；szu_uart 是写实像素框协议，保持原样
        // 交给主控/仿真器换算（见 README「两种角度口径」）。
        if (!cfg_.control.enabled && solution.valid && cfg_.command.protocol != "szu_uart") {
            // 稳定期：目标（重新）出现后的 arm_delay_ms 内不下发制导指令，
            // 避开"发射瞬间相机对准速度矢量"造成的跳变
            if (!guidance_armed_) {
                // 发射判定：记住"捕获时的视线角"，只有偏离超过阈值才认为镖在动。
                // 发射前镖静止、视线角几乎不动，此时必须发中立舵，否则出膛瞬间
                // 舵面已经被满舵压住，弹道直接被带偏。
                if (!have_acquire_angle_) {
                    acquire_yaw_ = solution.yaw;
                    acquire_pitch_ = solution.pitch;
                    have_acquire_angle_ = true;
                }
                const double deviation_deg =
                    std::max(std::fabs(solution.yaw - acquire_yaw_),
                             std::fabs(solution.pitch - acquire_pitch_)) *
                    kRadToDeg;
                if (cfg_.guidance.arm_on_motion_deg <= 0.0 ||
                    deviation_deg >= cfg_.guidance.arm_on_motion_deg) {
                    guidance_armed_ = true;
                    guidance_ready_t_ms_ = frame.t_ms;
                    have_acquire_angle_ = false;
                }
            }
            const bool in_arm_window =
                !guidance_armed_ ||
                static_cast<double>(frame.t_ms - guidance_ready_t_ms_) <
                    cfg_.guidance.arm_delay_ms;
            AimSolution law_input = solution;
            // 稳定期内按"角速度不可用"处理：制导律只输出偏置、不发动作
            if (in_arm_window) law_input.rate_valid = false;
            double cmd_yaw = 0.0;
            double cmd_pitch = 0.0;
            guidance_.apply(law_input, &cmd_yaw, &cmd_pitch);
            solution.yaw_pred = static_cast<float>(cmd_yaw);
            solution.pitch_pred = static_cast<float>(cmd_pitch);
        }
        // 解耦架构下（control.enabled）只发 DART 舵角报文，不再发旧的像素框/角度报文，
        // 否则两种指令会在同一条连接上互相干扰。
        if (!cfg_.control.enabled && solution.valid && cfg_.send && sender_) {
            sender_->sendAim(solution, detection.blob);
            ++stats_.sent;
        }
    }

    const auto end = std::chrono::steady_clock::now();
    const double process_ms =
        std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000.0;
    stats_.note(detection, process_ms, window_.zoomed, frame.t_ms);

    last_detection_ = detection;
    last_solution_ = solution;

    if (canvas) {
        OverlayState state;
        state.armed = tracker_.armed();
        state.zoomed = window_.zoomed;
        state.zoom = window_.zoom;
        state.crop = window_.crop;
        state.tracked_roi = tracker_.trackedRoi();
        state.detection = detection;
        state.solution = solution;
        state.fps = stats_.fps;
        state.process_ms = process_ms;
        state.t_ms = frame.t_ms;
        state.frame_index = stats_.frames;
        if (sender_) {
            std::ostringstream oss;
            oss << "command " << (sender_->connected() ? "connected" : "idle") << " sent="
                << sender_->sent() << " dropped=" << sender_->dropped()
                << " failed=" << sender_->failed();
            state.sender_state = oss.str();
        }
        drawOverlay(*canvas, state);
    }
}

}  // namespace dart_vision
