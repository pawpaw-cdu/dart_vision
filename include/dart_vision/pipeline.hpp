#ifndef DART_VISION_PIPELINE_HPP
#define DART_VISION_PIPELINE_HPP

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

#include "dart_vision/command_sender.hpp"
#include "dart_vision/config.hpp"
#include "dart_vision/control.hpp"
#include "dart_vision/detector.hpp"
#include "dart_vision/guidance.hpp"
#include "dart_vision/roi_tracker.hpp"
#include "dart_vision/solver.hpp"
#include "dart_vision/types.hpp"

namespace dart_vision {

struct PipelineStats {
    uint64_t frames = 0;
    uint64_t detected = 0;
    uint64_t sent = 0;
    uint64_t det_streak = 0;
    uint64_t max_det_streak = 0;
    uint64_t zoom_frames = 0;
    uint64_t zoom_hits = 0;      // 变焦窗口内直接命中的帧数
    uint64_t zoom_fallback = 0;  // 变焦窗口漏检、由整幅回退救回的帧数
    double sum_process_ms = 0.0;
    double sum_interval_ms = 0.0;
    double process_ms = 0.0;
    double interval_ms = 0.0;
    double fps = 0.0;
    int64_t first_t_ms = 0;
    int64_t last_t_ms = 0;
    bool has_first = false;

    void note(const Detection& det, double process_ms_value, bool zoomed, int64_t t_ms);
    std::string summary() const;
};

// 检测 -> 解算 -> 回发 的整条链路，对应 main_final.py 主循环里的
// find_max_blob() / build_found_roi() / send_blob_uart() / draw_overlay()。
class Pipeline {
public:
    Pipeline(const AppConfig& cfg, CommandSender* sender, DartStateHolder* state_holder);

    void processFrame(const Frame& frame, cv::Mat* canvas);

    void setArmed(bool armed);
    bool armed() const { return tracker_.armed(); }
    void setZoomEnabled(bool enabled);
    bool zoomEnabled() const { return tracker_.zoomed(); }
    void reset();

    const PipelineStats& stats() const { return stats_; }
    const Detection& lastDetection() const { return last_detection_; }
    const AimSolution& lastSolution() const { return last_solution_; }
    const RoiTracker::Window& lastWindow() const { return window_; }
    bool lastFromFallback() const { return last_from_fallback_; }
    const LabDetector& detector() const { return detector_; }
    const RoiTracker& tracker() const { return tracker_; }
    const AngleSolver& solver() const { return solver_; }
    const GuidanceLaw& guidance() const { return guidance_; }
    const DartController& controller() const { return controller_; }
    const ControlOutput& lastControl() const { return last_control_; }
    /// 最近一次控制更新时用到的飞镖状态（判断 STATE 回推有没有在流、速度多少）
    const DartState& lastState() const { return last_state_; }

private:
    AppConfig cfg_;
    LabDetector detector_;
    RoiTracker tracker_;
    AngleSolver solver_;
    GuidanceLaw guidance_;
    DartStateHolder* state_holder_ = nullptr;
    DartController controller_;
    ControlOutput last_control_;
    DartState last_state_;
    CommandSender* sender_ = nullptr;

    PipelineStats stats_;
    Detection last_detection_;
    AimSolution last_solution_;
    RoiTracker::Window window_;
    bool last_from_fallback_ = false;
    // 制导稳定期：重新捕获目标后先等 arm_delay_ms，再开始下发射率类指令
    int64_t guidance_ready_t_ms_ = 0;
    bool guidance_armed_ = false;
    bool have_acquire_angle_ = false;
    float acquire_yaw_ = 0.0f;
    float acquire_pitch_ = 0.0f;
};

}  // namespace dart_vision

#endif  // DART_VISION_PIPELINE_HPP
