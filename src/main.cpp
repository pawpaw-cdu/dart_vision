// dart_vision：制导飞镖视觉（检测 -> 解算 -> 回发）
//
// 输入：simulator_system（guided_dart 场景）的 TCP 图像流，或本地录像
// 输出：TCP 指令通道（szu_uart / text_angle / sim_gimbal 三种协议）
//
// 检测、ROI 追踪、两阶段变焦、解算与回发格式的移植来源见 README.md。
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "dart_vision/command_sender.hpp"
#include "dart_vision/config.hpp"
#include "dart_vision/frame_source.hpp"
#include "dart_vision/pipeline.hpp"
#ifdef DART_VISION_WITH_MEDIA
#include <opencv2/highgui.hpp>
#include "offline_video.hpp"
#endif

namespace {

std::atomic<bool> g_stop(false);

void onSignal(int) { g_stop.store(true); }

void printUsage() {
    std::cout
        << "用法: dart_vision [选项]\n"
        << "  --config PATH          配置文件（默认 config/simulator.yaml）\n"
        << "  --source tcp|video     图像源（默认取配置）\n"
        << "  --video PATH           离线录像路径（等价于 --source video）\n"
        << "  --host HOST            仿真器图像流地址\n"
        << "  --port PORT            仿真器图像流端口（默认 7666）\n"
        << "  --command-host HOST    指令回发地址（默认与 --host 相同）\n"
        << "  --command-port PORT    指令回发端口（默认 7667）\n"
        << "  --protocol NAME        szu_uart | text_angle | sim_gimbal\n"
        << "  --axis NAME            both | yaw | pitch（对应串口 0x11/0x22 模式）\n"
        << "  --no-send              只算不发，便于离线分析\n"
        << "  --no-show              不打开窗口（无头运行）\n"
        << "  --fast                 离线回放不按原帧率节流\n"
        << "  --loop                 录像循环播放\n"
        << "  --no-loop              录像只播一遍\n"
        << "  --record               启动即开始录像\n"
        << "  --duration-ms MS       运行指定时长后退出\n"
        << "  --exit-after-frames N  处理 N 帧后退出\n"
        << "  --log-every N          每 N 帧打印一次状态（默认 30，0 = 关闭）\n"
        << "  --dump-dir DIR         按 s 键时把画面/掩码保存到该目录\n"
        << "  --dump-every N         每 N 帧自动保存一张画面+掩码（0 = 关闭）\n"
        << "  --telemetry PATH       把每帧的瞄准误差/角速度/距离写成 CSV（调制导用）\n"
        << "  --help                 显示本帮助\n\n"
        << "运行中按键: q/ESC 退出  space 上电/断电  f 击发  r 录像  z 变焦  s 存图\n";
}

bool isDigits(const std::string& value) {
    return !value.empty() && std::all_of(value.begin(), value.end(), [](char c) {
               return c >= '0' && c <= '9';
           });
}

std::string executableDir(const char* argv0) {
    const std::string path = argv0 ? argv0 : "";
    const std::size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) return ".";
    return path.substr(0, pos);
}

bool fileExists(const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    std::fclose(f);
    return true;
}

// 允许从 build/ 目录直接运行：配置既支持相对当前目录，也支持相对可执行文件
std::string resolveConfigPath(const std::string& requested, const std::string& exe_dir) {
    if (fileExists(requested)) return requested;
    const std::string candidates[] = {exe_dir + "/../config/simulator.yaml",
                                      exe_dir + "/config/simulator.yaml",
                                      "../config/simulator.yaml"};
    for (const std::string& candidate : candidates) {
        if (fileExists(candidate)) {
            std::cout << "[配置] 使用 " << candidate << std::endl;
            return candidate;
        }
    }
    return requested;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::string config_path = "config/simulator.yaml";
    std::string dump_dir;
    std::string telemetry_path;
    std::string source_override;
    std::string video_override;
    std::string host_override;
    std::string command_host_override;
    std::string protocol_override;
    std::string axis_override;
    int port_override = 0;
    int command_port_override = 0;
    int log_every_override = -1;
    int dump_every = 0;
    int64_t duration_ms = 0;
    uint64_t exit_after_frames = 0;
    bool send_flag = true;
    bool show_flag = true;
    bool fast = false;
    bool loop = false;
    bool loop_forced_off = false;
    bool record_start = false;
    bool send_forced_off = false;
    bool show_forced_off = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto nextValue = [&](std::string* out) -> bool {
            if (i + 1 >= argc) return false;
            *out = argv[++i];
            return true;
        };
        std::string value;

        if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        } else if (arg == "--config" && nextValue(&value)) {
            config_path = value;
        } else if (arg == "--source" && nextValue(&value)) {
            source_override = value;
        } else if (arg == "--video" && nextValue(&value)) {
            video_override = value;
        } else if (arg == "--host" && nextValue(&value)) {
            host_override = value;
        } else if (arg == "--port" && nextValue(&value)) {
            if (isDigits(value)) port_override = std::stoi(value);
        } else if (arg == "--command-host" && nextValue(&value)) {
            command_host_override = value;
        } else if (arg == "--command-port" && nextValue(&value)) {
            if (isDigits(value)) command_port_override = std::stoi(value);
        } else if (arg == "--protocol" && nextValue(&value)) {
            protocol_override = value;
        } else if (arg == "--axis" && nextValue(&value)) {
            axis_override = value;
        } else if (arg == "--log-every" && nextValue(&value)) {
            if (isDigits(value)) log_every_override = std::stoi(value);
        } else if (arg == "--duration-ms" && nextValue(&value)) {
            if (isDigits(value)) duration_ms = std::stoll(value);
        } else if (arg == "--exit-after-frames" && nextValue(&value)) {
            if (isDigits(value)) exit_after_frames = std::stoull(value);
        } else if (arg == "--dump-dir" && nextValue(&value)) {
            dump_dir = value;
        } else if (arg == "--dump-every" && nextValue(&value)) {
            if (isDigits(value)) dump_every = std::stoi(value);
        } else if (arg == "--telemetry" && nextValue(&value)) {
            telemetry_path = value;
        } else if (arg == "--no-send") {
            send_flag = false;
            send_forced_off = true;
        } else if (arg == "--send") {
            send_flag = true;
            send_forced_off = false;
        } else if (arg == "--no-show") {
            show_flag = false;
            show_forced_off = true;
        } else if (arg == "--show") {
            show_flag = true;
            show_forced_off = false;
        } else if (arg == "--fast") {
            fast = true;
        } else if (arg == "--loop") {
            loop = true;
            loop_forced_off = false;
        } else if (arg == "--no-loop") {
            loop = false;
            loop_forced_off = true;
        } else if (arg == "--record") {
            record_start = true;
        } else {
            std::cerr << "未知参数: " << arg << std::endl;
            printUsage();
            return 2;
        }
    }

    config_path = resolveConfigPath(config_path, executableDir(argc > 0 ? argv[0] : ""));

    std::string warning;
    dart_vision::AppConfig config = dart_vision::AppConfig::load(config_path, &warning);
    if (!warning.empty()) std::cerr << "[配置] " << warning << std::endl;

    if (!source_override.empty()) config.source.mode = source_override;
    if (!video_override.empty()) {
        config.source.mode = "video";
        config.source.video_path = video_override;
    }
    if (!host_override.empty()) {
        config.source.host = host_override;
        if (command_host_override.empty()) config.command.host = host_override;
    }
    if (port_override > 0) config.source.port = port_override;
    if (!command_host_override.empty()) config.command.host = command_host_override;
    if (command_port_override > 0) config.command.port = command_port_override;
    if (!protocol_override.empty()) config.command.protocol = protocol_override;
    if (!axis_override.empty()) config.solver.axis = axis_override;
    if (log_every_override >= 0) config.log_every = log_every_override;
    if (fast) config.source.realtime = false;
    if (loop) config.source.loop = true;
    if (loop_forced_off) config.source.loop = false;
    config.show = show_flag && !show_forced_off;
    config.send = send_flag && !send_forced_off;
    config.exit_after_ms = duration_ms;
    if (config.command.host.empty()) config.command.host = config.source.host;
    config.camera.finalize();

#ifndef DART_VISION_WITH_MEDIA
    if (config.show) {
        std::cout << "[提示] 本机为无头构建（-DDART_VISION_WITH_MEDIA=OFF），不显示窗口，"
                     "也不提供录像；需要可视化请重新构建。"
                  << std::endl;
        config.show = false;
    }
    if (record_start || config.record.enabled) {
        std::cout << "[提示] 无头构建不提供录像功能" << std::endl;
        record_start = false;
        config.record.enabled = false;
    }
#endif

    std::cout << "[配置] " << config_path << "\n  " << config.describe() << std::endl;

    if (config.guidance.enabled && config.command.protocol == "szu_uart") {
        std::cout << "[提示] guidance 已开启，但 szu_uart 只回发原始像素框，制导律不会生效；"
                     "要调闭环请用 --protocol text_angle"
                  << std::endl;
    }

    // ---- 指令回发线程 ----
    // 状态容器：网络线程写入仿真器回推的 STATE，控制律（视觉侧）读取
    dart_vision::DartStateHolder state_holder;
    dart_vision::CommandSender sender(config.command, &state_holder);
    if (config.send && config.command.enabled) {
        sender.start();
        std::cout << "[回发] " << sender.describe() << std::endl;
    } else {
        config.send = false;
        std::cout << "[回发] 已关闭" << std::endl;
    }

    // ---- 图像源 ----
    std::string error;
    std::unique_ptr<dart_vision::FrameSource> source =
        dart_vision::createFrameSource(config_path, config.source, &error);
    if (!source) {
        std::cerr << "[图像] 打开失败: " << error << std::endl;
        return 1;
    }
    std::cout << "[图像] " << source->describe() << std::endl;

#ifdef DART_VISION_WITH_MEDIA
    // ---- 录像（沿用原工程独立录制线程） ----
    OFFLINE_VIDEO recorder(config.record.width, config.record.height, config.record.fps);
    bool recording = false;
    uint64_t snap_index = 0;
    if (record_start || config.record.enabled) recording = recorder.start();
#endif

    // ---- 业务链路 ----
    dart_vision::Pipeline pipeline(config, &sender, &state_holder);

    dart_vision::Frame frame;
    cv::Mat canvas;
    bool running = true;

    // ---- 遥测：每帧一行，供离线画曲线/统计制导效果 ----
    std::FILE* telemetry = nullptr;
    if (!telemetry_path.empty()) {
        telemetry = std::fopen(telemetry_path.c_str(), "w");
        if (telemetry) {
            std::fprintf(telemetry,
                         "frame,t_ms,phase,found,blob_x,blob_y,blob_w,blob_h,area,center_x,"
                         "center_y,yaw_deg,pitch_deg,yaw_rate_dps,pitch_rate_dps,"
                         "cmd_yaw_deg,cmd_pitch_deg,dist_m,fallback,zoom,sent,"
                         "ctl_active,ctl_pitch_deg,ctl_yaw_deg,accel_cmd_mps2,"
                         "avail_accel_mps2,aim_err_deg,ctl_range_m,range_valid,"
                         "los_rate_dps,state_valid,speed_mps,sim_t_ms,los_world_x,los_world_y,los_world_z\n");
            std::fflush(telemetry);
            std::cout << "[遥测] 写入 " << telemetry_path << std::endl;
        } else {
            std::cerr << "[遥测] 无法写入 " << telemetry_path << std::endl;
        }
    }

    while (running && !g_stop.load()) {
        if (!source->next(&frame)) {
            if (config.source.mode == "tcp") {
                std::cerr << "[图像] 图像流结束（模拟器关闭或协议错误）" << std::endl;
            }
            break;
        }

        if (config.source.mode == "video") {
            // 离线回放时 t_ms 从录像起点重新计算，便于复现 T_START / 变焦时序
            frame.t_ms = static_cast<int64_t>(static_cast<double>(frame.seq - 1) * 1000.0 /
                                              std::max(1.0, source->nominalFps()));
        }

        cv::Mat* canvas_ptr = nullptr;
        if (config.show || (!dump_dir.empty() && dump_every > 0)) {
            canvas = frame.bgr.clone();
            canvas_ptr = &canvas;
        }
        pipeline.processFrame(frame, canvas_ptr);

        if (!dump_dir.empty() && dump_every > 0 && canvas_ptr != nullptr &&
            pipeline.stats().frames % static_cast<uint64_t>(dump_every) == 0) {
            char name[512];
            std::snprintf(name, sizeof(name), "%s/frame_%06llu.png", dump_dir.c_str(),
                          static_cast<unsigned long long>(pipeline.stats().frames));
            cv::imwrite(name, canvas);

            const cv::Mat& mask = pipeline.detector().lastMask();
            const cv::Rect& roi = pipeline.detector().lastRoi();
            if (!mask.empty() && roi.area() > 0 && (roi & cv::Rect(0, 0, canvas.cols, canvas.rows)).area() == roi.area()) {
                cv::Mat mask_full = cv::Mat::zeros(canvas.size(), CV_8UC1);
                cv::Mat dst = mask_full(roi);
                mask.copyTo(dst);
                std::snprintf(name, sizeof(name), "%s/mask_%06llu.png", dump_dir.c_str(),
                              static_cast<unsigned long long>(pipeline.stats().frames));
                cv::imwrite(name, mask_full);
            }
        }

#ifdef DART_VISION_WITH_MEDIA
        if (recording) recorder.pushFrame(frame.bgr);

        if (config.show) {
            int key = -1;
            try {
                cv::imshow("Dart Vision", canvas);
                key = cv::waitKey(1);
            } catch (const cv::Exception& err) {
                // 无显示环境（headless 服务器 / 没配 DISPLAY 的 Ubuntu）时不要崩，
                // 直接退化成无头运行
                std::cerr << "[显示] 打开窗口失败，切换为无头运行: " << err.what() << std::endl;
                config.show = false;
            }
            if (key == 27 || key == 'q' || key == 'Q') {
                running = false;
            } else if (key == ' ') {
                pipeline.setArmed(!pipeline.armed());
                std::cout << "[控制] " << (pipeline.armed() ? "上电" : "断电") << std::endl;
            } else if (key == 'f' || key == 'F') {
                sender.sendFire();
                std::cout << "[控制] FIRE" << std::endl;
            } else if (key == 'r' || key == 'R') {
                if (recording) {
                    recorder.stop();
                    recording = false;
                } else {
                    recording = recorder.start();
                }
            } else if (key == 'z' || key == 'Z') {
                const bool enable = !pipeline.lastWindow().zoomed;
                pipeline.setZoomEnabled(enable);
                std::cout << "[控制] 变焦 " << (enable ? "开" : "关") << std::endl;
            } else if (key == 's' || key == 'S') {
                if (!dump_dir.empty()) {
                    char name[512];
                    std::snprintf(name, sizeof(name), "%s/frame_%06llu.png", dump_dir.c_str(),
                                  static_cast<unsigned long long>(snap_index));
                    cv::imwrite(name, canvas);
                    const cv::Mat& mask = pipeline.detector().lastMask();
                    if (!mask.empty()) {
                        std::snprintf(name, sizeof(name), "%s/mask_%06llu.png",
                                      dump_dir.c_str(),
                                      static_cast<unsigned long long>(snap_index));
                        cv::imwrite(name, mask);
                    }
                    std::cout << "[调试] 已保存快照 " << snap_index << std::endl;
                    ++snap_index;
                }
            }
        }
#endif

            const dart_vision::PipelineStats& stats = pipeline.stats();

        if (telemetry) {
            const dart_vision::Detection& det = pipeline.lastDetection();
            const dart_vision::AimSolution& sol = pipeline.lastSolution();
            const dart_vision::RoiTracker::Window& win = pipeline.lastWindow();
            const dart_vision::ControlOutput& ctl = pipeline.lastControl();
            const dart_vision::DartState& state = pipeline.lastState();
            const char* phase =
                !pipeline.armed() ? "off" : (win.zoomed ? "zoom" : "seek");
            std::fprintf(telemetry,
                         "%llu,%lld,%s,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,"
                         "%.4f,%.4f,%.3f,%d,%d,%llu,"
                         "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%d,%.1f,%d,%.3f,"
                         "%lld,%.5f,%.5f,%.5f\n",
                         static_cast<unsigned long long>(stats.frames),
                         static_cast<long long>(frame.t_ms), phase, det.found ? 1 : 0,
                         det.blob.x, det.blob.y, det.blob.w, det.blob.h, det.blob.area,
                         static_cast<double>(det.blob.centerX()),
                         static_cast<double>(det.blob.centerY()),
                         static_cast<double>(sol.yaw) * dart_vision::kRadToDeg,
                         static_cast<double>(sol.pitch) * dart_vision::kRadToDeg,
                         static_cast<double>(sol.yaw_rate) * dart_vision::kRadToDeg,
                         static_cast<double>(sol.pitch_rate) * dart_vision::kRadToDeg,
                         static_cast<double>(sol.yaw_pred) * dart_vision::kRadToDeg,
                         static_cast<double>(sol.pitch_pred) * dart_vision::kRadToDeg,
                         static_cast<double>(sol.distance_m),
                         pipeline.lastFromFallback() ? 1 : 0, win.zoomed ? 1 : 0,
                         static_cast<unsigned long long>(stats.sent),
                         ctl.active ? 1 : 0,
                         ctl.pitch * dart_vision::kRadToDeg,
                         ctl.yaw * dart_vision::kRadToDeg,
                         ctl.accel_cmd, ctl.available_accel, ctl.aim_error_deg, ctl.range_m,
                         ctl.range_valid ? 1 : 0, ctl.los_rate_dps,
                         state.valid ? 1 : 0,
                         std::sqrt(static_cast<double>(state.velocity.dot(state.velocity))),
                         static_cast<long long>(state.t_ms),
                         static_cast<double>(ctl.los_world[0]),
                         static_cast<double>(ctl.los_world[1]),
                         static_cast<double>(ctl.los_world[2]));
        }

        if (config.log_every > 0 && stats.frames % config.log_every == 0) {
            const dart_vision::Detection& det = pipeline.lastDetection();
            const dart_vision::AimSolution& sol = pipeline.lastSolution();
            const dart_vision::RoiTracker::Window& win = pipeline.lastWindow();
            char zoom_text[48] = "";
            if (win.zoomed) {
                std::snprintf(zoom_text, sizeof(zoom_text), " crop=(%d,%d %dx%d)", win.crop.x,
                              win.crop.y, win.crop.width, win.crop.height);
            }
            std::printf(
                "[%6llu] fps=%5.1f %s %s blob=(%4d,%4d %2dx%-2d area=%4d) yaw=%7.3f "
                "pitch=%7.3f dist=%5.2f 处理=%.2fms%s",
                static_cast<unsigned long long>(stats.frames), stats.fps,
                pipeline.armed() ? (pipeline.lastWindow().zoomed ? "zoom" : "seek") : "off ",
                det.found ? "det" : "---", det.blob.x, det.blob.y, det.blob.w, det.blob.h,
                det.blob.area, static_cast<double>(sol.yaw) * dart_vision::kRadToDeg,
                static_cast<double>(sol.pitch) * dart_vision::kRadToDeg,
                static_cast<double>(sol.distance_m), stats.process_ms, zoom_text);
            // 控制链诊断：需求加速度 / 可用加速度（分母）/ 实发舵角 / 门限距离。
            // "需求远小于可用"= 增益太小；"实发舵角贴住 20°"= 已经饱和，只能动物理参数。
            if (config.control.enabled) {
                const dart_vision::ControlOutput& ctl = pipeline.lastControl();
                const dart_vision::DartState& state = pipeline.lastState();
                std::printf(
                    " | ctl a=%5.2f av=%5.2f m/s2 pch=%6.2f yaw=%6.2f° lam=%6.1f°/s "
                    "ae=%5.2f° R=%5.2f%s state=%s v=%5.1f\n",
                    ctl.accel_cmd, ctl.available_accel,
                    ctl.pitch * dart_vision::kRadToDeg, ctl.yaw * dart_vision::kRadToDeg,
                    ctl.los_rate_dps, ctl.aim_error_deg, ctl.range_m,
                    ctl.range_valid ? "" : "?", state.valid ? "on" : "--",
                    std::sqrt(static_cast<double>(state.velocity.dot(state.velocity))));
            } else {
                std::printf("\n");
            }
            if (pipeline.lastFromFallback()) std::printf("   <- 整幅回退救回\n");
            std::fflush(stdout);
        }

        if (config.exit_after_ms > 0 && frame.t_ms > config.exit_after_ms) {
            std::cout << "[运行] 达到 --duration-ms 上限，退出" << std::endl;
            break;
        }
        if (exit_after_frames > 0 && stats.frames >= exit_after_frames) break;
    }

#ifdef DART_VISION_WITH_MEDIA
    if (recording) recorder.stop();
    if (config.show) cv::destroyAllWindows();
#endif

    if (telemetry) {
        std::fclose(telemetry);
        telemetry = nullptr;
    }

    std::cout << "[统计] " << pipeline.stats().summary() << std::endl;
    if (config.send) {
        std::cout << "[回发] 发送=" << sender.sent() << " 合并丢弃=" << sender.dropped()
                  << " 失败=" << sender.failed() << std::endl;
    }
    return 0;
}
