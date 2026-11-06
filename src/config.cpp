#include "dart_vision/config.hpp"
#include "dart_vision/types.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace dart_vision {

namespace {

template <typename T>
void readScalar(const YAML::Node& node, const char* key, T& out) {
    if (!node || !node.IsMap() || !node[key]) return;
    try {
        out = node[key].as<T>();
    } catch (const std::exception&) {
        // 字段类型不对时保留默认值
    }
}

void readString(const YAML::Node& node, const char* key, std::string& out) {
    if (!node || !node.IsMap() || !node[key]) return;
    try {
        out = node[key].as<std::string>();
    } catch (const std::exception&) {
    }
}

}  // namespace

void CameraConfig::finalize() {
    if (width <= 0) width = 640;
    if (height <= 0) height = 480;

    // 与 simulator_system/TUTORIAL.md 的换算一致：
    //   f = (高度或宽度 / 2) / tan(fov / 2)
    if (fx <= 0.0 || fy <= 0.0) {
        const double half = fov_deg * 0.5 * kDegToRad;
        const double t = std::tan(half);
        double f = 0.0;
        if (t > 1e-9) {
            if (fov_axis == "horizontal") {
                f = (width * 0.5) / t;
            } else {
                f = (height * 0.5) / t;
            }
        }
        if (f <= 0.0) f = 1.0;
        if (fx <= 0.0) fx = f;
        if (fy <= 0.0) fy = f;
    }
    if (cx <= 0.0) cx = width * 0.5;
    if (cy <= 0.0) cy = height * 0.5;
}

AppConfig AppConfig::load(const std::string& path, std::string* warning) {
    AppConfig cfg;
    cfg.config_path = path;

    // 默认指向深圳大学开源视觉使用的绿色导向灯阈值
    cfg.detector.thresholds.push_back({{9, 100, -102, -25, 0, 127}});

    YAML::Node root;
    bool loaded = false;
    try {
        root = YAML::LoadFile(path);
        loaded = root.IsMap();
    } catch (const std::exception& err) {
        if (warning) *warning = std::string("读取配置失败，使用默认值: ") + err.what();
    }
    if (!loaded) {
        cfg.camera.finalize();
        if (cfg.command.host.empty()) cfg.command.host = cfg.source.host;
        return cfg;
    }

    // 顶层 host / port 同时给 sim_receiver 使用
    readString(root, "host", cfg.source.host);
    readScalar(root, "port", cfg.source.port);

    if (root["source"]) {
        const YAML::Node& n = root["source"];
        readString(n, "mode", cfg.source.mode);
        readString(n, "host", cfg.source.host);
        readScalar(n, "port", cfg.source.port);
        readString(n, "video_path", cfg.source.video_path);
        readScalar(n, "loop", cfg.source.loop);
        readScalar(n, "realtime", cfg.source.realtime);
    }

    if (root["command"]) {
        const YAML::Node& n = root["command"];
        readScalar(n, "enabled", cfg.command.enabled);
        readString(n, "host", cfg.command.host);
        readScalar(n, "port", cfg.command.port);
        readString(n, "protocol", cfg.command.protocol);
        readScalar(n, "text_verbose", cfg.command.text_verbose);
        readScalar(n, "max_rate_hz", cfg.command.max_rate_hz);
        readScalar(n, "reconnect_ms", cfg.command.reconnect_ms);
    }
    if (cfg.command.host.empty()) cfg.command.host = cfg.source.host;

    if (root["camera"]) {
        const YAML::Node& n = root["camera"];
        readScalar(n, "width", cfg.camera.width);
        readScalar(n, "height", cfg.camera.height);
        readScalar(n, "fov_deg", cfg.camera.fov_deg);
        readString(n, "fov_axis", cfg.camera.fov_axis);
        readScalar(n, "fx", cfg.camera.fx);
        readScalar(n, "fy", cfg.camera.fy);
        readScalar(n, "cx", cfg.camera.cx);
        readScalar(n, "cy", cfg.camera.cy);
    }

    if (root["detector"]) {
        const YAML::Node& n = root["detector"];
        readScalar(n, "min_area_px", cfg.detector.min_area_px);
        readScalar(n, "max_area_px", cfg.detector.max_area_px);
        readScalar(n, "blur_kernel", cfg.detector.blur_kernel);
        readScalar(n, "open_kernel", cfg.detector.open_kernel);
        readScalar(n, "close_kernel", cfg.detector.close_kernel);

        const YAML::Node& list = n["thresholds"];
        if (list && list.IsSequence() && list.size() > 0) {
            std::vector<std::array<int, 6>> parsed;
            for (std::size_t i = 0; i < list.size(); ++i) {
                const YAML::Node& item = list[i];
                if (!item.IsSequence() || item.size() != 6) continue;
                std::array<int, 6> t{};
                for (std::size_t k = 0; k < 6; ++k) t[k] = item[k].as<int>();
                parsed.push_back(t);
            }
            if (!parsed.empty()) cfg.detector.thresholds = parsed;
        }
    }

    if (root["tracker"]) {
        const YAML::Node& n = root["tracker"];
        readScalar(n, "armed", cfg.tracker.armed);
        readScalar(n, "start_delay_ms", cfg.tracker.start_delay_ms);
        readScalar(n, "found_range_x", cfg.tracker.found_range_x);
        readScalar(n, "found_range_y", cfg.tracker.found_range_y);
        readScalar(n, "lost_reset_frames", cfg.tracker.lost_reset_frames);
        readScalar(n, "zoom_enable", cfg.tracker.zoom_enable);
        readScalar(n, "zoom_after_ms", cfg.tracker.zoom_after_ms);
        readScalar(n, "zoom_factor", cfg.tracker.zoom_factor);
        readScalar(n, "zoom_lost_frames", cfg.tracker.zoom_lost_frames);
    }

    if (root["solver"]) {
        const YAML::Node& n = root["solver"];
        readString(n, "axis", cfg.solver.axis);
        readScalar(n, "light_diameter_m", cfg.solver.light_diameter_m);
        readScalar(n, "rate_window", cfg.solver.rate_window);
        readScalar(n, "predict_horizon_ms", cfg.solver.predict_horizon_ms);
        readScalar(n, "rate_reset_gap_ms", cfg.solver.rate_reset_gap_ms);
        readScalar(n, "rate_min_samples", cfg.solver.rate_min_samples);
        readScalar(n, "rate_limit_dps", cfg.solver.rate_limit_dps);
    }

    if (root["control"]) {
        const YAML::Node& n = root["control"];
        readScalar(n, "enabled", cfg.control.enabled);
        readScalar(n, "pn_gain", cfg.control.pn_gain);
        readScalar(n, "accel_bias_mps2", cfg.control.accel_bias_mps2);
        readScalar(n, "aim_error_gain", cfg.control.aim_error_gain);
        readScalar(n, "zem_gain", cfg.control.zem_gain);
        readScalar(n, "nose_lead_m", cfg.control.nose_lead_m);
        readScalar(n, "availability_mps2", cfg.control.availability_mps2);
        readScalar(n, "reference_speed_mps", cfg.control.reference_speed_mps);
        readScalar(n, "max_fin_deg", cfg.control.max_fin_deg);
        readScalar(n, "engage_range_m", cfg.control.engage_range_m);
        readScalar(n, "min_range_m", cfg.control.min_range_m);
        readScalar(n, "light_diameter_m", cfg.control.light_diameter_m);
        readScalar(n, "focal_px", cfg.control.focal_px);
        readScalar(n, "aim_offset_up_m", cfg.control.aim_offset_up_m);
        readScalar(n, "aim_offset_forward_m", cfg.control.aim_offset_forward_m);
        readScalar(n, "rate_filter_secs", cfg.control.rate_filter_secs);
        readScalar(n, "max_sample_jump_deg", cfg.control.max_sample_jump_deg);
        readScalar(n, "range_min_blob_px", cfg.control.range_min_blob_px);
        readScalar(n, "engage_ramp_secs", cfg.control.engage_ramp_secs);
        readScalar(n, "engage_after_launch_secs", cfg.control.engage_after_launch_secs);
        readScalar(n, "roll_gain", cfg.control.roll_gain);
        readScalar(n, "pitch_open_loop_deg", cfg.control.pitch_open_loop_deg);
        readScalar(n, "yaw_open_loop_deg", cfg.control.yaw_open_loop_deg);
        readScalar(n, "roll_open_loop_deg", cfg.control.roll_open_loop_deg);
        readScalar(n, "camera_body_fixed", cfg.control.camera_body_fixed);
    }

    if (root["guidance"]) {
        const YAML::Node& n = root["guidance"];
        readScalar(n, "enabled", cfg.guidance.enabled);
        readString(n, "mode", cfg.guidance.mode);
        readScalar(n, "k_heading", cfg.guidance.k_heading);
        readScalar(n, "k_rate", cfg.guidance.k_rate);
        readScalar(n, "pn_gain", cfg.guidance.pn_gain);
        readScalar(n, "closing_speed_mps", cfg.guidance.closing_speed_mps);
        readScalar(n, "lateral_accel_limit", cfg.guidance.lateral_accel_limit);
        readScalar(n, "gravity_turn_comp", cfg.guidance.gravity_turn_comp);
        readScalar(n, "gravity_mps2", cfg.guidance.gravity_mps2);
        readScalar(n, "arm_delay_ms", cfg.guidance.arm_delay_ms);
        readScalar(n, "arm_on_motion_deg", cfg.guidance.arm_on_motion_deg);
        readScalar(n, "max_cmd_deg", cfg.guidance.max_cmd_deg);
        readScalar(n, "deadband_deg", cfg.guidance.deadband_deg);
        readScalar(n, "bias_yaw_deg", cfg.guidance.bias_yaw_deg);
        readScalar(n, "bias_pitch_deg", cfg.guidance.bias_pitch_deg);
    }

    if (root["record"]) {
        const YAML::Node& n = root["record"];
        readScalar(n, "enabled", cfg.record.enabled);
        readScalar(n, "fps", cfg.record.fps);
        readScalar(n, "width", cfg.record.width);
        readScalar(n, "height", cfg.record.height);
    }

    cfg.camera.finalize();
    return cfg;
}

std::string AppConfig::describe() const {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << "source=" << source.mode << " " << source.host << ":" << source.port;
    if (source.mode == "video") oss << " video=" << source.video_path;
    oss << "\n  command=" << (send ? "on" : "off") << " " << command.host << ":"
        << command.port << " protocol=" << command.protocol
        << " max_rate=" << command.max_rate_hz << "Hz";
    oss << "\n  camera=" << camera.width << "x" << camera.height << " fov=" << camera.fov_deg
        << "(" << camera.fov_axis << ") fx=" << camera.fx << " fy=" << camera.fy
        << " cx=" << camera.cx << " cy=" << camera.cy;
    oss << "\n  thresholds=";
    for (std::size_t i = 0; i < detector.thresholds.size(); ++i) {
        const std::array<int, 6>& t = detector.thresholds[i];
        if (i) oss << " | ";
        oss << "[" << t[0] << "," << t[1] << "," << t[2] << "," << t[3] << "," << t[4] << ","
            << t[5] << "]";
    }
    oss << " min_area=" << detector.min_area_px << "px";
    oss << "\n  tracker start_delay=" << tracker.start_delay_ms
        << "ms roi+/-(" << tracker.found_range_x << "," << tracker.found_range_y << ")"
        << " zoom=" << (tracker.zoom_enable ? "on" : "off") << " after=" << tracker.zoom_after_ms
        << "ms x" << tracker.zoom_factor;
    oss << "\n  solver axis=" << solver.axis << " light_d=" << solver.light_diameter_m
        << "m predict=" << solver.predict_horizon_ms << "ms";
    oss << "\n  control=" << (control.enabled ? "on" : "off") << " N=" << control.pn_gain
        << " scale=" << control.availability_mps2 << "m/s2@" << control.reference_speed_mps
        << "m/s max_fin=" << control.max_fin_deg << "deg gate=" << control.engage_range_m
        << "m aim_up=" << control.aim_offset_up_m << "m tau=" << control.rate_filter_secs
        << "s range_min_blob=" << control.range_min_blob_px << "px ramp="
        << control.engage_ramp_secs << "s engage_t="
        << control.engage_after_launch_secs << "s roll_gain=" << control.roll_gain;
    oss << "\n  guidance=" << (guidance.enabled ? "on" : "off") << " mode=" << guidance.mode;
    if (guidance.mode == "pn") {
        oss << " N=" << guidance.pn_gain << " Vc=" << guidance.closing_speed_mps
            << "m/s A_max=" << guidance.lateral_accel_limit << "m/s2"
            << " grav_comp=" << (guidance.gravity_turn_comp ? "on" : "off")
            << " arm_delay=" << guidance.arm_delay_ms << "ms";
    } else {
        oss << " k_heading=" << guidance.k_heading << " k_rate=" << guidance.k_rate << "s";
    }
    oss << " max_cmd=" << guidance.max_cmd_deg << "deg deadband=" << guidance.deadband_deg
        << "deg bias=(" << guidance.bias_yaw_deg << "," << guidance.bias_pitch_deg << ")deg";
    return oss.str();
}

}  // namespace dart_vision
