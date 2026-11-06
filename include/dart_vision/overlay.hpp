#ifndef DART_VISION_OVERLAY_HPP
#define DART_VISION_OVERLAY_HPP

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

#include "dart_vision/roi_tracker.hpp"
#include "dart_vision/types.hpp"

namespace dart_vision {

// 对应 main_final.py 的 draw_overlay()：把 ROI、命中框、FPS、时间戳画在图上
struct OverlayState {
    bool armed = true;
    bool zoomed = false;
    double zoom = 1.0;
    cv::Rect crop;          // 变焦窗口（原帧坐标）
    cv::Rect tracked_roi;   // cur_roi（原帧坐标）
    Detection detection;    // 原帧坐标
    AimSolution solution;
    double fps = 0.0;
    double process_ms = 0.0;
    int64_t t_ms = 0;
    uint64_t frame_index = 0;
    bool recording = false;
    std::string sender_state;
};

void drawOverlay(cv::Mat& canvas, const OverlayState& state);

}  // namespace dart_vision

#endif  // DART_VISION_OVERLAY_HPP
