#ifndef DART_VISION_ROI_TRACKER_HPP
#define DART_VISION_ROI_TRACKER_HPP

#include <cstdint>

#include <opencv2/core.hpp>

#include "dart_vision/config.hpp"
#include "dart_vision/types.hpp"

namespace dart_vision {

// 对应 main_final.py 里的三段状态逻辑：
//   1) t_ms > T_START 才开始检测
//   2) 命中后 cur_roi = build_found_roi(blob)，丢失后 cur_roi = INIT_ROI
//   3) 第二阶段把读出窗口挪到目标附近（READOUT_ROI_PHASE2），等效于对目标变焦
// 这里把 3) 实现为「裁剪 + 放大」的数字变焦：窗口越小，导向灯占的像素越多，
// 角度解算的量化误差越小。
class RoiTracker {
public:
    struct Window {
        cv::Rect crop;        // 本帧处理窗在原帧中的位置
        double zoom = 1.0;    // 处理图 = crop 放大 zoom 倍
        bool zoomed = false;  // 是否处于第二阶段变焦
        cv::Rect detect_roi;  // 在处理图中做 blob 搜索的区域（对应 cur_roi）
    };

    explicit RoiTracker(const TrackerConfig& cfg);

    void reset();
    void setArmed(bool armed);
    bool armed() const { return armed_; }

    // 每帧开始：给出本帧处理窗口。返回 false 表示本帧不检测（未使能 / 未到 T_START）
    bool beginFrame(const cv::Size& frame_size, int64_t t_ms, Window* window);

    // 检测结束：更新 ROI 状态机（det 使用原帧坐标）
    void update(const Detection& det, const cv::Size& frame_size);

    // 处理窗坐标 -> 原帧坐标
    static Blob toFrameCoords(const Blob& blob, const Window& window);
    static cv::Rect toFrameRect(const cv::Rect& rect, const Window& window);
    // 原帧坐标 -> 处理窗坐标
    static cv::Rect toWindowRect(const cv::Rect& rect, const Window& window);

    const cv::Rect& trackedRoi() const { return tracked_roi_; }
    bool zoomed() const { return zoomed_; }
    int lostFrames() const { return lost_frames_; }
    bool hasCenter() const { return has_center_; }

private:
    TrackerConfig cfg_;
    bool armed_ = true;
    bool zoomed_ = false;
    bool suppress_zoom_ = false;
    bool has_center_ = false;
    int lost_frames_ = 0;
    cv::Point2f center_{0.0f, 0.0f};   // 最近一次命中的目标中心（原帧坐标）
    cv::Size last_size_;
    cv::Rect tracked_roi_;             // cur_roi
};

}  // namespace dart_vision

#endif  // DART_VISION_ROI_TRACKER_HPP
