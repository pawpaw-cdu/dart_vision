#include "dart_vision/roi_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace dart_vision {

RoiTracker::RoiTracker(const TrackerConfig& cfg) : cfg_(cfg) {
    armed_ = cfg_.armed;
    if (cfg_.zoom_factor < 1.0) cfg_.zoom_factor = 1.0;
}

void RoiTracker::reset() {
    zoomed_ = false;
    suppress_zoom_ = false;
    has_center_ = false;
    lost_frames_ = 0;
    tracked_roi_ = cv::Rect();
}

void RoiTracker::setArmed(bool armed) {
    armed_ = armed;
    if (!armed_) reset();
}

bool RoiTracker::beginFrame(const cv::Size& frame_size, int64_t t_ms, Window* window) {
    if (!window) return false;
    last_size_ = frame_size;
    const cv::Rect full(0, 0, frame_size.width, frame_size.height);
    if (tracked_roi_.area() <= 0) tracked_roi_ = full;  // INIT_ROI

    if (!armed_) return false;
    if (t_ms < static_cast<int64_t>(cfg_.start_delay_ms)) return false;  // T_START

    // 第二阶段：只在对目标已有位置估计、且到达切换时间后进入
    zoomed_ = cfg_.zoom_enable && has_center_ && !suppress_zoom_ &&
              (t_ms >= static_cast<int64_t>(cfg_.zoom_after_ms));

    if (zoomed_) {
        const double z = std::max(1.0, cfg_.zoom_factor);
        const int cw = std::max(8, static_cast<int>(std::lround(frame_size.width / z)));
        const int ch = std::max(8, static_cast<int>(std::lround(frame_size.height / z)));
        int x = static_cast<int>(std::lround(center_.x - cw * 0.5));
        int y = static_cast<int>(std::lround(center_.y - ch * 0.5));
        x = std::max(0, std::min(frame_size.width - cw, x));
        y = std::max(0, std::min(frame_size.height - ch, y));
        window->crop = cv::Rect(x, y, cw, ch);
        window->zoom = z;
        window->zoomed = true;
        // 处理图是 crop 放大到整幅尺寸后的结果，因此搜索区域就是整幅处理图
        window->detect_roi = cv::Rect(0, 0, frame_size.width, frame_size.height);
        return true;
    }

    window->crop = full;
    window->zoom = 1.0;
    window->zoomed = false;
    window->detect_roi = tracked_roi_ & full;
    return window->detect_roi.area() > 0;
}

void RoiTracker::update(const Detection& det, const cv::Size& frame_size) {
    const cv::Rect full(0, 0, frame_size.width, frame_size.height);
    last_size_ = frame_size;

    if (det.found && det.blob.valid()) {
        lost_frames_ = 0;
        has_center_ = true;
        center_ = cv::Point2f(det.blob.centerX(), det.blob.centerY());
        suppress_zoom_ = false;  // 重新拿到目标后允许再次变焦

        // cur_roi = build_found_roi(blob)：命中矩形按 FOUND_RANGE 外扩
        cv::Rect roi(det.blob.x - cfg_.found_range_x, det.blob.y - cfg_.found_range_y,
                     det.blob.w + 2 * cfg_.found_range_x,
                     det.blob.h + 2 * cfg_.found_range_y);
        tracked_roi_ = roi & full;
        if (tracked_roi_.area() <= 0) tracked_roi_ = full;
        return;
    }

    ++lost_frames_;

    if (zoomed_ && lost_frames_ >= std::max(1, cfg_.zoom_lost_frames)) {
        // 变焦窗口内丢失，退回整幅搜索
        zoomed_ = false;
        suppress_zoom_ = true;
        has_center_ = false;
    }

    if (lost_frames_ >= std::max(1, cfg_.lost_reset_frames)) {
        tracked_roi_ = full;  // cur_roi = INIT_ROI
    }
}

Blob RoiTracker::toFrameCoords(const Blob& blob, const Window& window) {
    Blob out = blob;
    if (window.zoom <= 0.0) return out;
    out.x = window.crop.x + static_cast<int>(std::lround(blob.x / window.zoom));
    out.y = window.crop.y + static_cast<int>(std::lround(blob.y / window.zoom));
    const int w = std::max(1, static_cast<int>(std::lround(blob.w / window.zoom)));
    const int h = std::max(1, static_cast<int>(std::lround(blob.h / window.zoom)));
    out.w = w;
    out.h = h;
    // 亚像素尺寸：不要 lround。变焦时包围盒在放大图上的分辨率是 1/zoom 像素
    // （例如 zoom=4、放大图 9x9 -> 整帧 2.25x2.25），取整会把这份精度直接丢掉，
    // 而单目测距是拿它当分母的：1 个像素的取整误差 = 20~25% 的距离误差。
    out.wf = static_cast<float>(blob.w) / window.zoom;
    out.hf = static_cast<float>(blob.h) / window.zoom;
    out.cx = static_cast<float>(window.crop.x + blob.cx / window.zoom);
    out.cy = static_cast<float>(window.crop.y + blob.cy / window.zoom);
    return out;
}

cv::Rect RoiTracker::toFrameRect(const cv::Rect& rect, const Window& window) {
    if (window.zoom <= 0.0) return rect;
    return cv::Rect(window.crop.x + static_cast<int>(std::lround(rect.x / window.zoom)),
                    window.crop.y + static_cast<int>(std::lround(rect.y / window.zoom)),
                    std::max(1, static_cast<int>(std::lround(rect.width / window.zoom))),
                    std::max(1, static_cast<int>(std::lround(rect.height / window.zoom))));
}

cv::Rect RoiTracker::toWindowRect(const cv::Rect& rect, const Window& window) {
    if (window.zoom <= 0.0) return rect;
    return cv::Rect(static_cast<int>(std::lround((rect.x - window.crop.x) * window.zoom)),
                    static_cast<int>(std::lround((rect.y - window.crop.y) * window.zoom)),
                    std::max(1, static_cast<int>(std::lround(rect.width * window.zoom))),
                    std::max(1, static_cast<int>(std::lround(rect.height * window.zoom))));
}

}  // namespace dart_vision
