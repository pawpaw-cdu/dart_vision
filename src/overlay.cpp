#include "dart_vision/overlay.hpp"

#include <cmath>
#include <cstdio>

#include <opencv2/imgproc.hpp>

namespace dart_vision {

namespace {

void putLine(cv::Mat& canvas, int y, const std::string& text, const cv::Scalar& color,
             double scale = 0.5) {
    cv::putText(canvas, text, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX, scale, color, 1,
                cv::LINE_AA);
}

std::string formatAngle(double rad) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.3fdeg", rad * kRadToDeg);
    return std::string(buffer);
}

}  // namespace

void drawOverlay(cv::Mat& canvas, const OverlayState& state) {
    if (canvas.empty()) return;

    const cv::Scalar kBlue(255, 100, 0);
    const cv::Scalar kRed(0, 0, 255);
    const cv::Scalar kGreen(0, 255, 0);
    const cv::Scalar kYellow(0, 255, 255);
    const cv::Scalar kWhite(255, 255, 255);
    const cv::Scalar kCyan(255, 255, 0);

    // cur_roi（对应脚本里画蓝框的 cur_roi）
    if (state.tracked_roi.area() > 0) {
        cv::rectangle(canvas, state.tracked_roi, kBlue, 1);
    }

    // 变焦窗口（对应第二阶段挪动后的读出窗口）
    if (state.zoomed && state.crop.area() > 0) {
        cv::rectangle(canvas, state.crop, kCyan, 1);
        cv::putText(canvas, "ZOOM", cv::Point(state.crop.x + 2, state.crop.y + 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, kCyan, 1, cv::LINE_AA);
    }

    // 光轴中心：目标中心与它的偏差就是解算出来的 yaw/pitch
    const cv::Point center(canvas.cols / 2, canvas.rows / 2);
    cv::line(canvas, cv::Point(center.x - 12, center.y), cv::Point(center.x + 12, center.y),
             kYellow, 1);
    cv::line(canvas, cv::Point(center.x, center.y - 12), cv::Point(center.x, center.y + 12),
             kYellow, 1);

    if (state.detection.found && state.detection.blob.valid()) {
        const Blob& b = state.detection.blob;
        cv::rectangle(canvas, cv::Point(b.x, b.y), cv::Point(b.x + b.w, b.y + b.h), kRed, 1);
        const cv::Point aim(static_cast<int>(std::lround(b.centerX())),
                            static_cast<int>(std::lround(b.centerY())));
        cv::drawMarker(canvas, aim, kRed, cv::MARKER_CROSS, 8, 1);
        cv::line(canvas, center, aim, kRed, 1, cv::LINE_AA);

        char line[128];
        std::snprintf(line, sizeof(line), "Found! X=%d Y=%d area=%d", b.x + b.w / 2,
                      b.y + b.h / 2, b.area);
        putLine(canvas, 40, line, kGreen);
    }

    char text[160];
    std::snprintf(text, sizeof(text), "FPS:%.1f", state.fps);
    putLine(canvas, 16, text, kRed);
    std::snprintf(text, sizeof(text), "t=%lld ms  frame=%llu  %.2f ms/f",
                  static_cast<long long>(state.t_ms),
                  static_cast<unsigned long long>(state.frame_index), state.process_ms);
    putLine(canvas, 28, text, kYellow);

    const char* phase = !state.armed ? "DISARMED" : (state.zoomed ? "TRACK/zoom" : "SEARCH");
    std::snprintf(text, sizeof(text), "phase=%s zoom=x%.1f roi=%d,%d %dx%d", phase, state.zoom,
                  state.tracked_roi.x, state.tracked_roi.y, state.tracked_roi.width,
                  state.tracked_roi.height);
    putLine(canvas, 52, text, kWhite);

    if (state.solution.valid) {
        std::snprintf(text, sizeof(text), "yaw=%s pitch=%s dist=%.2fm axis=%s",
                      formatAngle(state.solution.yaw).c_str(),
                      formatAngle(state.solution.pitch).c_str(),
                      static_cast<double>(state.solution.distance_m),
                      axisName(state.solution.axis));
        putLine(canvas, 64, text, kGreen);
    }

    if (!state.sender_state.empty()) {
        putLine(canvas, canvas.rows - 8, state.sender_state, kWhite);
    }

    if (state.recording) {
        cv::circle(canvas, cv::Point(canvas.cols - 30, 30), 8, kRed, -1);
        cv::putText(canvas, "REC", cv::Point(canvas.cols - 70, 36), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    kRed, 2, cv::LINE_AA);
    }
}

}  // namespace dart_vision
