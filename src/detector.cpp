#include "dart_vision/detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace dart_vision {

namespace {

// OpenMV 的 Lab 阈值语义 -> OpenCV 8U Lab：
//   L: 0~100   -> 0~255   (乘 2.55)
//   A: -128~127 -> 0~255  (加 128)
//   B: -128~127 -> 0~255  (加 128)
cv::Scalar openmvToCv(const std::array<int, 6>& t, bool lower) {
    const double l = lower ? t[0] : t[1];
    const double a = (lower ? t[2] : t[3]) + 128.0;
    const double b = (lower ? t[4] : t[5]) + 128.0;
    const double l_cv = std::max(0.0, std::min(255.0, std::round(l * 255.0 / 100.0)));
    return cv::Scalar(l_cv, a, b);
}

}  // namespace

LabDetector::LabDetector(const DetectorConfig& cfg) : cfg_(cfg) {
    for (std::size_t i = 0; i < cfg_.thresholds.size(); ++i) {
        lo_.push_back(openmvToCv(cfg_.thresholds[i], true));
        hi_.push_back(openmvToCv(cfg_.thresholds[i], false));
    }
}

Detection LabDetector::detect(const cv::Mat& bgr, const cv::Rect& roi) {
    Detection det;
    mask_.release();
    last_roi_ = cv::Rect();

    if (bgr.empty()) return det;

    const cv::Rect bounds(0, 0, bgr.cols, bgr.rows);
    const cv::Rect work = roi & bounds;
    if (work.width <= 0 || work.height <= 0) return det;
    last_roi_ = work;

    cv::Mat view = bgr(work);
    if (cfg_.blur_kernel >= 3) {
        int k = cfg_.blur_kernel | 1;  // 保证奇数
        cv::GaussianBlur(view, view, cv::Size(k, k), 0);
    }

    cv::Mat lab;
    cv::cvtColor(view, lab, cv::COLOR_BGR2Lab);

    if (lo_.empty()) return det;
    cv::Mat acc = cv::Mat::zeros(lab.size(), CV_8UC1);
    for (std::size_t i = 0; i < lo_.size(); ++i) {
        cv::Mat one;
        cv::inRange(lab, lo_[i], hi_[i], one);
        cv::bitwise_or(acc, one, acc);
    }
    mask_ = acc;

    if (cfg_.open_kernel > 0) {
        cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT,
                                             cv::Size(cfg_.open_kernel, cfg_.open_kernel));
        cv::morphologyEx(mask_, mask_, cv::MORPH_OPEN, k);
    }
    if (cfg_.close_kernel > 0) {
        cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT,
                                             cv::Size(cfg_.close_kernel, cfg_.close_kernel));
        cv::morphologyEx(mask_, mask_, cv::MORPH_CLOSE, k);
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int count =
        cv::connectedComponentsWithStats(mask_, labels, stats, centroids, 8, CV_32S);

    int best = -1;
    int best_area = -1;
    for (int label = 1; label < count; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < cfg_.min_area_px) continue;
        if (cfg_.max_area_px > 0 && area > cfg_.max_area_px) continue;
        if (area > best_area) {
            best_area = area;
            best = label;
        }
    }
    if (best < 0) return det;

    det.found = true;
    det.blob.x = work.x + stats.at<int>(best, cv::CC_STAT_LEFT);
    det.blob.y = work.y + stats.at<int>(best, cv::CC_STAT_TOP);
    det.blob.w = stats.at<int>(best, cv::CC_STAT_WIDTH);
    det.blob.h = stats.at<int>(best, cv::CC_STAT_HEIGHT);
    det.blob.area = best_area;
    // 全幅检测时（含整幅回退路径）没有变焦可借，亚像素尺寸就等于整型尺寸
    det.blob.wf = static_cast<float>(det.blob.w);
    det.blob.hf = static_cast<float>(det.blob.h);
    det.blob.cx = static_cast<float>(work.x + centroids.at<double>(best, 0));
    det.blob.cy = static_cast<float>(work.y + centroids.at<double>(best, 1));
    det.score = static_cast<double>(best_area) /
                std::max(1.0, static_cast<double>(work.width) * work.height);
    return det;
}

}  // namespace dart_vision
