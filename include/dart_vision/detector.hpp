#ifndef DART_VISION_DETECTOR_HPP
#define DART_VISION_DETECTOR_HPP

#include <opencv2/core.hpp>

#include <vector>

#include "dart_vision/config.hpp"
#include "dart_vision/types.hpp"

namespace dart_vision {

// 移植自 main_final.py 的 find_max_blob()：
//     blobs = img.find_blobs([THRESHOLDS1[0]], roi=roi)  -> 取面积最大的 blob
// 差别只在于这里用 OpenCV 实现同一套 Lab 阈值 + 连通域，语义保持一致：
//   * 阈值仍按 OpenMV 的 Lab 语义书写（L: 0~100，A/B: -128~127）
//   * blob 面积 = 像素个数（blob.pixels()），不是轮廓面积
//   * blob 矩形 = 连通域外接矩形（blob.rect()）
class LabDetector {
public:
    explicit LabDetector(const DetectorConfig& cfg);

    // bgr: 待处理图（可能是变焦放大后的处理窗）
    // roi: 在 bgr 坐标系下的检测区域，对应脚本里的 cur_roi
    // 返回 blob 坐标同样在 bgr 坐标系下
    Detection detect(const cv::Mat& bgr, const cv::Rect& roi);

    const DetectorConfig& config() const { return cfg_; }

    // 调试用：最近一次的阈值掩码（位于 lastRoi() 坐标系内）
    const cv::Mat& lastMask() const { return mask_; }
    const cv::Rect& lastRoi() const { return last_roi_; }

private:
    DetectorConfig cfg_;
    cv::Mat mask_;
    cv::Rect last_roi_;
    // OpenMV Lab -> OpenCV 8U Lab 的查表缓存
    std::vector<cv::Scalar> lo_;
    std::vector<cv::Scalar> hi_;
};

}  // namespace dart_vision

#endif  // DART_VISION_DETECTOR_HPP
