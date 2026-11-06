#ifndef DART_VISION_TYPES_HPP
#define DART_VISION_TYPES_HPP

#include <cstdint>
#include <string>
#include <opencv2/core.hpp>

namespace dart_vision {

// 自带圆周率常量：不依赖 M_PI（glibc 在某些严格 ANSI 模式下不导出 M_PI，
// macOS 与 Linux 行为不一致，这里统一用工程内常量）
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// 一帧图像 = main_final.py 里的一次 sensor.snapshot()
struct Frame {
    cv::Mat bgr;
    uint64_t seq = 0;         // 图像源帧序号（video 源为解码帧号）
    uint64_t timestamp_us = 0;// 图像源时间戳（video 源为 0）
    int64_t t_ms = 0;         // 收到该帧时的本机毫秒时间戳，对应脚本里的 t_ms
};

// 与 OpenMV image.find_blobs() 返回的 blob 对应，坐标一律是整帧像素坐标
struct Blob {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
    int area = 0;   // 像素个数，与 blob.pixels() 一致（不是轮廓面积）
    float cx = 0.0f;// 像素质心 x
    float cy = 0.0f;// 像素质心 y
    // 亚像素尺寸（整帧像素）：变焦后包围盒的分辨率是 1/zoom 像素，
    // 原来的 w/h 是整型、反算时被 lround 掉，等于把变焦白给的精度扔了。
    // 这两个保留小数，专门给单目测距用。0 = 未设置，回退到 w/h。
    float wf = 0.0f;
    float hf = 0.0f;

    bool valid() const { return w > 0 && h > 0; }
    float centerX() const { return x + w * 0.5f; }  // 与脚本 x + w/2 一致
    float centerY() const { return y + h * 0.5f; }
    // 测距用的表观尺寸：优先用亚像素值
    float apparentSize() const {
        return (wf > 0.0f && hf > 0.0f) ? 0.5f * (wf + hf) : 0.5f * (w + h);
    }
};

struct Detection {
    bool found = false;
    Blob blob;
    double score = 0.0;  // 最大 blob 面积 / 检测 ROI 面积，仅用于调试
};

// 解算轴：对应主控下发的 0x11(yaw) / 0x22(pitch) 模式
enum AimAxis {
    kAxisBoth = 0,
    kAxisYaw = 1,
    kAxisPitch = 2,
};

inline const char* axisName(int axis) {
    switch (axis) {
        case kAxisYaw: return "yaw";
        case kAxisPitch: return "pitch";
        default: return "both";
    }
}

inline int axisFromString(const std::string& value) {
    if (value == "yaw") return kAxisYaw;
    if (value == "pitch") return kAxisPitch;
    return kAxisBoth;
}

struct AimSolution {
    bool valid = false;
    int axis = kAxisBoth;
    float u = 0.0f;          // 目标中心像素
    float v = 0.0f;
    float yaw = 0.0f;        // 弧度，右正（与 simulator GIMBAL yaw 同向）
    float pitch = 0.0f;      // 弧度，上正（与 simulator GIMBAL pitch 同向）
    float yaw_rate = 0.0f;   // rad/s，目标角速度估计
    float pitch_rate = 0.0f;
    bool rate_valid = false;  // 角速度样本是否足够（否则速率类指令应视为无效）
    float yaw_pred = 0.0f;   // 提前量预测角（predict_horizon_ms > 0 时有效）
    float pitch_pred = 0.0f;
    float distance_m = 0.0f; // 0 表示未测距
    int64_t t_ms = 0;
};

}  // namespace dart_vision

#endif  // DART_VISION_TYPES_HPP
