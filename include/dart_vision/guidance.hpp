#ifndef DART_VISION_GUIDANCE_HPP
#define DART_VISION_GUIDANCE_HPP

#include "dart_vision/config.hpp"
#include "dart_vision/types.hpp"

namespace dart_vision {

// 制导律：视线角误差 -> 期望舵角。
//
//   cmd = k_heading * err + k_rate * err_rate + bias
//   再按 deadband 去抖、按 max_cmd 限幅
//
// 两个可调增益的物理含义：
//   k_heading : 比例增益，把"目标偏了多少度"变成"舵打多少度"
//   k_rate    : 阻尼/提前量（单位秒）。k_rate = k_heading * T_lead 时，
//               等效于按 T_lead 秒后的预测位置瞄准；纯阻尼取小正值即可
//
// enabled = false 时完全不介入，回发的内容与检测结果一致（等价于此前的行为）。
class GuidanceLaw {
public:
    explicit GuidanceLaw(const GuidanceConfig& cfg);

    // 输入视线角解算结果，输出期望舵角（弧度）
    void apply(const AimSolution& solution, double* cmd_yaw, double* cmd_pitch) const;

    const GuidanceConfig& config() const { return cfg_; }

private:
    GuidanceConfig cfg_;
    double max_cmd_rad_ = 0.0;
    double deadband_rad_ = 0.0;
    double bias_yaw_rad_ = 0.0;
    double bias_pitch_rad_ = 0.0;
};

}  // namespace dart_vision

#endif  // DART_VISION_GUIDANCE_HPP
