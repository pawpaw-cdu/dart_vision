#include "dart_vision/guidance.hpp"

#include <algorithm>
#include <cmath>

namespace dart_vision {

namespace {

// 死区内视为已对准，避免末端因量化噪声来回打舵
double applyDeadband(double value, double deadband) {
    if (deadband <= 0.0) return value;
    if (std::fabs(value) <= deadband) return 0.0;
    return value > 0.0 ? value - deadband : value + deadband;
}

}  // namespace

GuidanceLaw::GuidanceLaw(const GuidanceConfig& cfg) : cfg_(cfg) {
    max_cmd_rad_ = std::max(0.0, cfg_.max_cmd_deg) * kDegToRad;
    deadband_rad_ = std::max(0.0, cfg_.deadband_deg) * kDegToRad;
    bias_yaw_rad_ = cfg_.bias_yaw_deg * kDegToRad;
    bias_pitch_rad_ = cfg_.bias_pitch_deg * kDegToRad;
}

void GuidanceLaw::apply(const AimSolution& solution, double* cmd_yaw,
                        double* cmd_pitch) const {
    if (!cmd_yaw || !cmd_pitch) return;

    if (!cfg_.enabled) {
        *cmd_yaw = solution.yaw_pred;
        *cmd_pitch = solution.pitch_pred;
        return;
    }

    if (!solution.rate_valid) {
        // 角速度样本还不够（刚重新捕获目标）：不做制导动作，只保留偏置。
        // 否则重力补偿项会拿 0 当视线角速度，算出 -g/V 那种"满舵"的假指令。
        *cmd_yaw = bias_yaw_rad_;
        *cmd_pitch = bias_pitch_rad_;
        return;
    }

    const double err_yaw = applyDeadband(solution.yaw, deadband_rad_);
    const double err_pitch = applyDeadband(solution.pitch, deadband_rad_);
    const double rate_yaw = solution.yaw_rate;
    const double rate_pitch = solution.pitch_rate;

    double yaw = 0.0;
    double pitch = 0.0;

    if (cfg_.mode == "pn") {
        // 与 simulator_system 的比例导引等价（[navigation]）：：
        //   a_cmd = N * Vc * λ̇                       （λ̇ = 视线角速度 rad/s）
        //   fin   = clamp(a_cmd / A_max, ±1) * max_cmd
        // 这里用相机（≈机体系）测到的视线角速度，符号与机体 X/Y 通道一致。
        // 注意：这消的是"视线角速率"，不是"视线角"，所以不会像纯追踪那样
        // 把机头强行压向目标、压平弹道。
        const double accel_limit = std::max(1e-3, cfg_.lateral_accel_limit);
        const double speed = std::max(1.0, cfg_.closing_speed_mps);

        // 机体系 -> 惯性系：相机跟着速度矢量转，所以测到的 pitch 视线角速度里
        // 含有弹道自身的低头率。弹道在重力下大约以 γ̇ ≈ -g/V 低头，把它减掉
        // 才得到比例导引真正要消的惯性系视线角速度。
        double rate_pitch_inertial = rate_pitch;
        if (cfg_.gravity_turn_comp) {
            rate_pitch_inertial -= cfg_.gravity_mps2 / speed;
        }

        const double a_yaw = cfg_.pn_gain * speed * rate_yaw;
        const double a_pitch = cfg_.pn_gain * speed * rate_pitch_inertial;
        yaw = std::max(-1.0, std::min(1.0, a_yaw / accel_limit)) * max_cmd_rad_;
        pitch = std::max(-1.0, std::min(1.0, a_pitch / accel_limit)) * max_cmd_rad_;
        // 允许在 PN 之上再叠加一点"回中"项（默认 k_heading = 0）
        yaw += cfg_.k_heading * err_yaw;
        pitch += cfg_.k_heading * err_pitch;
    } else {
        yaw = cfg_.k_heading * err_yaw + cfg_.k_rate * rate_yaw;
        pitch = cfg_.k_heading * err_pitch + cfg_.k_rate * rate_pitch;
    }

    yaw += bias_yaw_rad_;
    pitch += bias_pitch_rad_;

    yaw = std::max(-max_cmd_rad_, std::min(max_cmd_rad_, yaw));
    pitch = std::max(-max_cmd_rad_, std::min(max_cmd_rad_, pitch));

    *cmd_yaw = yaw;
    *cmd_pitch = pitch;
}

}  // namespace dart_vision
