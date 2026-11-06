#include "dart_vision/solver.hpp"

#include <algorithm>
#include <cmath>

namespace dart_vision {

AngleSolver::AngleSolver(const SolverConfig& cfg, const CameraConfig& camera)
    : cfg_(cfg), camera_(camera) {
    axis_ = axisFromString(cfg_.axis);
    if (cfg_.rate_window < 2) cfg_.rate_window = 2;
}

void AngleSolver::reset() {
    history_.clear();
}

AimSolution AngleSolver::solve(const Blob& blob, int64_t t_ms) {
    AimSolution out;
    if (!blob.valid()) return out;

    out.valid = true;
    out.axis = axis_;
    out.t_ms = t_ms;
    // 用连通域质心（亚像素）而不是 x + w/2：后者是整型包围盒中心，
    // 量化到 0.5px ≈ 0.155°，而视线角是制导链的输入，这份量化会
    // 直接变成 λ̇ 的阶梯噪声。
    out.u = blob.cx > 0.0f ? blob.cx : blob.centerX();
    out.v = blob.cy > 0.0f ? blob.cy : blob.centerY();

    const double fx = camera_.fx > 0 ? camera_.fx : 1.0;
    const double fy = camera_.fy > 0 ? camera_.fy : 1.0;
    out.yaw = static_cast<float>(std::atan2(out.u - camera_.cx, fx));
    out.pitch = static_cast<float>(std::atan2(camera_.cy - out.v, fy));

    // 单目测距：导向灯实际直径已知，用成像大小反推距离（远距离下量化误差较大）
    if (cfg_.light_diameter_m > 0.0) {
        const double apparent = blob.apparentSize();
        if (apparent >= 0.5) {
            const double focal = (fx + fy) * 0.5;
            out.distance_m =
                static_cast<float>(focal * cfg_.light_diameter_m / std::max(1.0, apparent));
        }
    }

    const double t_s = static_cast<double>(t_ms) / 1000.0;
    // 目标丢失一段时间后重新捕获时，旧样本会让最小二乘拟合出一个巨大的假角速度
    // （实测每次发射开头都会被它带成满舵），所以跨越大间隔就丢弃历史重新积累。
    if (!history_.empty() && cfg_.rate_reset_gap_ms > 0.0 &&
        (t_s - history_.back().t_s) * 1000.0 > cfg_.rate_reset_gap_ms) {
        history_.clear();
    }
    history_.push_back(Sample{t_s, out.yaw, out.pitch});
    while (static_cast<int>(history_.size()) > cfg_.rate_window) history_.pop_front();

    // 角速度：对最近若干帧做最小二乘拟合
    if (static_cast<int>(history_.size()) >= std::max(2, cfg_.rate_min_samples)) {
        const double n = static_cast<double>(history_.size());
        double st = 0, sy = 0, sp = 0, stt = 0, sty = 0, stp = 0;
        const double t0 = history_.front().t_s;
        for (std::size_t i = 0; i < history_.size(); ++i) {
            const double t = history_[i].t_s - t0;
            st += t;
            sy += history_[i].yaw;
            sp += history_[i].pitch;
            stt += t * t;
            sty += t * history_[i].yaw;
            stp += t * history_[i].pitch;
        }
        const double denom = n * stt - st * st;
        if (std::fabs(denom) > 1e-9) {
            out.yaw_rate = static_cast<float>((n * sty - st * sy) / denom);
            out.pitch_rate = static_cast<float>((n * stp - st * sp) / denom);
            out.rate_valid = true;
        }
    }

    // 瞬态（例如发射瞬间相机对准速度矢量）会让拟合值飙到几百 °/s，
    // 直接限幅，避免把舵一把打满。
    if (cfg_.rate_limit_dps > 0.0) {
        const float limit = static_cast<float>(cfg_.rate_limit_dps * kDegToRad);
        out.yaw_rate = std::max(-limit, std::min(limit, out.yaw_rate));
        out.pitch_rate = std::max(-limit, std::min(limit, out.pitch_rate));
    }

    out.yaw_pred = out.yaw;
    out.pitch_pred = out.pitch;
    if (cfg_.predict_horizon_ms > 0.0) {
        const double horizon = cfg_.predict_horizon_ms / 1000.0;
        out.yaw_pred = static_cast<float>(out.yaw + out.yaw_rate * horizon);
        out.pitch_pred = static_cast<float>(out.pitch + out.pitch_rate * horizon);
    }
    return out;
}

}  // namespace dart_vision
