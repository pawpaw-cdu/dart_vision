#ifndef DART_VISION_SOLVER_HPP
#define DART_VISION_SOLVER_HPP

#include <cstdint>
#include <deque>

#include "dart_vision/config.hpp"
#include "dart_vision/types.hpp"

namespace dart_vision {

// 像素 -> 角度的解算。
//
// 原开源工程里这一步在主控板上做：视觉只回发 blob 矩形。移植到 dart_vision 后
// 视觉端直接把角度算出来，主控（或仿真桥）拿到就能用。
//
// 相机坐标系约定与 simulator_system 一致：
//   光轴为 0，右为正（yaw），上为正（pitch）
//   yaw   = atan((u - cx) / fx)
//   pitch = atan((cy - v) / fy)
class AngleSolver {
public:
    AngleSolver(const SolverConfig& cfg, const CameraConfig& camera);

    AimSolution solve(const Blob& blob, int64_t t_ms);
    void reset();

private:
    struct Sample {
        double t_s = 0.0;
        double yaw = 0.0;
        double pitch = 0.0;
    };

    SolverConfig cfg_;
    CameraConfig camera_;
    int axis_ = kAxisBoth;
    std::deque<Sample> history_;
};

}  // namespace dart_vision

#endif  // DART_VISION_SOLVER_HPP
