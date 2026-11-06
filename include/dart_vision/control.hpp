#ifndef DART_VISION_CONTROL_HPP
#define DART_VISION_CONTROL_HPP

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

#include <opencv2/core.hpp>

#include "dart_vision/config.hpp"

namespace dart_vision {

// 仿真器通过同一条指令连接回推的飞镖状态（STATE 报文）。
// 控制律必须知道镖的速度/姿态，才能把视觉测到的机体系视线角转成惯性系、
// 并算出正确地机体系舵面指令。
struct DartState {
    bool valid = false;
    int64_t t_ms = 0;
    cv::Vec3f position{0.0f, 0.0f, 0.0f};
    cv::Vec3f velocity{0.0f, 0.0f, 0.0f};
    cv::Vec4f rotation{1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    bool launched = false;
    bool hit = false;
    // 仿真器回推的"满舵可用侧向加速度"（当前速度下，m/s²）。
    // 有了它，视觉端不必再手工同步 [control] availability_mps2：
    // 改仿真器 [aero] 舵效后数值会自动跟着变。0 = 仿真器没给，退回配置值。
    double available_accel = 0.0;
    /// 仿真器的墙钟时间戳（微秒），与图像帧头里的时间戳同一个钟。
    /// 用来把"这一帧图像"和"那一刻的镖姿态"精确配对。0 = 仿真器没给。
    uint64_t wall_us = 0;

    bool airborne() const { return launched && !hit; }
};

// 网络线程写、视觉线程读。内部保留一小段历史，供"按图像时刻取姿态"用。
class DartStateHolder {
public:
    void update(const DartState& state);
    DartState get() const;
    /// 取与给定墙钟时刻最接近的姿态（优先不晚于该时刻的那一个）。
    /// target_us == 0 或没有历史时退回最新值。
    DartState getAtWallClock(uint64_t target_us) const;

private:
    mutable std::mutex mutex_;
    DartState state_;
    std::deque<DartState> history_;
};

bool parseStateLine(const std::string& line, DartState* state);

struct ControlOutput {
    bool active = false;
    double pitch = 0.0;
    double yaw = 0.0;
    double roll = 0.0;
    double los_rate_dps = 0.0;
    double range_m = 0.0;
    double accel_cmd = 0.0;
    double available_accel = 0.0;
    double aim_error_deg = 0.0;
    /// 单目测距是否可信（导向灯够大）。false 时不做灯→靶心换算与瞄点误差反馈。
    bool range_valid = false;
    /// 交战门限打开后的指令斜坡系数（0→1，1 = 已完全放开）
    double engage_ramp = 1.0;
    /// 当前滚转角（度，0 = 机背朝上）——诊断用
    double roll_deg = 0.0;
    /// 视觉算出的**世界系**视线单位向量——诊断用。
    /// 拿它和"镖位置+姿态+靶位置"算出的真值对比，就能判定坐标变换
    /// （机体系→世界系、相机安装）有没有整体偏。
    cv::Vec3f los_world{0.0f, 0.0f, -1.0f};
};

// 控制律（按"仿真只管物理、控制归视觉"的原则，从仿真器搬到视觉端）：
//   a_cmd = N · Vc · λ̇  +  瞄点误差反馈  +  常值偏置
//   舵角  = clamp(a_cmd / a_可用, ±1) × max_fin_deg
// 其中 λ̇ 用视觉视线角 + 仿真回推的镖姿态/速度在**世界系**里算，
// 再旋转到机体系输出——这正是仿真器原先替你做的那些变换。
class DartController {
public:
    explicit DartController(const ControlConfig& cfg);

    // yaw/pitch: 视觉测得的视线角（机体系，弧度）
    // blob_size_px: 导向灯表观尺寸（用于单目测距，<=0 表示不可用）
    ControlOutput update(double yaw, double pitch, double blob_size_px,
                         const DartState& state, int64_t t_ms);

    void reset();

private:
    struct Sample {
        double t_s = 0.0;
        cv::Vec3f los{0.0f, 0.0f, 0.0f};
    };

    ControlConfig cfg_;
    std::deque<Sample> history_;
    cv::Vec3f los_rate_{0.0f, 0.0f, 0.0f};
    double distance_m_ = 0.0;
    /// 最近一次测距是否来自足够大的导向灯
    bool range_valid_ = false;
    /// 本次交战门限打开的仿真时刻（ms）；-1 = 尚未进入交战区
    int64_t gate_open_t_ms_ = -1;
    /// 本发离架的时刻（STATE 的 t_ms）；-1 = 还没离架
    int64_t launch_t_ms_ = -1;
    /// 上次因"STATE 没带回推可用加速度"而告警的时刻（ms），用于限频
    int64_t last_fallback_warn_t_ms_ = -100000;
    int64_t last_sample_t_ms_ = -1;
};

}  // namespace dart_vision

#endif  // DART_VISION_CONTROL_HPP
