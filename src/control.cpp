#include "dart_vision/control.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace dart_vision {

namespace {

constexpr double kRadToDeg = 57.29577951308232;
constexpr double kDegToRad = 1.0 / kRadToDeg;

template <typename T>
T clampValue(T value, T low, T high) {
    return value < low ? low : (value > high ? high : value);
}

cv::Vec3f normalize(const cv::Vec3f& v) {
    const float n = std::sqrt(v.dot(v));
    if (n < 1.0e-6f) return cv::Vec3f(0.0f, 0.0f, 0.0f);
    return v * (1.0f / n);
}

cv::Vec3f rotate(const cv::Vec4f& q, const cv::Vec3f& v) {
    // 四元数 (w,x,y,z) 旋转向量：v' = v + 2w(q×v) + 2q×(q×v)
    const cv::Vec3f u(q[1], q[2], q[3]);
    const cv::Vec3f t = u.cross(v) * 2.0f;
    return v + t * q[0] + u.cross(t);
}

cv::Vec3f inverseRotate(const cv::Vec4f& q, const cv::Vec3f& v) {
    return rotate(cv::Vec4f(q[0], -q[1], -q[2], -q[3]), v);
}

// 相机机身系：相机沿 -Z 前视、上轴对齐世界竖直（与仿真器 align_camera_to_flight 一致）
// 目标在画面里的 (yaw, pitch) 就在这里定义：yaw 右正、pitch 上正。
cv::Vec3f cameraDirectionInWorld(const cv::Vec3f& velocity, double yaw, double pitch) {
    const cv::Vec3f forward = normalize(velocity);
    if (forward.dot(forward) < 0.5f) return cv::Vec3f(0.0f, 0.0f, -1.0f);
    cv::Vec3f world_up(0.0f, 1.0f, 0.0f);
    if (std::fabs(forward.dot(world_up)) > 0.999f) world_up = cv::Vec3f(0.0f, 0.0f, 1.0f);
    cv::Vec3f right = normalize(forward.cross(world_up));
    const cv::Vec3f up = normalize(right.cross(forward));
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    return normalize(right * static_cast<float>(sy * cp) + up * static_cast<float>(sp) +
                     forward * static_cast<float>(cy * cp));
}

/// 机体系视线方向（相机固连在镖头时用这个）。
///
/// 机体系约定与仿真一致：x 向右、y 向上、机头朝 -z。
/// 相机固连就意味着"相机系 = 机体系"（只差一个平移），所以可直接按
/// yaw/pitch 构造机体系方向，再用 STATE 的四元数转到世界系。
///
/// 注意：这与上面的 `cameraDirectionInWorld`（用速度矢量构造相机系、
/// 对应 `align_to_velocity = true` 的虚拟云台）是**两种不同的相机**，
/// 用错哪一个，制导拿到的世界系方向就整体偏一个弹道倾角。
cv::Vec3f cameraDirectionInBody(double yaw, double pitch) {
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    return normalize(cv::Vec3f(static_cast<float>(sy * cp), static_cast<float>(sp),
                               static_cast<float>(-cy * cp)));
}

}  // namespace

void DartStateHolder::update(const DartState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
    // 只保留最近 ~2s（50Hz × 100），足够覆盖图像传输+解码的延迟
    history_.push_back(state);
    while (history_.size() > 100) history_.pop_front();
}

DartState DartStateHolder::getAtWallClock(uint64_t target_us) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (target_us == 0 || history_.empty()) return state_;
    const DartState* best = nullptr;
    for (const DartState& candidate : history_) {
        if (candidate.wall_us == 0) continue;      // 旧版仿真器没给时间戳
        // 优先取"不晚于目标时刻"的最近样本；没有则取最早的
        if (candidate.wall_us <= target_us &&
            (best == nullptr || candidate.wall_us > best->wall_us)) {
            best = &candidate;
        }
    }
    return best != nullptr ? *best : history_.front();
}

DartState DartStateHolder::get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

bool parseStateLine(const std::string& line, DartState* state) {
    if (!state) return false;
    if (line.rfind("STATE", 0) != 0) return false;

    std::istringstream iss(line);
    std::string tag;
    DartState parsed;
    int launched = 0;
    int hit = 0;
    double t_ms = 0.0;
    if (!(iss >> tag >> t_ms >> parsed.position[0] >> parsed.position[1] >> parsed.position[2] >>
          parsed.velocity[0] >> parsed.velocity[1] >> parsed.velocity[2] >> parsed.rotation[0] >>
          parsed.rotation[1] >> parsed.rotation[2] >> parsed.rotation[3] >> launched >> hit)) {
        return false;
    }
    parsed.t_ms = static_cast<int64_t>(t_ms);
    parsed.launched = launched != 0;
    parsed.hit = hit != 0;
    // 可选字段（旧版仿真器不发，忽略即可）：
    //   第 14 个 = 满舵可用侧向加速度；第 15 个 = 墙钟时间戳（微秒）
    std::string avail_token;
    if (iss >> avail_token) {
        char* end = nullptr;
        const double avail = std::strtod(avail_token.c_str(), &end);
        if (end != avail_token.c_str() && std::isfinite(avail) && avail >= 0.0) {
            parsed.available_accel = avail;
        }
    }
    std::string wall_token;
    if (iss >> wall_token) {
        char* end = nullptr;
        const unsigned long long wall = std::strtoull(wall_token.c_str(), &end, 10);
        if (end != wall_token.c_str()) {
            parsed.wall_us = static_cast<uint64_t>(wall);
        }
    }
    parsed.valid = true;
    *state = parsed;
    return true;
}

DartController::DartController(const ControlConfig& cfg) : cfg_(cfg) {}

void DartController::reset() {
    history_.clear();
    los_rate_ = cv::Vec3f(0.0f, 0.0f, 0.0f);
    distance_m_ = 0.0;
    range_valid_ = false;
    last_sample_t_ms_ = -1;
}

ControlOutput DartController::update(double yaw, double pitch, double blob_size_px,
                                     const DartState& state, int64_t t_ms) {
    ControlOutput out;
    if (!cfg_.enabled || !state.valid || !state.airborne()) {
        launch_t_ms_ = -1;
        reset();
        return out;
    }
    // 记下本发离架时刻：用 STATE 的仿真时间，避免和视觉的帧时间混用
    if (launch_t_ms_ < 0) {
        launch_t_ms_ = state.t_ms;
    }

    // ---- 静开环标定：直接下发固定舵角，绕过制导律/门限/斜坡 ----
    // 用来测"固定偏舵 → 落点偏移"的符号与增益。为 0 时完全不影响原行为。
    if (cfg_.pitch_open_loop_deg != 0.0 || cfg_.yaw_open_loop_deg != 0.0 ||
        cfg_.roll_open_loop_deg != 0.0) {
        const double max_fin = std::max(0.0, cfg_.max_fin_deg) * kDegToRad;
        out.pitch = clampValue(cfg_.pitch_open_loop_deg * kDegToRad, -max_fin, max_fin);
        out.yaw = clampValue(cfg_.yaw_open_loop_deg * kDegToRad, -max_fin, max_fin);
        out.roll = clampValue(cfg_.roll_open_loop_deg * kDegToRad, -max_fin, max_fin);
        out.active = true;
        out.range_m = distance_m_;
        out.engage_ramp = 1.0;
        return out;
    }

    const cv::Vec3f velocity = state.velocity;
    const double speed = std::sqrt(velocity.dot(velocity));
    if (speed < 1.0) {  // 还没出膛
        reset();
        return out;
    }

    // 1) 视觉测得的机体系视线角 -> 世界系视线方向（这一步原先在仿真器里）
    // 相机固连 → 用姿态把机体系视线角转到世界系；
    // 相机随速度矢量转（align_to_velocity）→ 用速度构造相机系（旧行为）。
    // 两者必须与仿真器 [camera] align_to_velocity 保持一致，否则整体偏一个弹道倾角。
    const cv::Vec3f los = cfg_.camera_body_fixed
                              ? rotate(state.rotation, cameraDirectionInBody(yaw, pitch))
                              : cameraDirectionInWorld(velocity, yaw, pitch);
    if (los.dot(los) < 0.5f) return out;
    out.los_world = los;

    // 2) 单目测距（导向灯直径已知），用于"灯 -> 靶心"的几何换算
    //    距离只在导向灯足够大时才可信：2 px 反推 18.5 m、3 px 反推 12.3 m，
    //    量化误差 ±25%。而"灯→靶心"换算和瞄点误差反馈都以距离为分母，
    //    直接吃这份抖动会变成几十厘米的落点散布。
    //    distance_m_ 始终更新（交战门限要用连续值，避免门限抖动），
    //    range_valid_ 只表示"这次的距离够不够准"。
    if (cfg_.light_diameter_m > 0.0 && blob_size_px > 0.5) {
        // fx 由相机内参给出（本工程解算器同源），这里用 fx·D/像素尺寸
        distance_m_ = cfg_.light_diameter_m * cfg_.focal_px / blob_size_px;
        range_valid_ = blob_size_px >= std::max(1.0, cfg_.range_min_blob_px);
    } else {
        range_valid_ = false;
    }

    // 3) 瞄点 = 灯位置 + 装甲板固定偏移（+Y 向上、+Z 更远）。
    //    同时保留未归一化的 aim_vec（镖 -> 瞄点，长度≈距离）——ZEM 项要用它的**长度**，
    //    只看方向的旧写法会丢掉"还有多远、还剩多少时间"这条信息。
    cv::Vec3f aim_vec = los * static_cast<float>(distance_m_);
    double aim_error_deg = 0.0;
    if (range_valid_ && distance_m_ > cfg_.min_range_m &&
        (std::fabs(cfg_.aim_offset_up_m) > 1.0e-6 || std::fabs(cfg_.aim_offset_forward_m) > 1.0e-6)) {
        const cv::Vec3f offset(0.0f, static_cast<float>(cfg_.aim_offset_up_m),
                               static_cast<float>(cfg_.aim_offset_forward_m));
        const cv::Vec3f corrected = normalize(los * static_cast<float>(distance_m_) + offset);
        if (corrected.dot(corrected) > 0.5f) {
            aim_vec = los * static_cast<float>(distance_m_) + offset;
            aim_error_deg =
                std::acos(clampValue(static_cast<double>(los.dot(corrected)), -1.0, 1.0)) *
                kRadToDeg;
        }
    }
    cv::Vec3f aim_dir = normalize(aim_vec);
    if (aim_dir.dot(aim_dir) < 0.5f) aim_dir = los;

    // 3b) 瞄点回退一个"机头前伸"矢量：制导把**质心**导向瞄点，判定看的是**镖头**。
    //     不回退的话，质心命中靶面时机头还在靶面前上方（实测 Δz=-0.153、Δy=+0.106 m），
    //     画面里就是"镖身擦靶、机头悬在靶面和绿灯之间的空隙里"。
    if (cfg_.nose_lead_m > 1.0e-6) {
        const cv::Vec3f forward = rotate(state.rotation, cv::Vec3f(0.0f, 0.0f, -1.0f));
        if (forward.dot(forward) > 0.5f) {
            aim_vec -= forward * static_cast<float>(cfg_.nose_lead_m);
            aim_dir = normalize(aim_vec);
            if (aim_dir.dot(aim_dir) < 0.5f) aim_dir = los;
        }
    }

    // 4) 惯性系视线角速度：只在收到新样本时差分（视觉 30Hz、视觉线程 ~60Hz）
    cv::Vec3f rate = los_rate_;
    bool new_sample = t_ms > last_sample_t_ms_;
    if (new_sample) {
        if (last_sample_t_ms_ >= 0) {
            const double dt = static_cast<double>(t_ms - last_sample_t_ms_) / 1000.0;
            if (dt > 1.0e-3 && dt < 0.5) {
                const cv::Vec3f raw = (los - history_.back().los) * static_cast<float>(1.0 / dt);
                const float jump_deg =
                    std::acos(clampValue(
                        static_cast<double>(history_.back().los.dot(los)), -1.0, 1.0)) *
                    static_cast<float>(kRadToDeg);
                if (jump_deg <= cfg_.max_sample_jump_deg) {
                    const double tau = std::max(1.0e-3, cfg_.rate_filter_secs);
                    const double alpha = clampValue(dt / (dt + tau), 0.0, 1.0);
                    los_rate_ = los_rate_ + (raw - los_rate_) * static_cast<float>(alpha);
                    rate = los_rate_;
                }
            }
        }
        history_.push_back(Sample{static_cast<double>(t_ms) / 1000.0, los});
        while (history_.size() > 16) history_.pop_front();
        last_sample_t_ms_ = t_ms;
    }

    // 5) 交战门限：远距离不介入（舵面中立，走纯弹道）
    //    优先用"发射后计时"（每发介入点几乎一致），否则退回单目测距门限。
    const bool gate_closed =
        cfg_.engage_after_launch_secs > 1.0e-6
            ? (state.t_ms - launch_t_ms_) <
                  static_cast<int64_t>(cfg_.engage_after_launch_secs * 1000.0)
            : (cfg_.engage_range_m > 0.0 && distance_m_ > cfg_.min_range_m &&
               distance_m_ > cfg_.engage_range_m);
    if (gate_closed) {
        gate_open_t_ms_ = -1;  // 出了交战区就重新计时（下一发/再次接近重走斜坡）
        out.active = true;
        out.range_m = distance_m_;
        return out;
    }
    // 门限刚打开的那一刻记时，之后的指令按斜坡从 0 拉到 1，避免阶跃冲击
    if (gate_open_t_ms_ < 0) {
        gate_open_t_ms_ = t_ms;
    }
    const double ramp =
        cfg_.engage_ramp_secs > 1.0e-6
            ? clampValue(static_cast<double>(t_ms - gate_open_t_ms_) /
                             (cfg_.engage_ramp_secs * 1000.0),
                         0.0, 1.0)
            : 1.0;

    // 6) 指令加速度（世界系）
    const double closing_speed = std::max(0.0, speed);  // 靶视为静止
    // 指令满量程（分母）：优先用仿真器回推的**真实**满舵可用侧向加速度，
    // 这样改仿真器 [aero] 舵效后不需要手工同步 [control] availability_mps2
    // （这一步没同步过就是"改了舵效但指令变弱/常饱和"的经典来源）。
    // 仿真器没回推时（旧版/真机）退回 availability_mps2 的 v² 标定。
    const double speed_ratio = speed / std::max(1.0, cfg_.reference_speed_mps);
    const double reported_scale = state.available_accel;
    const double accel_scale =
        std::max(1.0, reported_scale > 0.0
                          ? reported_scale
                          : cfg_.availability_mps2 * speed_ratio * speed_ratio);
    // ⚠ 兜底标定一旦被用上，同样的指令加速度会算出**偏大的舵角**：14 m/s 时
    // 仿真器回推值是 ≈8.0 m/s²，而兜底值只有 ≈4.0 m/s² —— 恰好 2 倍。
    // 这会让俯仰过冲、末端发散（实测症状：落点双峰、俯仰指令是正常值的 2.3 倍）。
    // 每发只吼一次，避免刷屏。
    if (reported_scale <= 0.0 && cfg_.availability_mps2 > 0.0) {
        if (t_ms - last_fallback_warn_t_ms_ > 1500) {
            last_fallback_warn_t_ms_ = t_ms;
            std::fprintf(stderr,
                         "[警告] STATE 没带回推的可用侧向加速度（avail），正在用兜底标定 "
                         "availability_mps2=%.2f×(v/%.0f)²=%.2f m/s² —— 舵角会偏大 ~2 倍。"
                         "请确认视觉端与仿真器版本匹配（STATE 第 14 个字段）。\n",
                         cfg_.availability_mps2, cfg_.reference_speed_mps, accel_scale);
        }
    }
    cv::Vec3f accel = rate * static_cast<float>(cfg_.pn_gain * closing_speed);
    accel += cv::Vec3f(0.0f, static_cast<float>(cfg_.accel_bias_mps2), 0.0f);
    // ---- 零控脱靶量（ZEM）位置反馈 ----
    // "零控"= 从现在起舵面不再出力，镖按**自由弹道**飞（位置 p+vT+½gT²），
    // 到达瞄点所在距离时离瞄点还有多少米。把这个偏差在剩余时间 T 内压成 0
    // 需要 a = 2·ZEM/T²（双积分器最小时间解）。
    //
    // ⚠ 必须带上 ½gT² 这一项。这是这一版最容易踩的坑：如果按**直线**外推，
    // 得到的"脱靶量"里混进了重力本来就会提供的下坠，制导再把这份下坠补一遍，
    // 就会把镖压到靶面下方。实测（seed 999、15 发、靶面 0.926×0.066）：
    //   纯弹道机头过靶 高 +0.083 m（弹道角 −37.9°）  ← 其实基本就在靶上
    //   N=0.4 弱制导     高 +0.066 m
    //   直线 ZEM gain=1  低 −0.544 m（弹道角 −46.1°）← 多压了 0.6 m
    //   N=1.6 强 PN      低 −0.535 m（同样特征）
    // 带上重力项后，偏差只剩下"靶自己动掉的那部分 + 真正的制导误差"。
    if (cfg_.zem_gain > 1.0e-6 && distance_m_ > cfg_.min_range_m) {
        const double time_to_go = distance_m_ / std::max(1.0, closing_speed);
        if (time_to_go > 1.0e-3) {
            const cv::Vec3f gravity(0.0f, static_cast<float>(-cfg_.gravity_mps2), 0.0f);
            const float t = static_cast<float>(time_to_go);
            // p(T) = p + v·T + ½gT²  =>  miss = 瞄点 − p(T)
            const cv::Vec3f miss = aim_vec - velocity * t - gravity * (0.5f * t * t);
            // ⚠ 只取**垂直于速度**的那个分量。舵面产生的气动力垂直于来流
            // （开环实测：纯 +10° 俯仰舵 → 机体力 (0,+3.52,−0.90) N，没有轴向分量），
            // 而 miss 里天然带着一段沿速度方向的分量：time_to_go 是用到**绿灯**的
            // 斜距算的，而瞄点在灯后面 0.1 m，于是预测点会"冲过"瞄点所在的平面。
            // 实测这一段有 0.8 m，乘 2/T² 后是 ~8 m/s² 的**倒推力**，映到机体系
            // 竖直轴就变成 −20° 的下压舵（这就是"制导把镖压到靶面下方 0.5 m"）。
            // 投影掉之后：seed 999 第 2 发在门限时刻的真实 ZEM 只有 0.01 m，
            // 也就是"本来就不用怎么修" —— 和纯弹道只高 0.083 m 是同一件事。
            const cv::Vec3f vhat(velocity[0] / static_cast<float>(speed),
                                 velocity[1] / static_cast<float>(speed),
                                 velocity[2] / static_cast<float>(speed));
            const cv::Vec3f miss_perp = miss - vhat * miss.dot(vhat);
            const double gain = cfg_.zem_gain * 2.0 / (time_to_go * time_to_go);
            accel += miss_perp * static_cast<float>(gain);
        }
    }
    if (cfg_.aim_error_gain > 0.0 && range_valid_ && distance_m_ > cfg_.min_range_m) {
        const cv::Vec3f error_dir = aim_dir - los;
        accel += error_dir * static_cast<float>(cfg_.aim_error_gain * 2.0 * closing_speed *
                                                closing_speed / distance_m_);
    }

    // 7) 世界系 -> 机体系（原先也在仿真器里），再换算成舵角
    const cv::Vec3f local = inverseRotate(state.rotation, accel);
    const double max_fin = std::max(0.0, cfg_.max_fin_deg) * kDegToRad;
    out.pitch =
        clampValue(static_cast<double>(local[1]) / accel_scale, -1.0, 1.0) * max_fin * ramp;
    out.yaw =
        clampValue(static_cast<double>(local[0]) / accel_scale, -1.0, 1.0) * max_fin * ramp;
    out.roll = 0.0;
    out.active = true;
    out.los_rate_dps = std::sqrt(rate.dot(rate)) * kRadToDeg;
    out.range_m = distance_m_;
    out.accel_cmd = std::sqrt(accel.dot(accel));
    out.available_accel = accel_scale;
    out.aim_error_deg = aim_error_deg;
    out.range_valid = range_valid_;
    out.engage_ramp = ramp;

    // 8) 滚转稳定（仿真器侧内部律已随解耦删除，这里补回来）
    //    滚转误差 = (机体上轴 × 世界竖直) · 机体前向 = -sin(bank)。
    //    指令取 -误差×增益，把机背拉回朝上。与 pitch/yaw 同一个限幅与斜坡。
    //    注：Bevy 的机头方向是 -Z。
    const cv::Vec3f body_up = rotate(state.rotation, cv::Vec3f(0.0f, 1.0f, 0.0f));
    const cv::Vec3f body_forward =
        rotate(state.rotation, cv::Vec3f(0.0f, 0.0f, -1.0f));
    const double roll_error = static_cast<double>(
        body_up.cross(cv::Vec3f(0.0f, 1.0f, 0.0f)).dot(body_forward));
    out.roll = cfg_.roll_gain != 0.0
                   ? clampValue(-roll_error * cfg_.roll_gain, -1.0, 1.0) * max_fin * ramp
                   : 0.0;
    out.roll_deg = static_cast<double>(std::asin(clampValue(
                       static_cast<double>(roll_error), -1.0, 1.0))) *
                   kRadToDeg;
    return out;
}

}  // namespace dart_vision
