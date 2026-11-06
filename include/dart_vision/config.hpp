#ifndef DART_VISION_CONFIG_HPP
#define DART_VISION_CONFIG_HPP

#include <array>
#include <string>
#include <vector>

namespace dart_vision {

struct SourceConfig {
    std::string mode = "tcp";  // tcp | video
    std::string host = "127.0.0.1";
    int port = 7666;
    std::string video_path;
    bool loop = true;
    bool realtime = true;
};

struct CommandConfig {
    bool enabled = true;
    std::string host;          // 空 = 使用顶层 host
    int port = 7667;
    std::string protocol = "szu_uart";
    bool text_verbose = false;  // text_angle 是否附带距离/像素等附加字段
    double max_rate_hz = 60.0;
    int reconnect_ms = 500;
};

struct CameraConfig {
    int width = 640;
    int height = 480;
    double fov_deg = 45.0;
    std::string fov_axis = "vertical";
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;

    void finalize();  // 依据 fov / 分辨率补齐内参
};

struct DetectorConfig {
    std::vector<std::array<int, 6>> thresholds;  // OpenMV Lab 语义
    int min_area_px = 2;
    int max_area_px = 0;
    int blur_kernel = 0;
    int open_kernel = 0;
    int close_kernel = 0;
};

struct TrackerConfig {
    bool armed = true;
    double start_delay_ms = 80.0;
    int found_range_x = 40;
    int found_range_y = 40;
    int lost_reset_frames = 3;
    bool zoom_enable = true;
    double zoom_after_ms = 900.0;
    double zoom_factor = 4.0;
    int zoom_lost_frames = 2;
};

struct SolverConfig {
    std::string axis = "both";
    double light_diameter_m = 0.20;
    int rate_window = 8;
    double predict_horizon_ms = 0.0;
    // 角速度估计的健壮性：重新捕获目标后不要用跨越丢失期的样本拟合
    double rate_reset_gap_ms = 200.0;  // 与上一帧间隔超过该值就清空历史（0 = 关闭）
    int rate_min_samples = 3;          // 少于该样本数时角速度输出 0
    double rate_limit_dps = 90.0;      // 角速度输出限幅（°/s），抑制瞬态假值（0 = 不限）
};

// 制导律：把视线角误差变成期望舵角（闭环调参的主要旋钮）
struct GuidanceConfig {
    bool enabled = false;          // false = 直接回发视线角（保持原有行为）
    std::string mode = "pursuit";  // pursuit: cmd = k·误差；pn: cmd = N·Vc·λ̇/A_max
    double k_heading = 1.0;        // pursuit 模式的比例增益
    double k_rate = 0.0;           // pursuit 模式的阻尼/提前量（秒）
    // pn 模式：与 simulator_system 的 [navigation] 参数一一对应
    double pn_gain = 4.0;              // navigation.constant
    double closing_speed_mps = 18.0;   // 接近速度估计
    double lateral_accel_limit = 34.0; // navigation.max_lateral_accel
    // 相机随速度矢量转，测到的是机体系视线角速度；比例导引要的是惯性系。
    // 两者相差弹道自身的低头率 γ̇ ≈ -g/V，补偿项 = -g/V（只作用于 pitch）。
    bool gravity_turn_comp = true;
    double gravity_mps2 = 9.81;
    // 重新捕获目标后的稳定期：这段时间内不下发速率类指令
    // （对应原开源脚本的 T_START，避开发射瞬间相机对准速度矢量的跳变）
    double arm_delay_ms = 150.0;
    // 发射判定：预先捕获时记录视线角，偏离超过该角度才认为"镖已经在动"，
    // 之前一律发中立舵（否则发射前就会满舵压着，出膛瞬间直接把弹道打歪）
    double arm_on_motion_deg = 3.0;
    double max_cmd_deg = 20.0;     // 指令限幅，应与仿真器 [servo] max_angle_deg 一致
    double deadband_deg = 0.0;     // 死区
    double bias_yaw_deg = 0.0;     // 常值偏置（安装/瞄点补偿）
    double bias_pitch_deg = 0.0;
};

struct RecordConfig {
    bool enabled = false;
    double fps = 30.0;
    int width = 640;
    int height = 480;
};

// 控制律：从仿真器搬到视觉端的那部分（含坐标变换）
struct ControlConfig {
    bool enabled = false;          // true = 由本工程算舵角并通过 DART 报文回发
    double pn_gain = 1.0;          // N：a = N·Vc·λ̇
    // 常值竖直偏置，直接加在世界系 +Y 上，所以**正值 = 向上、负值 = 向下**。
    // （早期注释写成"负值向上"，与 control.cpp 的实现相反，已更正。）
    double accel_bias_mps2 = 0.0;
    double aim_error_gain = 1.0;   // 瞄点误差反馈增益（0 = 关闭）
    // 零控脱靶量（ZEM）反馈增益：这是**唯一**真正压静态偏差的位置反馈项。
    //
    // 为什么必须单独有它：原来的 aim_error 项算的是 (瞄点方向 − 绿灯方向)，
    // 那是"灯→靶心"的**固定几何偏角**（终端只有 0.2~0.3°），里面没有任何
    // 位置信息。于是制导只剩 PN 的 λ̇ 项，而 N·Vc·λ̇ 对"静止靶 + 静态偏差"
    // 只是一个很软的比例项：实测 N=0.4 时横向修正量只有需求的 13%
    // （靶横移 0.749 m、镖只跟 0.098 m），这就是"yaw 完全跟不上"。
    //
    // ZEM 项的物理含义：按当前速度继续飞，会从瞄点旁边错过多少米。
    //   a = zem_gain · 2·Vc²/R · sin(速度方向 与 到瞄点视线 的夹角)
    // 系数 2·Vc²/R 是"在剩余时间 R/Vc 内把这段偏差压成 0"所需的加速度，
    // 所以 zem_gain = 1 就是最小时间解。它按脱靶量缩放：偏差大时给满舵，
    // 偏差收敛后自动退回，因此不需要像 pn_gain 那样在"跟得上"和"不发散"
    // 之间折中（实测 pn_gain 从 0.4 提到 1.6 才能把横向跟踪从 0.13 提到 0.64，
    // 但同一增益把竖向从 +0.07 m 打到 −0.53 m —— 因为竖向的初始偏差有 3 m，
    // 纯 λ̇ 律在这么短的剩余时间里必然过冲）。
    double zem_gain = 1.0;
    // 重力加速度（ZEM 的零控弹道预测要用它）。必须与仿真器 [aero] gravity_scale
    // 对应的真实重力一致，否则又会出现"多补/少补一份下坠"。
    double gravity_mps2 = 9.81;
    // 机头前伸量（米）：机体系 -Z 方向、从镖的质心到机头尖的距离。
    //
    // 为什么必须单独有它：制导把（位置/速度代表的）**质心**导向瞄点，而比赛判定看的是
    // **镖头**。俯冲时机头尖在质心前方约 0.117 m，于是质心命中靶面时机头还在靶面前方
    // —— 实测（机头尖碰撞球判据 + 关闭本项）机头尖过靶 Δz=-0.153 m、Δy=+0.106 m，
    // 即"命中"其实是镖身触靶、机头悬在靶面前上方那段空隙里。
    // >0 时把瞄点整体回退一个机头前伸矢量（用 STATE 回来的姿态算世界方向），
    // 这样落到靶上的是机头而不是质心。
    double nose_lead_m = 0.0;
    double availability_mps2 = 6.7; // 参考速度下的可用侧向加速度（18 m/s 时约 6.7）
    double reference_speed_mps = 18.0;
    double max_fin_deg = 20.0;     // 舵面最大偏角（与仿真器 [servo] max_angle_deg 一致）
    double engage_range_m = 8.0;   // >0：距离大于该值时舵面中立（纯弹道），进入后才制导
    double min_range_m = 1.5;      // 单目测距下限保护
    // ---- 视觉量 -> 世界系变换所需 ----
    double light_diameter_m = 0.20;      // 导向灯直径（单目测距用）
    double focal_px = 184.75;            // 相机焦距（像素），默认 640x480/120° 水平 FOV
    double aim_offset_up_m = 0.14;       // 靶面中心比导向灯高多少（米）
    double aim_offset_forward_m = 0.10;  // 靶面中心比导向灯远多少（米）
    double rate_filter_secs = 0.12;      // λ̇ 低通时间常数
    double max_sample_jump_deg = 3.0;    // 相邻样本视线角跳变阈值（拦检测跳变）
    // 单目测距可信所需的最小导向灯像素尺寸。低于它就认为"距离不可信"，
    // 此时**不做**灯→靶心的几何换算与瞄点误差反馈（这两项都以距离为分母，
    // 2~3 px 时 ±25% 的距离抖动会直接变成几十厘米的落点散布），
    // 但仍按上一次的距离估计做交战门限判定，避免门限抖动。
    double range_min_blob_px = 3.0;
    // 交战门限打开后的指令斜坡时长（秒，0 = 关闭 = 直接阶跃）。
    // 门限一开，λ̇ 立刻从"不控制"变成一个大值，指令是阶跃上去的；
    // 实测这一下低头冲击是落点偏低的直接来源（N=1.0 时门限后 0.3s 俯仰舵
    // 达到 -0.30，把落点压到靶面下方 0.23m）。用斜坡把它摊开，
    // 可以在**不牺牲末端权限**的前提下消掉冲击——比单纯降 pn_gain 更划算，
    // 因为 pn_gain 会把整个终端修正一起缩小。
    double engage_ramp_secs = 0.0;
    // 用"发射后经过的秒数"代替单目测距做交战门限（>0 时生效，0 = 关闭，
    // 仍用 engage_range_m）。
    //
    // 为什么需要它：单目测距来自 2~6px 的导向灯，逐发的表观尺寸有 ±25% 的差异，
    // 于是"距离降到 engage_range_m"这件事每发发生在不同的真实距离上
    // （实测真实距离 9.3~12.3m，开启时刻差 0.16s）。
    // 而制导指令（尤其是常值偏置）是按**交战时长**积分的：时长差 0.16s
    // 会让落点在轮次间漂移 0.17m——和靶面 0.066m 的容差同一个量级，
    // 表现就是"有时候全中、有时候一个都不中"。
    //
    // 弹道本身在时间上高度可重复（实测 15 发在靶面前 8m 处的垂直位置只差 1mm），
    // 所以按时间开门能让每发的介入点几乎完全一致。
    double engage_after_launch_secs = 0.0;
    /// 滚转稳定增益（0 = 关闭，也就是"从不下发 roll"的旧行为）。
    ///
    /// 解耦时仿真器把内部滚转稳定律一起删了，而视觉端一直写着 roll = 0，
    /// 于是镖在末端自由滚转（实测 ±22°~35°、跨度 57°），既不受控也不可观测。
    /// 滚转一漂，机体系的 pitch/yaw 相对世界竖直就跟着转：
    /// 竖直修正的效率随滚转角变化、横向与竖直互相串——这就是
    /// "yaw 跟不上 + 竖直旋钮调不动 + 同配置两轮差 4~20cm" 的直接原因。
    ///
    /// 用 STATE 的四元数算滚转误差，误差项 × 本增益 直接作为舵角指令下发。
    /// 符号不对（越稳越歪）时把它改成负值即可。
    double roll_gain = 0.0;
    /// 静开环标定：直接下发一个固定的 pitch / yaw 舵角（度），
    /// 绕过制导律、交战门限和斜坡。非零时生效，用来测量
    /// "固定偏舵 → 落点偏移"的**符号和增益** —— 这是唯一能把
    /// 竖直/横向通道标定清楚的办法（靠间接推断已经自相矛盾过三次）。
    /// 两者都为 0 = 关闭，行为与原来完全一致。
    double pitch_open_loop_deg = 0.0;
    double yaw_open_loop_deg = 0.0;
    /// 滚转静开环标定：直接下发固定 roll 舵角（度）。需要仿真器
    /// [aero] fin_torque_scale > 0 才有物理权限。
    double roll_open_loop_deg = 0.0;
    /// 相机是否固连在镖头（对应仿真器 [camera] align_to_velocity = false）。
    /// true  = 固连：机体系视线角 × 姿态 → 世界系（真机的形态）
    /// false = 随速度矢量转：用速度构造相机系（旧行为，仅用于 A/B 对照）
    bool camera_body_fixed = true;
};

struct AppConfig {
    SourceConfig source;
    CommandConfig command;
    CameraConfig camera;
    DetectorConfig detector;
    TrackerConfig tracker;
    SolverConfig solver;
    ControlConfig control;
    GuidanceConfig guidance;
    RecordConfig record;

    std::string config_path;
    bool show = true;
    bool send = true;
    int log_every = 30;
    int64_t exit_after_ms = 0;  // 0 = 不自动退出

    // 读取 YAML；文件缺失或字段缺失时使用结构体默认值并回报 warning
    static AppConfig load(const std::string& path, std::string* warning);
    std::string describe() const;
};

}  // namespace dart_vision

#endif  // DART_VISION_CONFIG_HPP
