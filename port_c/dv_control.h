#ifndef DV_CONTROL_H
#define DV_CONTROL_H

/*
 * 制导飞镖 —— 控制层 C 语言移植版（给电控/MCU 用）
 *
 * 来源：/dart_vision/src/control.cpp（C++ 版，与 simulator_system 联调验证过：
 *       15 发/轮、机头尖命中靶面 10~12 发、机头相对靶心 Δy = +0.017 ± 0.025 m）
 *
 * 这一层干什么：把「视觉给的机体系视线角」+「IMU/飞控给的姿态与速度」
 *              换算成 **三轴舵角指令**（pitch/yaw/roll，弧度）。
 * 不干什么：不做检测、不做解算、不做舵机混控与位置环（后面两个在本文件的
 *          dv_mix_fins / dv_servo_step，可以直接拿去用）。
 *
 * 依赖：只用 <math.h>。无动态内存、无文件、无打印（告警走 dv_warn_hook）。
 *
 * 坐标系约定（**必须和视觉/仿真一致，移植时第一件事就是核对这个**）：
 *   机体系：x = 右、y = 上、机头朝 -z（Bevy/仿真约定）
 *   世界系：右手系，y 向上，重力 = (0, -g, 0)
 *   角度：全部用**弧度**（视觉给的 yaw/pitch 也是弧度；yaw 右正、pitch 上正）
 * 如果你们的 IMU 用的是「x 前、y 右、z 下」，先转成上面的约定再调用，
 * 详见 README.md 的「坐标系转换」一节。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 基础类型                                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    float x, y, z;
} dv_vec3;

typedef struct {
    float w, x, y, z; /* 四元数，与视觉/仿真同序（w 在前） */
} dv_quat;

/* 镖的状态：由电控自己（IMU/姿态解算/飞控）填充，单位 m、m/s、四元数无量纲 */
typedef struct {
    uint8_t  valid;    /* 0 = 本帧状态不可用（数据没更新/超时），控制层会直接复位 */
    uint8_t  launched; /* 1 = 已发射 */
    uint8_t  hit;      /* 1 = 已判定命中（控制层会停手） */
    int64_t  t_ms;     /* 单调时间戳（ms），控制层用它做差速/计时门限 */
    dv_vec3  position; /* 世界系位置，m（只在诊断里用，可填 0） */
    dv_vec3  velocity; /* 世界系速度，m/s（**必填**） */
    dv_quat  rotation; /* 机体系 -> 世界系 的姿态四元数（**必填**） */
    /* 满舵可用侧向加速度（m/s²，当前速度下）。真机没有这个回推就填 0，
     * 控制层会退化成用 cfg->availability_mps2 做 v² 标定 —— 但那会让
     * 舵角偏大（实测 14 m/s 时 4.0 vs 8.0，正好 2 倍），所以**强烈建议**
     * 电控按自家的舵效把这个值算出来填进去。 */
    float    available_accel;
} dv_state;

/* ------------------------------------------------------------------ */
/* 配置（默认值 = 当前联调定稿值，见 README）                          */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t enabled; /* 0 = 全关（输出全 0），调试用 */

    /* ---- 制导律 ---- */
    float pn_gain;          /* 比例导引 a = N·Vc·λ̇。当前定稿 0（见 README 3.2） */
    float zem_gain;         /* 零控脱靶量位置反馈增益。当前定稿 0.6（1.0 会翻面） */
    float aim_error_gain;   /* 老的「灯→靶心」几何偏角项。当前 0（无位置信息） */
    float accel_bias_mps2;  /* 世界系 +Y 常值偏置（正值向上），微调落点用 */
    float gravity_mps2;     /* 重力加速度，必须与真实一致（ZEM 要用） */
    float nose_lead_m;      /* 机头前伸量：质心到机头尖距离，制导瞄机头用 */

    /* ---- 相机/测距 ---- */
    uint8_t camera_body_fixed; /* 1 = 相机固连镖头（真机就是这种）；0 = 随速度矢量转 */
    float light_diameter_m;    /* 导向灯直径（单目测距基准），0 = 不测距 */
    float focal_px;            /* 相机焦距（像素） */
    float range_min_blob_px;   /* 灯像素小于它 → 认为测距不可信 */
    float min_range_m;         /* 距离下限保护 */

    /* ---- 瞄点 ---- */
    float aim_offset_up_m;      /* 靶面中心比灯高多少（几何 +0.07，含校靶余量） */
    float aim_offset_forward_m; /* 靶面中心比灯远多少（几何 +0.10） */

    /* ---- 视线角速率滤波 ---- */
    float rate_filter_secs;     /* 一阶低通时间常数 */
    float max_sample_jump_deg;  /* 相邻样本跳变超过它就不更新速率 */

    /* ---- 交战门限 ---- */
    float engage_after_launch_secs; /* >0：发射后多少秒才开始制导（推荐，逐发一致） */
    float engage_range_m;           /* >0 且上面为 0 时：用单目测距做门限 */
    float engage_ramp_secs;         /* 门限打开后指令从 0 斜坡到 1 的时长 */

    /* ---- 指令 -> 舵角 ---- */
    float availability_mps2;   /* 兜底：参考速度下的可用侧向加速度 */
    float reference_speed_mps; /* 兜底标定的参考速度 */
    float max_fin_deg;         /* 舵面最大偏角（与舵机机械限幅一致） */
    float roll_gain;           /* 滚转稳定增益，0 = 不主动稳滚（当前 0） */

    /* ---- 静开环标定（非 0 就绕过制导律，直接发固定舵角；单位度） ---- */
    float pitch_open_loop_deg;
    float yaw_open_loop_deg;
    float roll_open_loop_deg;
} dv_config;

/* 控制层内部状态（放在电控的全局/静态区，不要放栈上） */
typedef struct {
    dv_vec3 los_rate;              /* 世界系视线角速率（滤波后） */
    dv_vec3 last_los;              /* 上一帧世界系视线单位向量 */
    int64_t last_sample_t_ms;      /* 上一次做差分的时刻，-1 = 没有 */
    int64_t launch_t_ms;           /* 本发离架时刻，-1 = 未离架 */
    int64_t gate_open_t_ms;        /* 交战门限打开时刻，-1 = 未开 */
    int64_t last_fallback_warn_ms; /* 兜底标定告警限频 */
    float   distance_m;            /* 单目测距（始终更新，门限用连续值） */
    uint8_t range_valid;           /* 测距是否可信（灯够大） */
} dv_control_state;

/* 控制输出 */
typedef struct {
    uint8_t active;   /* 1 = 本次有控制律介入（0 = 舵面应回中立） */
    float pitch;      /* 机体系俯仰舵角，弧度（正 = 抬头方向） */
    float yaw;        /* 机体系偏航舵角，弧度 */
    float roll;       /* 滚转舵角，弧度 */
    /* ---- 诊断（真机可以先不用，调试时很有用） ---- */
    float los_rate_dps;   /* 视线角速率模长，度/秒 */
    float range_m;        /* 单目测距，m */
    float accel_cmd;      /* 指令加速度模长，m/s² */
    float available_accel;/* 本次用的「满舵可用侧向加速度」，m/s² */
    float aim_error_deg;  /* 瞄点相对视线的夹角，度 */
    float engage_ramp;    /* 门限斜坡系数 0~1 */
    float roll_deg;       /* 当前滚转角，度（0 = 机背朝上） */
    dv_vec3 los_world;    /* 世界系视线单位向量（核坐标变换用） */
    uint8_t range_valid;
    uint8_t used_fallback_scale; /* 1 = 本次用了兜底标定（舵角会偏大） */
} dv_output;

/* 告警回调：可选。不注册就静默。msg 是常量字符串，可直接丢进串口/日志。 */
typedef void (*dv_warn_hook)(const char* msg);
void dv_set_warn_hook(dv_warn_hook hook);

/* ------------------------------------------------------------------ */
/* 控制律                                                              */
/* ------------------------------------------------------------------ */

/* 填默认配置（= 联调定稿值，见 README）。上电调用一次，再按本机实测覆盖。 */
void dv_config_default(dv_config* cfg);

/* 上电/换发初始化一次 */
void dv_control_init(dv_control_state* st);

/* 复位（丢目标、还没出膛、命中后调用） */
void dv_control_reset(dv_control_state* st);

/*
 * 计算一帧舵角指令。
 *
 *   st      控制层状态（内部保存跨帧数据）
 *   cfg     配置
 *   los_yaw_rad, los_pitch_rad  视觉给的**机体系**视线角（弧度，右正/上正）
 *   blob_size_px                导向灯表观尺寸（像素，边长/直径，按测距标定填；<=0 表示不可用）
 *   los_t_ms                    **这一对视线角所属的视觉时刻**（ms）。
 *                               每次调用都传同一个值 = 控制与视觉同频；
 *                               视觉 30 Hz、控制 200 Hz 时，只有新帧才更新它
 *                               （视线角速率只在这个时刻前进时才算差分）。
 *   state   本帧镖状态
 *   t_ms    本次调用时刻（ms，与 state.t_ms 同一个单调时钟。门限/斜坡用它）
 *   返回    舵角指令 + 诊断
 *
 * 建议调用频率：与 IMU/姿态回路同频（≥100 Hz）。视觉只有 30 Hz 时，
 * 在两次视觉之间重复调用本函数、`los_t_ms` 保持不变、只更新 state 与 t_ms ——
 * 这样高频调用不会把视线角速率算成 0（速率只在新视觉样本上更新）。
 */
dv_output dv_control_step(dv_control_state* st, const dv_config* cfg, float los_yaw_rad,
                          float los_pitch_rad, float blob_size_px, int64_t los_t_ms,
                          const dv_state* state, int64_t t_ms);

/* 像素 -> 机体系视线角（若电控直接拿图像坐标，用这个；fx/fy 单位像素，cx/cy 主点） */
void dv_pixel_to_los(float u_px, float v_px, float fx, float fy, float cx, float cy,
                     float* out_yaw_rad, float* out_pitch_rad);

/* ------------------------------------------------------------------ */
/* 舵机侧：四片几何混合 + 位置环（从仿真器 guided_dart 移植，可直接用）  */
/* ------------------------------------------------------------------ */

/*
 * 四尾翼几何混合（X 字布局的标准做法）：
 *     θ_i = gain · (pitch·cosφ_i − yaw·sinφ_i + roll)
 * φ_i = 该片铰链的方位角（机体系 x-y 平面内，从 +x 起算，逆时针为正）。
 * 这样：纯俯仰/偏航指令不产生滚转力矩（Σθ=0）；滚转指令不产生净侧向力。
 *
 * 本机模型实测铰链方位角：尾翼B −45°、主翼B −45°、主翼A +45°、尾翼A −135°
 * （即作为"线"只有 ±45° 两条，A/B 各自成对）。把实测值填进 phi_rad[]。
 *
 *   gain  几何混合增益（仿真里 0.5；1.0 = 用满舵行程）
 */
void dv_mix_fins(const float phi_rad[4], float gain, float pitch, float yaw, float roll,
                 float max_angle_rad, float out_angle_rad[4]);

/* 舵机位置环（串级 PID 的外环，让舵面跟上期望角）+ 速率限幅。
 * 每次按控制周期调用；dt 单位秒。state 里的积分/上一拍误差需跨帧保存。 */
typedef struct {
    float integral;      /* 积分累加器（误差·秒） */
    float prev_error;    /* 上一拍误差 */
    uint8_t initialized; /* 0 = 首拍，内部会跳过微分 */
} dv_servo_state;

typedef struct {
    uint8_t enabled;
    float kp, ki, kd;
    float integral_limit;
    float output_limit_deg; /* 这一层允许叠加的最大角度 */
    float max_angle_deg;    /* 舵面机械限幅 */
    float max_rate_deg_s;   /* 舵机最大角速度 */
} dv_servo_cfg;

void dv_servo_init(dv_servo_state* st);
/*
 *   desired_rad        期望舵角（来自 dv_control_step 或 dv_mix_fins）
 *   current_rad        当前实际舵角（反馈）
 *   返回 target_rad    这一拍要驱动到的角度（位置环输出，已限幅）
 * 之后再自己做速率限幅（或调用 dv_servo_rate_limit）送到执行器。
 */
float dv_servo_step(dv_servo_state* st, const dv_servo_cfg* cfg, float desired_rad,
                    float current_rad, float dt);

/* 速率限幅：把 (current -> target) 的变化限制在 max_rate·dt 内 */
float dv_servo_rate_limit(float current_rad, float target_rad, float max_rate_rad_s, float dt);

#ifdef __cplusplus
}
#endif

#endif /* DV_CONTROL_H */
