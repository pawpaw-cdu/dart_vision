/*
 * 制导飞镖控制层 —— C 实现（逐行对照 /dart_vision/src/control.cpp）
 * 无动态内存、无 IO，只用 <math.h>。
 */

#include "dv_control.h"

#include <math.h>
#include <string.h>

#define DV_RAD_TO_DEG 57.29577951308232f
#define DV_DEG_TO_RAD 0.017453292519943295f

/* ------------------------------------------------------------------ */
/* 小工具                                                              */
/* ------------------------------------------------------------------ */

static dv_warn_hook g_warn_hook = 0;

void dv_set_warn_hook(dv_warn_hook hook) { g_warn_hook = hook; }

static void dv_warn(const char* msg) {
    if (g_warn_hook != 0) {
        g_warn_hook(msg);
    }
}

static float dv_clampf(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

static dv_vec3 dv_v3(float x, float y, float z) {
    dv_vec3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

static dv_vec3 dv_v3_add(dv_vec3 a, dv_vec3 b) { return dv_v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static dv_vec3 dv_v3_sub(dv_vec3 a, dv_vec3 b) { return dv_v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static dv_vec3 dv_v3_scale(dv_vec3 a, float s) { return dv_v3(a.x * s, a.y * s, a.z * s); }
static float dv_v3_dot(dv_vec3 a, dv_vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

static dv_vec3 dv_v3_cross(dv_vec3 a, dv_vec3 b) {
    return dv_v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

static float dv_v3_len(dv_vec3 a) { return sqrtf(dv_v3_dot(a, a)); }

static dv_vec3 dv_v3_normalize(dv_vec3 a) {
    const float n = dv_v3_len(a);
    if (n < 1.0e-6f) {
        return dv_v3(0.0f, 0.0f, 0.0f);
    }
    return dv_v3_scale(a, 1.0f / n);
}

static dv_quat dv_quat_conj(dv_quat q) {
    dv_quat r;
    r.w = q.w;
    r.x = -q.x;
    r.y = -q.y;
    r.z = -q.z;
    return r;
}

/* 四元数 (w,x,y,z) 旋转向量：v' = v + 2w(q×v) + 2q×(q×v) */
static dv_vec3 dv_rotate(dv_quat q, dv_vec3 v) {
    const dv_vec3 u = dv_v3(q.x, q.y, q.z);
    const dv_vec3 t = dv_v3_scale(dv_v3_cross(u, v), 2.0f);
    return dv_v3_add(dv_v3_add(v, dv_v3_scale(t, q.w)), dv_v3_cross(u, t));
}

static dv_vec3 dv_inverse_rotate(dv_quat q, dv_vec3 v) { return dv_rotate(dv_quat_conj(q), v); }

/*
 * 机体系视线方向（相机固连在镖头时用这个）。
 * 机体系约定：x 向右、y 向上、机头朝 -z；视觉给的 yaw 右正、pitch 上正。
 */
static dv_vec3 dv_camera_dir_in_body(float yaw, float pitch) {
    const float cy = cosf(yaw), sy = sinf(yaw);
    const float cp = cosf(pitch), sp = sinf(pitch);
    return dv_v3_normalize(dv_v3(sy * cp, sp, -cy * cp));
}

/*
 * 相机随速度矢量转（虚拟云台，align_to_velocity = true 的旧行为）。
 * 真机相机是固连的，一般用不到；留着做对照，避免两套约定混用。
 */
static dv_vec3 dv_camera_dir_in_world(dv_vec3 velocity, float yaw, float pitch) {
    const dv_vec3 forward = dv_v3_normalize(velocity);
    if (dv_v3_dot(forward, forward) < 0.5f) {
        return dv_v3(0.0f, 0.0f, -1.0f);
    }
    dv_vec3 world_up = dv_v3(0.0f, 1.0f, 0.0f);
    if (fabsf(dv_v3_dot(forward, world_up)) > 0.999f) {
        world_up = dv_v3(0.0f, 0.0f, 1.0f);
    }
    const dv_vec3 right = dv_v3_normalize(dv_v3_cross(forward, world_up));
    const dv_vec3 up = dv_v3_normalize(dv_v3_cross(right, forward));
    const float cy = cosf(yaw), sy = sinf(yaw);
    const float cp = cosf(pitch), sp = sinf(pitch);
    return dv_v3_normalize(dv_v3_add(
        dv_v3_add(dv_v3_scale(right, sy * cp), dv_v3_scale(up, sp)),
        dv_v3_scale(forward, cy * cp)));
}

/* ------------------------------------------------------------------ */
/* 配置默认值 = 联调定稿值                                             */
/* ------------------------------------------------------------------ */

void dv_config_default(dv_config* cfg) {
    if (cfg == 0) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->enabled = 1;

    cfg->pn_gain = 0.0f;   /* λ̇ 项会和重力打架，当前定稿关掉（ZEM 已含位置+速度反馈）*/
    cfg->zem_gain = 0.6f;  /* 1.0 = 最小时间解，太激进：实测同一配置会 12/15 翻 0/15 */
    cfg->aim_error_gain = 0.0f;
    cfg->accel_bias_mps2 = 0.2f;
    cfg->gravity_mps2 = 9.81f;
    cfg->nose_lead_m = 0.1173f; /* = 镖身碰撞盒半长；制导瞄机头而不是瞄质心 */

    cfg->camera_body_fixed = 1;
    cfg->light_diameter_m = 0.10f;
    cfg->focal_px = 301.72f;
    cfg->range_min_blob_px = 4.0f;
    cfg->min_range_m = 1.5f;

    cfg->aim_offset_up_m = 0.140f;      /* 几何 +0.07 + 校靶余量（见 README）*/
    cfg->aim_offset_forward_m = 0.10f;

    cfg->rate_filter_secs = 0.12f;
    cfg->max_sample_jump_deg = 3.0f;

    cfg->engage_after_launch_secs = 1.40f;
    cfg->engage_range_m = 8.0f;
    cfg->engage_ramp_secs = 0.25f;

    cfg->availability_mps2 = 6.7f;
    cfg->reference_speed_mps = 18.0f;
    cfg->max_fin_deg = 20.0f;
    cfg->roll_gain = 0.0f;

    cfg->pitch_open_loop_deg = 0.0f;
    cfg->yaw_open_loop_deg = 0.0f;
    cfg->roll_open_loop_deg = 0.0f;
}

/* ------------------------------------------------------------------ */
/* 复位/初始化                                                         */
/* ------------------------------------------------------------------ */

void dv_control_reset(dv_control_state* st) {
    if (st == 0) {
        return;
    }
    st->los_rate = dv_v3(0.0f, 0.0f, 0.0f);
    st->last_los = dv_v3(0.0f, 0.0f, 0.0f);
    st->last_sample_t_ms = -1;
    st->gate_open_t_ms = -1;
    st->distance_m = 0.0f;
    st->range_valid = 0;
}

void dv_control_init(dv_control_state* st) {
    if (st == 0) {
        return;
    }
    memset(st, 0, sizeof(*st));
    st->last_sample_t_ms = -1;
    st->launch_t_ms = -1;
    st->gate_open_t_ms = -1;
    st->last_fallback_warn_ms = -100000;
}

/* ------------------------------------------------------------------ */
/* 主控制律                                                            */
/* ------------------------------------------------------------------ */

dv_output dv_control_step(dv_control_state* st, const dv_config* cfg, float los_yaw_rad,
                          float los_pitch_rad, float blob_size_px, int64_t los_t_ms,
                          const dv_state* state, int64_t t_ms) {
    dv_output out;
    dv_vec3 velocity;
    dv_vec3 los;
    dv_vec3 aim_vec;
    dv_vec3 aim_dir;
    dv_vec3 rate;
    dv_vec3 accel;
    dv_vec3 local;
    float speed;
    float max_fin;
    float ramp;
    float closing_speed;
    float speed_ratio;
    float reported_scale;
    float accel_scale;
    float aim_error_deg;
    uint8_t new_sample;
    uint8_t gate_closed;

    memset(&out, 0, sizeof(out));
    out.los_world = dv_v3(0.0f, 0.0f, -1.0f);
    out.engage_ramp = 1.0f;

    if (st == 0 || cfg == 0 || state == 0) {
        return out;
    }
    if (!cfg->enabled || !state->valid || !state->launched || state->hit) {
        st->launch_t_ms = -1;
        dv_control_reset(st);
        return out;
    }

    /* 本发离架时刻（用状态时间戳，别用视觉帧时间） */
    if (st->launch_t_ms < 0) {
        st->launch_t_ms = state->t_ms;
    }

    max_fin = (cfg->max_fin_deg > 0.0f ? cfg->max_fin_deg : 0.0f) * DV_DEG_TO_RAD;

    /* ---- 静开环标定：直接发固定舵角，绕过制导律/门限/斜坡 ---- */
    if (cfg->pitch_open_loop_deg != 0.0f || cfg->yaw_open_loop_deg != 0.0f ||
        cfg->roll_open_loop_deg != 0.0f) {
        out.pitch = dv_clampf(cfg->pitch_open_loop_deg * DV_DEG_TO_RAD, -max_fin, max_fin);
        out.yaw = dv_clampf(cfg->yaw_open_loop_deg * DV_DEG_TO_RAD, -max_fin, max_fin);
        out.roll = dv_clampf(cfg->roll_open_loop_deg * DV_DEG_TO_RAD, -max_fin, max_fin);
        out.active = 1;
        out.range_m = st->distance_m;
        return out;
    }

    velocity = state->velocity;
    speed = dv_v3_len(velocity);
    if (speed < 1.0f) { /* 还没出膛 */
        dv_control_reset(st);
        return out;
    }

    /* 1) 机体系视线角 -> 世界系视线方向 */
    los = cfg->camera_body_fixed ? dv_rotate(state->rotation,
                                             dv_camera_dir_in_body(los_yaw_rad, los_pitch_rad))
                                 : dv_camera_dir_in_world(velocity, los_yaw_rad, los_pitch_rad);
    if (dv_v3_dot(los, los) < 0.5f) {
        return out;
    }
    out.los_world = los;

    /* 2) 单目测距：R = D·fx/像素。distance 始终更新，range_valid 只表示「够不够准」 */
    if (cfg->light_diameter_m > 0.0f && blob_size_px > 0.5f) {
        st->distance_m = cfg->light_diameter_m * cfg->focal_px / blob_size_px;
        st->range_valid = (blob_size_px >= (cfg->range_min_blob_px > 1.0f
                                                ? cfg->range_min_blob_px
                                                : 1.0f))
                              ? 1
                              : 0;
    } else {
        st->range_valid = 0;
    }

    /* 3) 瞄点 = 灯位置 + 靶面偏移（世界系：(0, up, forward)，forward = +z 朝靶内） */
    aim_vec = dv_v3_scale(los, st->distance_m);
    aim_error_deg = 0.0f;
    if (st->range_valid && st->distance_m > cfg->min_range_m &&
        (fabsf(cfg->aim_offset_up_m) > 1.0e-6f ||
         fabsf(cfg->aim_offset_forward_m) > 1.0e-6f)) {
        const dv_vec3 offset = dv_v3(0.0f, cfg->aim_offset_up_m, cfg->aim_offset_forward_m);
        const dv_vec3 corrected =
            dv_v3_normalize(dv_v3_add(dv_v3_scale(los, st->distance_m), offset));
        if (dv_v3_dot(corrected, corrected) > 0.5f) {
            aim_vec = dv_v3_add(dv_v3_scale(los, st->distance_m), offset);
            aim_error_deg = acosf(dv_clampf(dv_v3_dot(los, corrected), -1.0f, 1.0f)) * DV_RAD_TO_DEG;
        }
    }
    aim_dir = dv_v3_normalize(aim_vec);
    if (dv_v3_dot(aim_dir, aim_dir) < 0.5f) {
        aim_dir = los;
    }

    /* 3b) 瞄机头：把瞄点回退一个「机头前伸矢量」
     *     制导把质心导向瞄点，而判定看镖头。不回退时实测命中瞬间机头还在
     *     靶面前方 0.15 m、上方 0.11 m —— 画面里就是「镖身擦靶、机头悬空」。 */
    if (cfg->nose_lead_m > 1.0e-6f) {
        const dv_vec3 forward = dv_rotate(state->rotation, dv_v3(0.0f, 0.0f, -1.0f));
        if (dv_v3_dot(forward, forward) > 0.5f) {
            aim_vec = dv_v3_sub(aim_vec, dv_v3_scale(forward, cfg->nose_lead_m));
            aim_dir = dv_v3_normalize(aim_vec);
            if (dv_v3_dot(aim_dir, aim_dir) < 0.5f) {
                aim_dir = los;
            }
        }
    }

    /* 4) 世界系视线角速率：**只在新视觉样本上**差分（los_t_ms 前进时）。
     *    这样 200 Hz 调用 + 30 Hz 视觉时，速率不会被「同一帧喂两次」冲成 0。 */
    rate = st->los_rate;
    new_sample = (los_t_ms > st->last_sample_t_ms) ? 1 : 0;
    if (new_sample) {
        if (st->last_sample_t_ms >= 0) {
            const float dt = (float)(los_t_ms - st->last_sample_t_ms) / 1000.0f;
            if (dt > 1.0e-3f && dt < 0.5f) {
                const dv_vec3 raw = dv_v3_scale(dv_v3_sub(los, st->last_los), 1.0f / dt);
                const float jump_deg =
                    acosf(dv_clampf(dv_v3_dot(st->last_los, los), -1.0f, 1.0f)) * DV_RAD_TO_DEG;
                if (jump_deg <= cfg->max_sample_jump_deg) {
                    const float tau = cfg->rate_filter_secs > 1.0e-3f ? cfg->rate_filter_secs
                                                                      : 1.0e-3f;
                    const float alpha = dv_clampf(dt / (dt + tau), 0.0f, 1.0f);
                    st->los_rate = dv_v3_add(st->los_rate,
                                             dv_v3_scale(dv_v3_sub(raw, st->los_rate), alpha));
                    rate = st->los_rate;
                }
            }
        }
        st->last_los = los;
        st->last_sample_t_ms = los_t_ms;
    }

    /* 5) 交战门限：优先「发射后计时」（逐发介入点一致），否则用单目测距 */
    if (cfg->engage_after_launch_secs > 1.0e-6f) {
        const int64_t need = (int64_t)(cfg->engage_after_launch_secs * 1000.0f);
        gate_closed = (state->t_ms - st->launch_t_ms) < need ? 1 : 0;
    } else {
        gate_closed = (cfg->engage_range_m > 0.0f && st->distance_m > cfg->min_range_m &&
                       st->distance_m > cfg->engage_range_m)
                          ? 1
                          : 0;
    }
    if (gate_closed) {
        st->gate_open_t_ms = -1;
        out.active = 1; /* 门限内也置 active：舵角为 0，但控制链是通的 */
        out.range_m = st->distance_m;
        return out;
    }
    if (st->gate_open_t_ms < 0) {
        st->gate_open_t_ms = t_ms;
    }
    ramp = (cfg->engage_ramp_secs > 1.0e-6f)
               ? dv_clampf((float)(t_ms - st->gate_open_t_ms) /
                               (cfg->engage_ramp_secs * 1000.0f),
                           0.0f, 1.0f)
               : 1.0f;

    /* 6) 指令加速度（世界系） */
    closing_speed = speed; /* 靶视为静止 */
    speed_ratio = speed / (cfg->reference_speed_mps > 1.0f ? cfg->reference_speed_mps : 1.0f);
    reported_scale = state->available_accel;
    accel_scale = reported_scale > 0.0f
                      ? reported_scale
                      : cfg->availability_mps2 * speed_ratio * speed_ratio;
    if (accel_scale < 1.0f) {
        accel_scale = 1.0f;
    }
    out.used_fallback_scale = (reported_scale > 0.0f) ? 0 : 1;
    if (out.used_fallback_scale && cfg->availability_mps2 > 0.0f) {
        if (t_ms - st->last_fallback_warn_ms > 1500) {
            st->last_fallback_warn_ms = t_ms;
            /* 兜底标定 14 m/s 时约 4.0 m/s²，而真实可用约 8.0 -> 舵角偏大 ~2 倍 */
            dv_warn("dv_control: no available_accel from IMU/STATE -> using fallback scale; "
                    "fin deflection will be roughly 2x too large at 14 m/s");
        }
    }

    /* 6a) 比例导引项（当前定稿 pn_gain = 0，保留以便对照） */
    accel = dv_v3_scale(rate, cfg->pn_gain * closing_speed);
    /* 6b) 常值竖直偏置（世界系 +Y，正值向上） */
    accel = dv_v3_add(accel, dv_v3(0.0f, cfg->accel_bias_mps2, 0.0f));

    /* 6c) 零控脱靶量（ZEM）位置反馈 —— 当前主力项
     *
     *   T    = R / Vc
     *   miss = 瞄点 − ( p + v·T + ½gT² )      自由弹道会落在哪（必须带重力项！）
     *   miss⊥ = miss − v̂(miss·v̂)              只留垂直于速度的分量（舵面力垂直于来流）
     *   a   += miss⊥ · zem_gain · 2/T²
     *
     * 两个坑（实测都会把镖压到靶面下方 ~0.5 m）：
     *   ① 漏掉 ½gT²：把重力本来就会给的下坠算成脱靶，再补一遍；
     *   ② 不投影掉沿速度分量：T 用得到绿灯的斜距、瞄点在灯后面 0.1 m，预测点会
     *      「冲过」瞄点平面 ~0.8 m，乘 2/T² 后是 ~8 m/s² 的倒推力。
     */
    if (cfg->zem_gain > 1.0e-6f && st->distance_m > cfg->min_range_m) {
        const float t = st->distance_m / (closing_speed > 1.0f ? closing_speed : 1.0f);
        if (t > 1.0e-3f) {
            const dv_vec3 gravity = dv_v3(0.0f, -cfg->gravity_mps2, 0.0f);
            const dv_vec3 predicted = dv_v3_add(
                dv_v3_scale(velocity, t), dv_v3_scale(gravity, 0.5f * t * t));
            const dv_vec3 miss = dv_v3_sub(aim_vec, predicted);
            const dv_vec3 vhat = dv_v3_scale(velocity, 1.0f / speed);
            const dv_vec3 miss_perp = dv_v3_sub(miss, dv_v3_scale(vhat, dv_v3_dot(miss, vhat)));
            const float gain = cfg->zem_gain * 2.0f / (t * t);
            accel = dv_v3_add(accel, dv_v3_scale(miss_perp, gain));
        }
    }

    /* 6d) 老的「灯→靶心」几何偏角项（固定 <0.3°，无位置信息，当前关掉） */
    if (cfg->aim_error_gain > 0.0f && st->range_valid && st->distance_m > cfg->min_range_m) {
        const dv_vec3 error_dir = dv_v3_sub(aim_dir, los);
        accel = dv_v3_add(accel,
                          dv_v3_scale(error_dir,
                                      cfg->aim_error_gain * 2.0f * closing_speed * closing_speed /
                                          st->distance_m));
    }

    /* 7) 世界系 -> 机体系，再换算成舵角（舵角 = 指令 / 满舵可用加速度 × 满舵） */
    local = dv_inverse_rotate(state->rotation, accel);
    out.pitch = dv_clampf(local.y / accel_scale, -1.0f, 1.0f) * max_fin * ramp;
    out.yaw = dv_clampf(local.x / accel_scale, -1.0f, 1.0f) * max_fin * ramp;
    out.roll = 0.0f;
    out.active = 1;
    out.los_rate_dps = dv_v3_len(rate) * DV_RAD_TO_DEG;
    out.range_m = st->distance_m;
    out.accel_cmd = dv_v3_len(accel);
    out.available_accel = accel_scale;
    out.aim_error_deg = aim_error_deg;
    out.range_valid = st->range_valid;
    out.engage_ramp = ramp;

    /* 8) 滚转稳定（roll_gain = 0 时不动作）
     *    滚转误差 = (机体上轴 × 世界竖直) · 机体前向 = −sin(bank) */
    {
        const dv_vec3 body_up = dv_rotate(state->rotation, dv_v3(0.0f, 1.0f, 0.0f));
        const dv_vec3 body_forward = dv_rotate(state->rotation, dv_v3(0.0f, 0.0f, -1.0f));
        const float roll_error =
            dv_v3_dot(dv_v3_cross(body_up, dv_v3(0.0f, 1.0f, 0.0f)), body_forward);
        out.roll = (cfg->roll_gain != 0.0f)
                       ? dv_clampf(-roll_error * cfg->roll_gain, -1.0f, 1.0f) * max_fin * ramp
                       : 0.0f;
        out.roll_deg = asinf(dv_clampf(roll_error, -1.0f, 1.0f)) * DV_RAD_TO_DEG;
    }
    return out;
}

/* ------------------------------------------------------------------ */
/* 像素 -> 机体系视线角                                                */
/* ------------------------------------------------------------------ */

void dv_pixel_to_los(float u_px, float v_px, float fx, float fy, float cx, float cy,
                     float* out_yaw_rad, float* out_pitch_rad) {
    /* 图像 u 向右、v 向下；机体系 yaw 右正、pitch 上正 */
    if (out_yaw_rad != 0) {
        *out_yaw_rad = atanf((u_px - cx) / fx);
    }
    if (out_pitch_rad != 0) {
        *out_pitch_rad = atanf((cy - v_px) / fy);
    }
}

/* ------------------------------------------------------------------ */
/* 四片几何混合                                                        */
/* ------------------------------------------------------------------ */

void dv_mix_fins(const float phi_rad[4], float gain, float pitch, float yaw, float roll,
                 float max_angle_rad, float out_angle_rad[4]) {
    int i;
    if (phi_rad == 0 || out_angle_rad == 0) {
        return;
    }
    for (i = 0; i < 4; ++i) {
        const float pitch_gain = cosf(phi_rad[i]);
        const float yaw_gain = -sinf(phi_rad[i]);
        const float roll_gain = 1.0f;
        const float angle =
            gain * (pitch * pitch_gain + yaw * yaw_gain + roll * roll_gain);
        out_angle_rad[i] = dv_clampf(angle, -max_angle_rad, max_angle_rad);
    }
}

/* ------------------------------------------------------------------ */
/* 舵机位置环 + 速率限幅                                               */
/* ------------------------------------------------------------------ */

void dv_servo_init(dv_servo_state* st) {
    if (st == 0) {
        return;
    }
    memset(st, 0, sizeof(*st));
}

float dv_servo_step(dv_servo_state* st, const dv_servo_cfg* cfg, float desired_rad,
                    float current_rad, float dt) {
    float max_angle_rad;
    float desired;
    float error;
    float candidate_integral;
    float derivative;
    float raw_correction;
    float correction;
    float output_limit_rad;
    float integral_limit;
    uint8_t saturated_in_error_direction;

    if (cfg == 0) {
        return desired_rad;
    }
    max_angle_rad = cfg->max_angle_deg * DV_DEG_TO_RAD;
    desired = dv_clampf(desired_rad, -max_angle_rad, max_angle_rad);
    if (st == 0 || !cfg->enabled) {
        if (st != 0) {
            st->integral = 0.0f;
            st->prev_error = 0.0f;
            st->initialized = 0;
        }
        return desired;
    }
    if (dt < 1.0e-6f) {
        dt = 1.0e-6f;
    }
    if (dt > 0.05f) {
        dt = 0.05f;
    }
    output_limit_rad = cfg->output_limit_deg * DV_DEG_TO_RAD;
    if (output_limit_rad < 0.0f) {
        output_limit_rad = 0.0f;
    }
    integral_limit = cfg->integral_limit > 0.0f ? cfg->integral_limit : 0.0f;

    error = desired - current_rad;
    candidate_integral = dv_clampf(st->integral + error * dt, -integral_limit, integral_limit);
    derivative = st->initialized ? (error - st->prev_error) / dt : 0.0f;
    raw_correction = (cfg->kp > 0.0f ? cfg->kp : 0.0f) * error +
                     (cfg->ki > 0.0f ? cfg->ki : 0.0f) * candidate_integral +
                     (cfg->kd > 0.0f ? cfg->kd : 0.0f) * derivative;
    correction = dv_clampf(raw_correction, -output_limit_rad, output_limit_rad);

    /* 饱和且误差同向时不再累积积分（抗积分饱和） */
    saturated_in_error_direction =
        ((raw_correction > output_limit_rad && error > 0.0f) ||
         (raw_correction < -output_limit_rad && error < 0.0f))
            ? 1
            : 0;
    if (!saturated_in_error_direction) {
        st->integral = candidate_integral;
    }
    st->prev_error = error;
    st->initialized = 1;
    return dv_clampf(desired + correction, -max_angle_rad, max_angle_rad);
}

float dv_servo_rate_limit(float current_rad, float target_rad, float max_rate_rad_s, float dt) {
    float max_delta;
    float delta;
    if (dt <= 0.0f) {
        return current_rad;
    }
    max_delta = max_rate_rad_s * dt;
    if (max_delta < 0.0f) {
        max_delta = -max_delta;
    }
    delta = dv_clampf(target_rad - current_rad, -max_delta, max_delta);
    return current_rad + delta;
}
