/*
 * dv_control 用法示例（也是自检程序）：
 *   cc -std=c99 -Wall -Wextra -O2 dv_control.c dv_control_example.c -lm -o dv_demo && ./dv_demo
 *
 * 它伪造一条从发射到过靶的弹道，按 200 Hz 调控制律（视觉 30 Hz 只更新视线角），
 * 打印关键时刻的舵角，用来确认移植后的接口接对了。
 */

#include "dv_control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static void on_warn(const char* msg) { printf("  [告警] %s\n", msg); }

#define EX_DEG2RAD 0.017453292519943295f

/* 示例自带的极小向量工具（不依赖 dv_control.c 内部静态函数） */
static dv_vec3 ex_v3(float x, float y, float z) {
    dv_vec3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}
static float ex_len(dv_vec3 a) { return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z); }
static dv_vec3 ex_norm(dv_vec3 a) {
    const float n = ex_len(a);
    return (n < 1e-6f) ? ex_v3(0, 0, 0) : ex_v3(a.x / n, a.y / n, a.z / n);
}
static float ex_clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static dv_vec3 ex_rotate(dv_quat q, dv_vec3 v) { /* 与 dv_control.c 同一公式 */
    const dv_vec3 u = ex_v3(q.x, q.y, q.z);
    const dv_vec3 t = ex_v3(2.0f * (u.y * v.z - u.z * v.y), 2.0f * (u.z * v.x - u.x * v.z),
                            2.0f * (u.x * v.y - u.y * v.x));
    return ex_v3(v.x + q.w * t.x + (u.y * t.z - u.z * t.y),
                 v.y + q.w * t.y + (u.z * t.x - u.x * t.z),
                 v.z + q.w * t.z + (u.x * t.y - u.y * t.x));
}
/* 极简弹道：不接仿真，只为了让示例能跑出合理量级 */
typedef struct {
    dv_vec3 pos;
    dv_vec3 vel;
    dv_quat att;
} fake_dart;

static dv_quat quat_from_axis(float ax, float ay, float az, float angle) {
    const float n = sqrtf(ax * ax + ay * ay + az * az);
    const float s = sinf(angle * 0.5f);
    dv_quat q;
    q.w = cosf(angle * 0.5f);
    q.x = ax / n * s;
    q.y = ay / n * s;
    q.z = az / n * s;
    return q;
}

int main(void) {
    dv_config cfg;
    dv_control_state st;
    dv_servo_cfg servo_cfg;
    dv_servo_state servo[4];
    float phi[4];
    float fin_cmd[4], fin_actual[4];
    fake_dart dart;
    int i;
    int step;
    const float dt = 0.005f;         /* 200 Hz */
    const int64_t t0_ms = 0;
    dv_output out;
    float los_yaw = 0.0f, los_pitch = 0.0f, blob_px = 0.0f;
    int64_t vision_t_ms = 0; /* 视觉样本时刻：30 Hz 才更新 */
    int vision_counter = 0;

    dv_set_warn_hook(on_warn);
    dv_config_default(&cfg);
    dv_control_init(&st);

    /* 舵机侧：四片实测铰链方位角（尾翼B/主翼B/主翼A/尾翼A） */
    phi[0] = -45.0f * EX_DEG2RAD;
    phi[1] = -45.0f * EX_DEG2RAD;
    phi[2] = 45.0f * EX_DEG2RAD;
    phi[3] = -135.0f * EX_DEG2RAD;
    for (i = 0; i < 4; ++i) {
        dv_servo_init(&servo[i]);
        fin_actual[i] = 0.0f;
    }
    memset(&servo_cfg, 0, sizeof(servo_cfg));
    servo_cfg.enabled = 1;
    servo_cfg.kp = 1.0f;
    servo_cfg.ki = 0.25f;
    servo_cfg.kd = 0.02f;
    servo_cfg.integral_limit = 0.35f;
    servo_cfg.output_limit_deg = 2.6f;
    servo_cfg.max_angle_deg = 20.0f;
    servo_cfg.max_rate_deg_s = 150.0f;

    /* 发射初速 17.66 m/s、仰角 35.5°、朝 +z；机头 = 机体系 -z */
    dart.pos = (dv_vec3){0.0f, -0.53f, -25.56f};
    dart.vel = (dv_vec3){0.0f, 9.7f, 14.4f};
    dart.att = quat_from_axis(1.0f, 0.0f, 0.0f, -35.5f * EX_DEG2RAD);

    printf("步数   t(s)   速度  距离  门限  ctlP(deg) ctlY(deg) 可用a  指令a\n");
    for (step = 0; step < 700; ++step) {
        const int64_t t_ms = t0_ms + (int64_t)(step * dt * 1000.0f);
        dv_state state;

        /* 伪造一位 IMU：位置/速度/姿态 */
        dart.vel.y += -9.81f * dt;
        dart.pos.x += dart.vel.x * dt;
        dart.pos.y += dart.vel.y * dt;
        dart.pos.z += dart.vel.z * dt;

        memset(&state, 0, sizeof(state));
        state.valid = 1;
        state.launched = 1;
        state.t_ms = t_ms;
        state.position = dart.pos;
        state.velocity = dart.vel;
        state.rotation = dart.att;
        /* 真机这里应填「当前速度下满舵可用侧向加速度」；示例先填 0 走兜底 */
        state.available_accel = 0.0f;

        /* 伪造 30 Hz 视觉：控制 200 Hz 跑，只有新帧才更新 los 与 vision_t_ms */
        if (step % 7 == 0) {
            vision_t_ms = t_ms;
            const float dz = -0.7767f - dart.pos.z;
            const float dy = -0.4856f - dart.pos.y;
            const float range = sqrtf(dz * dz + dy * dy);
            /* 灯在机体系：先算世界系方向，再转到机体系（真机由检测+解算给出） */
            const dv_vec3 los_world = ex_norm(ex_v3(-3.27f - dart.pos.x, dy, dz));
            const dv_quat att_conj = {dart.att.w, -dart.att.x, -dart.att.y, -dart.att.z};
            const dv_vec3 los_body = ex_rotate(att_conj, los_world);
            los_yaw = atan2f(los_body.x, -los_body.z);
            los_pitch = asinf(ex_clamp(los_body.y, -1.0f, 1.0f));
            blob_px = (range > 1.0f) ? (cfg.light_diameter_m * cfg.focal_px / range) : 8.0f;
            vision_counter++;
        }

        out = dv_control_step(&st, &cfg, los_yaw, los_pitch, blob_px,
                              vision_t_ms, &state, t_ms);

        /* 舵角 -> 四片（几何混合）-> 位置环 -> 速率限幅 */
        if (out.active) {
            dv_mix_fins(phi, 0.5f, out.pitch, out.yaw, out.roll,
                        cfg.max_fin_deg * EX_DEG2RAD, fin_cmd);
        } else {
            for (i = 0; i < 4; ++i) {
                fin_cmd[i] = 0.0f;
            }
        }
        for (i = 0; i < 4; ++i) {
            const float target = dv_servo_step(&servo[i], &servo_cfg, fin_cmd[i], fin_actual[i], dt);
            fin_actual[i] = dv_servo_rate_limit(fin_actual[i], target,
                                                servo_cfg.max_rate_deg_s * EX_DEG2RAD, dt);
        }

        if (step % 60 == 0 || (step > 250 && step % 20 == 0 && step < 400)) {
            printf("%4d %7.2f %6.1f %6.1f %5.2f  %+8.2f  %+8.2f %6.2f %6.2f\n", step,
                   t_ms / 1000.0, ex_len(dart.vel), out.range_m, out.engage_ramp,
                   out.pitch * 57.29578f, out.yaw * 57.29578f, out.available_accel,
                   out.accel_cmd);
        }
        if (dart.pos.z > 2.0f) {
            break;
        }
    }
    printf("\n过靶附近：z=%.2f m, y=%.3f m（靶心 y=-0.4156）, 视觉帧 %d 帧\n", dart.pos.z,
           dart.pos.y, vision_counter);
    printf("四片舵角(度) = %.2f %.2f %.2f %.2f\n", fin_actual[0] * 57.29578f,
           fin_actual[1] * 57.29578f, fin_actual[2] * 57.29578f, fin_actual[3] * 57.29578f);
    return 0;
}
