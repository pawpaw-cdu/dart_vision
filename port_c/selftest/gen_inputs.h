/*
 * C / C++ 共用的确定性输入发生器（保证两版控制律吃到**逐位相同**的输入）。
 * 只做解析计算，不用任何库（除 math.h），因此 C 与 C++ 都能原样编译。
 */
#ifndef DV_GEN_INPUTS_H
#define DV_GEN_INPUTS_H

#include <math.h>
#include <stdint.h>

/* 0 = 填真实可用加速度（正常路径）；1 = 填 0，用来验证兜底标定分支 */
#ifndef GEN_AVAIL_ZERO
#define GEN_AVAIL_ZERO 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t t_ms;
    float los_yaw;      /* 机体系视线角，弧度（右正） */
    float los_pitch;    /* 机体系视线角，弧度（上正） */
    float blob_px;      /* 导向灯表观尺寸，像素 */
    float pos[3];       /* 世界系位置 */
    float vel[3];       /* 世界系速度 */
    float quat[4];      /* w,x,y,z：机体系 -> 世界系 */
    float avail;        /* 满舵可用侧向加速度；=0 时走兜底标定 */
    int   launched;
    int   hit;
    int   valid;
} gen_input;

/* 世界系约定同 dv_control.h：x 右、y 上、z 朝靶；灯心固定 */
static const float GEN_LIGHT[3] = {-3.2f, -0.4856f, -0.8767f};
static const float GEN_DT_S = 0.005f; /* 200 Hz */

/* 简单两步四元数乘（w,x,y,z） */
static void gen_quat_mul(const float a[4], const float b[4], float out[4]) {
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static void gen_quat_from_x_axis(float angle, float out[4]) {
    out[0] = cosf(angle * 0.5f);
    out[1] = sinf(angle * 0.5f);
    out[2] = 0.0f;
    out[3] = 0.0f;
}

/* 机体系 -> 世界系 旋转（与 dv_control.c 同一公式），便于反算视线角 */
static void gen_rotate(const float q[4], const float v[3], float out[3]) {
    const float u[3] = {q[1], q[2], q[3]};
    const float t[3] = {2.0f * (u[1] * v[2] - u[2] * v[1]), 2.0f * (u[2] * v[0] - u[0] * v[2]),
                        2.0f * (u[0] * v[1] - u[1] * v[0])};
    out[0] = v[0] + q[0] * t[0] + (u[1] * t[2] - u[2] * t[1]);
    out[1] = v[1] + q[0] * t[1] + (u[2] * t[0] - u[0] * t[2]);
    out[2] = v[2] + q[0] * t[2] + (u[0] * t[1] - u[1] * t[0]);
}

static void gen_inverse_rotate(const float q[4], const float v[3], float out[3]) {
    const float c[4] = {q[0], -q[1], -q[2], -q[3]};
    gen_rotate(c, v, out);
}

/*
 * 造一条从发射到过靶的弹道（抛物线）+ 随动姿态：
 *   初速 17.66 m/s、仰角 35.5°，姿态取「俯仰角 = 弹道倾角」的固定形式。
 * 目标：让控制律走到门限打开、斜坡、饱和等各条分支。
 */
static void gen_make(int step, gen_input* in) {
    const float t = (float)step * GEN_DT_S;
    const float g = 9.81f;
    float los_world[3];
    float los_body[3];
    float range;
    float pitch_angle;

    in->t_ms = (int64_t)step * 5; /* 200 Hz -> 5 ms/step */
    in->launched = 1;
    in->hit = 0;
    in->valid = 1;

    /* 位置/速度：p0 + v0·t + ½g t² */
    in->pos[0] = 0.0f;
    in->pos[1] = -0.53f + 9.734f * t - 0.5f * g * t * t;   /* v0y = 17.66·sin35.5° */
    in->pos[2] = -25.56f + 14.379f * t;                    /* v0z = 17.66·cos35.5° */
    in->vel[0] = 0.0f;
    in->vel[1] = 9.734f - g * t;
    in->vel[2] = 14.379f;

    /* 姿态：俯仰角跟随弹道倾角（机头 -z 指向飞行方向） */
    pitch_angle = atan2f(in->vel[1], in->vel[2]); /* 正 = 抬头 */
    gen_quat_from_x_axis(-pitch_angle, in->quat);

    /* 视线：世界系方向 -> 机体系 -> 视线角 */
    los_world[0] = GEN_LIGHT[0] - in->pos[0];
    los_world[1] = GEN_LIGHT[1] - in->pos[1];
    los_world[2] = GEN_LIGHT[2] - in->pos[2];
    range = sqrtf(los_world[0] * los_world[0] + los_world[1] * los_world[1] +
                  los_world[2] * los_world[2]);
    los_world[0] /= range;
    los_world[1] /= range;
    los_world[2] /= range;
    gen_inverse_rotate(in->quat, los_world, los_body);
    in->los_yaw = atan2f(los_body[0], -los_body[2]);
    in->los_pitch = asinf(fmaxf(-1.0f, fminf(1.0f, los_body[1])));

    /* 导向灯表观尺寸：D·fx/R（D=0.10, fx=301.72），小距离下限 6 px */
    in->blob_px = 0.10f * 301.72f / (range > 0.5f ? range : 0.5f);
    if (in->blob_px < 6.0f) {
        in->blob_px = 6.0f;
    }

    /* 满舵可用侧向加速度：按仿真器模型 0.0407·v²（14 m/s ≈ 8.0）。
     * 由 GEN_AVAIL_ZERO 控制是否走兜底标定分支。 */
    {
        const float speed = sqrtf(in->vel[0] * in->vel[0] + in->vel[1] * in->vel[1] +
                                  in->vel[2] * in->vel[2]);
        in->avail = (GEN_AVAIL_ZERO != 0) ? 0.0f : 0.0407f * speed * speed;
    }
}

/* 视觉帧率：30 Hz -> 每 7 个 200 Hz 步更新一次视线角；不更新时由调用方沿用旧值。
 * 这里用宏控制是否模拟 30 Hz 视觉（1 = 只在 30 Hz 更新，0 = 每步都更新）。 */
#ifndef GEN_VISION_30HZ
#define GEN_VISION_30HZ 1
#endif

#ifdef __cplusplus
}
#endif

#endif /* DV_GEN_INPUTS_H */
