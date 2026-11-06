/* C 版控制律：按 gen_inputs.h 的同一串输入跑一遍，打印逐帧输出（与 C++ 原版对比用） */

#include "../dv_control.h"
#include "gen_inputs.h"

#include <stdio.h>
#include <string.h>

int main(void) {
    dv_config cfg;
    dv_control_state st;
    int step;

    dv_config_default(&cfg);
    dv_control_init(&st);

    for (step = 0; step < 700; ++step) {
        gen_input gi;
        dv_state s;
        dv_output o;

        gen_make(step, &gi);
        memset(&s, 0, sizeof(s));
        s.valid = (uint8_t)gi.valid;
        s.launched = (uint8_t)gi.launched;
        s.hit = (uint8_t)gi.hit;
        s.t_ms = gi.t_ms;
        s.position = (dv_vec3){gi.pos[0], gi.pos[1], gi.pos[2]};
        s.velocity = (dv_vec3){gi.vel[0], gi.vel[1], gi.vel[2]};
        s.rotation = (dv_quat){gi.quat[0], gi.quat[1], gi.quat[2], gi.quat[3]};
        s.available_accel = gi.avail;

        o = dv_control_step(&st, &cfg, gi.los_yaw, gi.los_pitch, gi.blob_px, gi.t_ms, &s,
                            gi.t_ms);
        printf("%d %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", step,
               o.active ? 1 : 0, (double)o.pitch, (double)o.yaw, (double)o.roll,
               (double)o.los_rate_dps, (double)o.range_m, (double)o.accel_cmd,
               (double)o.available_accel, (double)o.aim_error_deg, (double)o.engage_ramp,
               (double)o.roll_deg);
    }
    return 0;
}
