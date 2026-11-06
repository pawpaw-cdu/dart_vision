/* C++ 原版控制律（dart_vision/src/control.cpp）——用与 C 版完全相同的输入跑一遍 */

#include "dart_vision/control.hpp"
#include "gen_inputs.h"

#include <cstdio>

int main() {
    dart_vision::ControlConfig cfg;
    /* 与 dv_config_default() 逐项对齐 */
    cfg.enabled = true;
    cfg.pn_gain = 0.0;
    cfg.zem_gain = 0.6;
    cfg.aim_error_gain = 0.0;
    cfg.accel_bias_mps2 = 0.2;
    cfg.gravity_mps2 = 9.81;
    cfg.nose_lead_m = 0.1173;
    cfg.availability_mps2 = 6.7;
    cfg.reference_speed_mps = 18.0;
    cfg.max_fin_deg = 20.0;
    cfg.engage_range_m = 8.0;
    cfg.min_range_m = 1.5;
    cfg.light_diameter_m = 0.10;
    cfg.focal_px = 301.72;
    cfg.aim_offset_up_m = 0.140;
    cfg.aim_offset_forward_m = 0.10;
    cfg.rate_filter_secs = 0.12;
    cfg.max_sample_jump_deg = 3.0;
    cfg.range_min_blob_px = 4.0;
    cfg.engage_ramp_secs = 0.25;
    cfg.engage_after_launch_secs = 1.40;
    cfg.roll_gain = 0.0;
    cfg.pitch_open_loop_deg = 0.0;
    cfg.yaw_open_loop_deg = 0.0;
    cfg.roll_open_loop_deg = 0.0;
    cfg.camera_body_fixed = true;

    dart_vision::DartController controller(cfg);

    for (int step = 0; step < 700; ++step) {
        gen_input gi;
        gen_make(step, &gi);

        dart_vision::DartState s;
        s.valid = gi.valid != 0;
        s.launched = gi.launched != 0;
        s.hit = gi.hit != 0;
        s.t_ms = gi.t_ms;
        s.position = cv::Vec3f(gi.pos[0], gi.pos[1], gi.pos[2]);
        s.velocity = cv::Vec3f(gi.vel[0], gi.vel[1], gi.vel[2]);
        s.rotation = cv::Vec4f(gi.quat[0], gi.quat[1], gi.quat[2], gi.quat[3]);
        s.available_accel = gi.avail;

        const dart_vision::ControlOutput o =
            controller.update(gi.los_yaw, gi.los_pitch, gi.blob_px, s, gi.t_ms);

        std::printf("%d %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", step,
                    o.active ? 1 : 0, o.pitch, o.yaw, o.roll, o.los_rate_dps, o.range_m,
                    o.accel_cmd, o.available_accel, o.aim_error_deg, o.engage_ramp,
                    o.roll_deg);
    }
    return 0;
}
