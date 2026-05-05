#include "balance_ctrl.h"

#include <math.h>

#include "../../common/utils.h"

float run_pitch_control(const Feedback_Data_t *fb,
                        const Safety_Limits_t *lim,
                        PID_Controller_t *outer_pid,
                        PID_Controller_t *inner_pid) {
    float pitch_speed_target = pid_calculate(outer_pid, 0.0f, fb->body_pitch);
    float torque = pid_calculate(inner_pid, pitch_speed_target, fb->gyro_pitch_rate);

    return clamp(torque, -lim->max_wheel_torque, lim->max_wheel_torque);
}

float run_roll_control(const Feedback_Data_t *fb,
                       const Safety_Limits_t *lim,
                       PID_Controller_t *pid_roll) {
    if (fabsf(fb->body_roll) > lim->roll_safe_threshold) {
        return 0.0f;
    }

    float roll_target = fb->on_bridge ? 0.0f : fb->steering_cmd * lim->max_roll_angle;
    float leg_length_delta = pid_calculate(pid_roll, roll_target, fb->body_roll);

    return clamp(leg_length_delta, -lim->max_leg_length_change, lim->max_leg_length_change);
}
