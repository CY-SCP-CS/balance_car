#include "controller.h"


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

    float roll_target = fb->on_bridge ? 0.0f : (fb->steering_cmd * lim->max_roll_angle);

    float windup_threshold = lim->max_roll_angle * 0.1f;
    float leg_length_delta = pid_calculate(pid_roll, roll_target, fb->body_roll);

    return clamp(leg_length_delta, -lim->max_leg_length_change, lim->max_leg_length_change);
}


float run_yaw_control(const Feedback_Data_t *fb,
                       const Safety_Limits_t *lim,
                       PID_Controller_t *pid_yaw,
                       float max_yaw_rate) {
    if (fb->on_bridge) {
        return 0.0f;
    }

    float yaw_rate_target = fb->steering_cmd * max_yaw_rate;
    float steer_diff = pid_calculate(pid_yaw, yaw_rate_target, fb->gyro_yaw_rate);

    return clamp(steer_diff, -pid_yaw->output_limit, pid_yaw->output_limit);
}


void run_joint_pd_control(const Output_Data_t *target,
                          const Feedback_Data_t *fb,
                          const Safety_Limits_t *lim,
                          PD_Controller_t *pd_fl, PD_Controller_t *pd_bl,
                          PD_Controller_t *pd_fr, PD_Controller_t *pd_br,
                          Output_Data_t *out) {
    (void)lim;

    out->joint_torque_fl = pd_calculate(pd_fl, target->joint_angle_cmd_fl, fb->motor_angle_fl, fb->motor_vel_fl);
    out->joint_torque_bl = pd_calculate(pd_bl, target->joint_angle_cmd_bl, fb->motor_angle_bl, fb->motor_vel_bl);
    out->joint_torque_fr = pd_calculate(pd_fr, target->joint_angle_cmd_fr, fb->motor_angle_fr, fb->motor_vel_fr);
    out->joint_torque_br = pd_calculate(pd_br, target->joint_angle_cmd_br, fb->motor_angle_br, fb->motor_vel_br);
}


void run_leg_position_control(float leg_length_delta,
                               const LegConfig_t *leg,
                               const Feedback_Data_t *fb,
                               const Safety_Limits_t *lim,
                               PID_Controller_t *pid_x_L, PID_Controller_t *pid_y_L,
                               PID_Controller_t *pid_x_R, PID_Controller_t *pid_y_R,
                               Output_Data_t *out) {
    float x_target = fb->velocity_cmd * lim->max_lateral_offset;

    float y_target_L = leg->nominal_leg_length + leg_length_delta;
    float y_target_R = leg->nominal_leg_length - leg_length_delta;

    float xL, yL, xR, yR;
    five_bar_fk(leg, fb->motor_angle_fl, fb->motor_angle_bl, &xL, &yL);
    five_bar_fk(leg, fb->motor_angle_fr, fb->motor_angle_br, &xR, &yR);

    float dxL = pid_calculate(pid_x_L, x_target, xL);
    float dyL = pid_calculate(pid_y_L, y_target_L, yL);
    five_bar_ik(leg, x_target + dxL, y_target_L + dyL,
                &out->joint_angle_cmd_fl, &out->joint_angle_cmd_bl);

    float dxR = pid_calculate(pid_x_R, x_target, xR);
    float dyR = pid_calculate(pid_y_R, y_target_R, yR);
    five_bar_ik(leg, x_target + dxR, y_target_R + dyR,
                &out->joint_angle_cmd_fr, &out->joint_angle_cmd_br);
}


void run_torque_distribution(float total_torque, float steer_diff,
                              const Safety_Limits_t *lim,
                              Output_Data_t *out) {
    out->wheel_torque_L = clamp(total_torque + steer_diff, -lim->max_wheel_torque, lim->max_wheel_torque);
    out->wheel_torque_R = clamp(total_torque - steer_diff, -lim->max_wheel_torque, lim->max_wheel_torque);
}
