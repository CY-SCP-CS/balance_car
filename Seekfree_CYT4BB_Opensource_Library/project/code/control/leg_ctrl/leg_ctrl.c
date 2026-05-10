#include "leg_ctrl.h"

void run_leg_position_control(float leg_length_delta,
                              const LegConfig_t *leg,
                              const Ctrl_Input_t *fb,
                              const Safety_Limits_t *lim,
                              PID_Controller_t *pid_x_L,
                              PID_Controller_t *pid_y_L,
                              PID_Controller_t *pid_x_R,
                              PID_Controller_t *pid_y_R,
                              Ctrl_Output_t *out) {
    float x_target = fb->velocity_cmd * lim->max_lateral_offset;
    float y_target_L = leg->nominal_leg_length + leg_length_delta;
    float y_target_R = leg->nominal_leg_length - leg_length_delta;

    float xL;
    float yL;
    float xR;
    float yR;

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
