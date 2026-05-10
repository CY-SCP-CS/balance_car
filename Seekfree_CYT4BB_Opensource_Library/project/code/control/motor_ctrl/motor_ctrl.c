#include "motor_ctrl.h"

#include "../../common/utils.h"

void run_joint_pd_control(const Ctrl_Output_t *target,
                          const Ctrl_Input_t *fb,
                          PD_Controller_t *pd_fl,
                          PD_Controller_t *pd_bl,
                          PD_Controller_t *pd_fr,
                          PD_Controller_t *pd_br,
                          Ctrl_Output_t *out) {
    out->joint_torque_fl = pd_calculate(pd_fl, target->joint_angle_cmd_fl,
                                        fb->motor_angle_fl, fb->motor_vel_fl);
    out->joint_torque_bl = pd_calculate(pd_bl, target->joint_angle_cmd_bl,
                                        fb->motor_angle_bl, fb->motor_vel_bl);
    out->joint_torque_fr = pd_calculate(pd_fr, target->joint_angle_cmd_fr,
                                        fb->motor_angle_fr, fb->motor_vel_fr);
    out->joint_torque_br = pd_calculate(pd_br, target->joint_angle_cmd_br,
                                        fb->motor_angle_br, fb->motor_vel_br);
}

void run_torque_distribution(float total_torque,
                             float steer_diff,
                             const Safety_Limits_t *lim,
                             Ctrl_Output_t *out) {
    out->wheel_torque_L = clamp(total_torque + steer_diff,
                                -lim->max_wheel_torque, lim->max_wheel_torque);
    out->wheel_torque_R = clamp(total_torque - steer_diff,
                                -lim->max_wheel_torque, lim->max_wheel_torque);
}
