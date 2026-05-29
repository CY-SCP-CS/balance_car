#include "leg_pid_control.h"

void leg_pid_init(Leg_PID_t *leg, float kp, float ki, float kd,
                  float out_max, float integral_max) {
    pid_init(&leg->front, kp, ki, kd, ROBOT_CONTROL_DT, out_max, integral_max);
    pid_init(&leg->back,  kp, ki, kd, ROBOT_CONTROL_DT, out_max, integral_max);
}

void leg_pid_control(Leg_PID_t *pid,
                     const Leg_Target_t *target,
                     const Sensor_data_t *sensor,
                     LegSide_t side,
                     Motor_cmd_duty_t *motor_cmd) {

    float front_angle, back_angle;
    float front_target, back_target;

    if (side == LEG_LEFT) {
        front_angle   = sensor->joint_left_front_angle;
        back_angle    = sensor->joint_left_back_angle;
        front_target  = target->front;
        back_target   = target->back;
    } else {
        front_angle   = sensor->joint_right_front_angle;
        back_angle    = sensor->joint_right_back_angle;
        front_target  = target->front;
        back_target   = target->back;
    }


    float front_out = pid_calculate(&pid->front, front_target, front_angle);
    float back_out  = pid_calculate(&pid->back,  back_target,  back_angle);

    int pwm_front = (int)front_out;
    int pwm_back  = (int)back_out;

    if (side == LEG_LEFT) {

        motor_cmd->left_front_joint_pwm = -pwm_front;
        motor_cmd->left_back_joint_pwm  = -pwm_back;
    } else {
        motor_cmd->right_front_joint_pwm = -pwm_front;
        motor_cmd->right_back_joint_pwm  = -pwm_back;
    }
}
