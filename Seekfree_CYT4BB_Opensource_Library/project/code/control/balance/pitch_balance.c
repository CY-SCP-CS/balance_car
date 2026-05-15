#include "pitch_balance.h"

void pitch_balance_control(const Sensor_data_t *sensor_data,
    PID_Controller_t *pid_angle, PID_Controller_t *pid_gyro,
    PID_Controller_t *pid_speed_left, PID_Controller_t *pid_speed_right,
    Motor_cmd_duty_t *motor_cmd){

    float angle_pitch_cur = sensor_data->angle_pitch;
    float gyro_pitch_tar = pid_calculate(pid_angle, 0, angle_pitch_cur); /* 车体角度环 */

    float gyro_pitch_cur = sensor_data->gyro_pitch;
    float motor_speed_tar = pid_calculate(pid_gyro, gyro_pitch_tar, gyro_pitch_cur); /* 车体角速度环 */

    float motor_left_speed_cur = sensor_data->motor_left_speed;
    float motor_right_speed_cur = sensor_data->motor_right_speed;
    float pwm_left = pid_calculate(pid_speed_left, motor_speed_tar, motor_left_speed_cur);
    float pwm_right = pid_calculate(pid_speed_right, motor_speed_tar, motor_right_speed_cur); /* 电机速度环 */

    motor_cmd->left_motor_pwm  = ROUND(pwm_left);
    motor_cmd->right_motor_pwm = ROUND(pwm_right);
}