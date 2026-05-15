#include "pitch_balance.h"

/* IMU 安装偏移量（rad）：车体水平放置时 angle_roll 的读数 */
#define ANGLE_OFFSET    (-3.7f) /* 车体水平时 angle_roll 的读数 */
#define DEG_TO_RAD   (0.01745f)
#define RAD_TO_DEG ( 57.29578f)
void pitch_balance_control(const Sensor_data_t *sensor_data,
    PID_Controller_t *pid_speed, PID_Controller_t *pid_angle, PID_Controller_t *pid_gyro,
    Motor_cmd_duty_t *motor_cmd){

    float speed_cur = (sensor_data->motor_left_speed + sensor_data->motor_right_speed) / 2.0f;
    float angle_target = pid_calculate(pid_speed, 0.0f, speed_cur); 

    float angle_cur = sensor_data->angle_roll - ANGLE_OFFSET * DEG_TO_RAD; 
    float gyro_target = pid_calculate(pid_angle, angle_target, angle_cur);

    float gyro_cur = sensor_data->gyro_roll;
    float pwm_base = pid_calculate(pid_gyro, gyro_target, gyro_cur);

    float pwm_left  = pwm_base; 
    float pwm_right = pwm_base; 

    motor_cmd->left_motor_pwm  = ROUND(-pwm_left);
    motor_cmd->right_motor_pwm = ROUND(pwm_right);
}