#include "pitch_balance.h"
#include "math.h"
/* IMU 安装偏移量（rad）：车体水平放置时 angle_roll 的读数 */
#define ANGLE_OFFSET    (-3.6f) /* 车体水平时 angle_roll 的读数 */
#define DEG_TO_RAD   (0.01745f)
#define RAD_TO_DEG ( 57.29578f)
/*
void pitch_balance_control(const Sensor_data_t *sensor_data,
    PID_Controller_t *pid_speed, PID_Controller_t *pid_angle, PID_Controller_t *pid_gyro,
    Motor_cmd_duty_t *motor_cmd){

    float speed_cur = (sensor_data->motor_left_speed + sensor_data->motor_right_speed) / 2.0f;
    float angle_target = pid_calculate(pid_speed, 0.0f, speed_cur); 

    float angle_cur = sensor_data->angle_pitch - ANGLE_OFFSET * DEG_TO_RAD; 
    float gyro_target = pid_calculate(pid_angle, angle_target, angle_cur);

    float gyro_cur = sensor_data->gyro_pitch;
    float pwm_base = pid_calculate(pid_gyro, gyro_target, gyro_cur);

    float pwm_left  = pwm_base; 
    float pwm_right = -pwm_base; 

    motor_cmd->left_motor_pwm  = ROUND(pwm_left);
    motor_cmd->right_motor_pwm = ROUND(pwm_right);
}
*/

// 假设在头文件或配置中定义补偿系数
#define K_GRAVITY_COMP  50.0f  // 重力补偿基准系数（需根据实际载重和动力微调）

void pitch_balance_control(const Sensor_data_t *sensor_data,
    PID_Controller_t *pid_speed, PID_Controller_t *pid_angle, PID_Controller_t *pid_gyro,
    Motor_cmd_duty_t *motor_cmd){

    // ---- 1. 速度环计算 ----
    float speed_cur = (sensor_data->motor_left_speed + sensor_data->motor_right_speed) / 2.0f;
    float speed_output = pid_calculate(pid_speed, 0.0f, speed_cur); 

    // ---- 2. 角度环计算 ----
    float angle_cur = sensor_data->angle_pitch - ANGLE_OFFSET * DEG_TO_RAD; 
    float angle_output = pid_calculate(pid_angle, 0.0f, angle_cur);

    // ---- 3. 并环叠加 ----
    float gyro_target = angle_output + speed_output;

    // ---- 4. 角速度环计算 ----
    float gyro_cur = sensor_data->gyro_pitch;
    float pwm_base = pid_calculate(pid_gyro, gyro_target, gyro_cur);

    // ======== 非线性重量/重力矩前馈补偿 ========
    // 倾翻力矩 T = m * g * L * sin(theta)
    // 当角度变大或重量增加时，前馈输出非线性增加
    float ff_gravity = K_GRAVITY_COMP * sinf(angle_cur);
    
    // 将前馈直接叠加到最终的 PWM 输出上
    pwm_base += ff_gravity;
    // ==========================================

    // 电机输出赋值
    float pwm_left  = pwm_base; 
    float pwm_right = -pwm_base; 

    motor_cmd->left_motor_pwm  = ROUND(pwm_left);
    motor_cmd->right_motor_pwm = ROUND(pwm_right);
}