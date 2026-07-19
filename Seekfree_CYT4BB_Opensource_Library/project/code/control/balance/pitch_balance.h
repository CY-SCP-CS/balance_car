#ifndef PITCH_BALANCE_H
#define PITCH_BALANCE_H

#include "../../lib/pid/pid_calculate.h"
#include "../../app/robot_control/types.h"

/* 纯平衡控制: 角度→角速度串级 + 重力前馈, 返回平衡 PWM */
float balance_control(const Sensor_data_t *sensor,
    PID_Controller_t *pid_angle, PID_Controller_t *pid_gyro);

/* 独立速度环: 满频 PID, 返回速度 PWM */
float speed_control(const Sensor_data_t *sensor,
    PID_Controller_t *pid_speed, float target_speed);

/* 模糊 PID 版本 (保留, 未使用) */
void pitch_balance_control_fuzzy_pid(const Sensor_data_t *sensor,
    PID_Controller_t *pid_speed, PID_Controller_t *pid_angle, PID_Controller_t *pid_gyro,
    Motor_cmd_duty_t *motor_cmd, float target_speed);

/* 复位 pitch_balance_control_fuzzy_pid 内部 static 变量 */
void pitch_balance_reset_statics(void);

#endif /* PITCH_BALANCE_H */
