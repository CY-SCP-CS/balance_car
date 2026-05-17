#ifndef PITCH_BALANCE_H
#define PITCH_BALANCE_H

#include "../../lib/pid/pid_calculate.h"
#include "../../app/robot_control/types.h"

void pitch_balance_control(const Sensor_data_t *sensor_data,
    PID_Controller_t *pid_speed, PID_Controller_t *pid_angle, PID_Controller_t *pid_gyro,
    Motor_cmd_duty_t *motor_cmd);

#endif /* PITCH_BALANCE_H */
