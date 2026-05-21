#ifndef LEG_PID_CONTROL_H
#define LEG_PID_CONTROL_H

#include "../../app/robot_control/types.h"
#include "../../lib/pid/pid_calculate.h"

/* 腿部关节 PID 控制器集合 */
typedef struct {
    PID_Controller_t front;  /* 前关节 PID */
    PID_Controller_t back;   /* 后关节 PID */
} Leg_PID_t;

/* 腿部关节目标角度 (rad, 相对限位的偏移量, 正=远离限位) */
typedef struct {
    float front;   /* 前关节目标偏移 (rad) */
    float back;    /* 后关节目标偏移 (rad) */
} Leg_Target_t;

/* 初始化一条腿的关节 PID (kp, ki, kd, out_max, integral_max) */
void leg_pid_init(Leg_PID_t *leg, float kp, float ki, float kd,
                  float out_max, float integral_max);

/* 执行一条腿的 PID 控制
 * pid      - PID 参数
 * target   - 目标角度偏移 (相对限位的 rad)
 * sensor   - 传感器数据 (包含关节角度, 经过标定偏移)
 * side     - LEG_LEFT / LEG_RIGHT
 * motor_cmd- 输出的 PWM
 */
void leg_pid_control(Leg_PID_t *pid,
                     const Leg_Target_t *target,
                     const Sensor_data_t *sensor,
                     LegSide_t side,
                     Motor_cmd_duty_t *motor_cmd);

#endif /* LEG_PID_CONTROL_H */
