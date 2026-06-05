#ifndef ANGLE_OFFSET_H
#define ANGLE_OFFSET_H

#include <stdbool.h>
#include <stdint.h>
#include "../../app/robot_control/types.h"
#include "../../common/types.h"

/* 关节标识 */
typedef enum {
    JOINT_LEFT_FRONT = 0,
    JOINT_LEFT_BACK,
    JOINT_RIGHT_FRONT,
    JOINT_RIGHT_BACK,
    JOINT_COUNT
} JointID_t;

/* 标定参数 */
typedef struct
{
    int16_t homing_pwm;             /* 撞限位用 PWM 幅度 (0~10000)    */
    uint16_t stall_cycles;          /* 判定堵转需持续的周期数          */
    uint16_t settle_cycles;         /* 关节开始前的稳定等待 (周期数)   */
    uint16_t timeout_cycles;        /* 整体超时保护 (周期数), 0=禁用   */
    float stall_threshold;          /* 堵转判据：每周期角度变化 < 此值 (rad) */
    int8_t dir[JOINT_COUNT];        /* 各关节撞限位的旋转方向 (+1/-1)  */
} AngleOffset_Config_t;

extern const AngleOffset_Config_t g_angle_offset_default_cfg;

void angle_offset_start(const AngleOffset_Config_t *cfg);

void angle_offset_start_leg(LegSide_t leg, const AngleOffset_Config_t *cfg);

void angle_offset_process(const Sensor_data_t *sensor, Motor_cmd_duty_t *motor_cmd);

bool angle_offset_is_done(void);

bool angle_offset_has_fault(void);

void angle_offset_apply_to_sensor(Sensor_data_t *sensor);

#endif /* ANGLE_OFFSET_H */
