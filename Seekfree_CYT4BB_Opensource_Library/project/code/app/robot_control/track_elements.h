#ifndef TRACK_ELEMENTS_H
#define TRACK_ELEMENTS_H

#include "types.h"
#include "../../control/leg/leg_pid_control.h"
#include <stdbool.h>

/* ===== 1. 原地旋转720度 ===== */
void  track_rotate720_init(void);
void  track_rotate720_start(void);
bool  track_rotate720_is_active(void);
bool  track_rotate720_is_done(void);
bool  track_rotate720_should_suppress_odom(void);
void  track_rotate720_reset(void);
void  track_rotate720_update(Sensor_data_t *sensor, Move_cmd_t *cmd);

/* ===== 2. 单边桥与爬坡 (固定归一化速度) ===== */
void  track_bridge_climb_init(void);
void  track_bridge_climb_activate(void);
void  track_bridge_climb_deactivate(void);
bool  track_bridge_climb_is_active(void);
void  track_bridge_climb_apply(Move_cmd_t *cmd);

/* ===== 3. 颠簸路段 (交替单轮过条 / 柔顺+脱困) ===== */

/* 编译开关: BUMPY_MODE_ALTERNATING vs BUMPY_MODE_COMPLIANT */
#define BUMPY_MODE_ALTERNATING 0
#define BUMPY_MODE_COMPLIANT   1
#define BUMPY_MODE             BUMPY_MODE_ALTERNATING

void  track_bumpy_init(void);
void  track_bumpy_activate(void);
void  track_bumpy_deactivate(void);
bool  track_bumpy_is_active(void);
float track_bumpy_get_speed(void);

/* 增益切换 (模式A为空操作, 模式B降低KP实现柔顺) */
void  track_bumpy_apply_compliance(void);
void  track_bumpy_restore_stiffness(void);

/* 状态机: 每周期调用, 用轮速检测撞条并推进阶段 */
void  track_bumpy_update(const Sensor_data_t *sensor);

/* 当前偏航偏置 (rad), 叠加到 target_direction */
float track_bumpy_get_yaw_bias(void);

/* 当前左/右腿收腿量 (mm), 叠加到足端 Y */
float track_bumpy_get_left_lift(void);
float track_bumpy_get_right_lift(void);

#endif /* TRACK_ELEMENTS_H */
