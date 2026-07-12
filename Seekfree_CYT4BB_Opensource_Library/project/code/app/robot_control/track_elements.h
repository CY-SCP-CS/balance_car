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
void  track_rotate720_reset(void);
void  track_rotate720_update(Sensor_data_t *sensor, Move_cmd_t *cmd);

/* ===== 2. 单边桥与爬坡 (固定归一化速度) ===== */
void  track_bridge_climb_init(void);
void  track_bridge_climb_activate(void);
void  track_bridge_climb_deactivate(void);
bool  track_bridge_climb_is_active(void);
void  track_bridge_climb_apply(Move_cmd_t *cmd);

/* ===== 3. 颠簸路段 ===== */
void  track_bumpy_init(void);
void  track_bumpy_activate(void);
void  track_bumpy_deactivate(void);
bool  track_bumpy_is_active(void);
float track_bumpy_get_clearance_offset(void);
void  track_bumpy_apply_leg_gains(Leg_PID_t *left, Leg_PID_t *right);
void  track_bumpy_restore_leg_gains(Leg_PID_t *left, Leg_PID_t *right);

#endif /* TRACK_ELEMENTS_H */
