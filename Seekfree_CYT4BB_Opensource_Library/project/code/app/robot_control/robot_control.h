#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include "../../common/types.h"
#include "../../control/leg/leg_cmd_solve.h"
#include "../../control/leg/vmc_calculate.h"
#include "../../control/balance/pitch_balance.h"

#define DEG_TO_RAD (3.1415926f / 180.0f)

extern Motor_cmd_duty_t g_motor_cmd;

extern Sensor_data_t g_sensor_data;

extern Move_cmd_t g_move_cmd;

void robot_control_init(void);

void sensor_update(const Sensor_data_t *sensor);

void command_update(const Move_cmd_t *cmd);

void control_task(void);

void sensor_cmd_update(const Ctrl_Input_t *ctrl, Sensor_data_t *sensor, Move_cmd_t *cmd);

#endif /* ROBOT_CONTROL_H */