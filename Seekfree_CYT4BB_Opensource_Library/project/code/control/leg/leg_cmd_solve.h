#ifndef LEG_CMD_SOLVE_H
#define LEG_CMD_SOLVE_H

#include "../../app/robot_control/types.h"
#include "../../lib/pid/pid_calculate.h"

void leg_cmd_solve(const Move_cmd_t *move_cmd,
    const Sensor_data_t *sensor,
    PID_Controller_t *pid_leg_speed,
    PID_Controller_t *pid_leg_roll,
    Foot_position_t *foot_position_left,
    Foot_position_t *foot_position_right);

#endif /* LEG_CMD_SOLVE_H */