#ifndef LEG_CMD_SOLVE_H
#define LEG_CMD_SOLVE_H

#include "../../app/robot_control/types.h"

void leg_cmd_solve(const Move_cmd_t *move_cmd, Foot_position_t *foot_position_left,
    Foot_position_t *foot_position_right);

#endif /* LEG_CMD_SOLVE_H */