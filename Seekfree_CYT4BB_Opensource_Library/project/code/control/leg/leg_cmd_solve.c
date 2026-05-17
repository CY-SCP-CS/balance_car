#include "leg_cmd_solve.h"
#include <math.h>

/* 足端最大 x 偏移量（mm） */
#define FOOT_X_OFFSET_MAX   30.0f

#define CLAMP(val, lo, hi)  fminf(fmaxf(val, lo), hi)

void leg_cmd_solve(const Move_cmd_t *move_cmd, Foot_position_t *foot_position_left,
    Foot_position_t *foot_position_right){

    float speed  = CLAMP(move_cmd->target_speed,  -1.0f, 1.0f);
    float height = CLAMP(move_cmd->target_height, -1.0f, 1.0f);
    float roll   = CLAMP(move_cmd->target_roll,   -1.0f, 1.0f);

    foot_position_left->x  = speed * FOOT_X_OFFSET_MAX;
    foot_position_right->x = speed * FOOT_X_OFFSET_MAX;

    foot_position_left->y  = (height + roll) * LEG_LENGTH_STANDARD;
    foot_position_right->y = (height - roll) * LEG_LENGTH_STANDARD;
}