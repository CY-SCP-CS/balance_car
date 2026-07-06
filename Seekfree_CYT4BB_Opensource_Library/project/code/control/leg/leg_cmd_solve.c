#include "leg_cmd_solve.h"
#include <math.h>
#include <stdint.h>

/* ─── 足端最大偏移量 ─── */
#define FOOT_X_OFFSET_MAX      90.0f   /* 速度闭环最大 x 偏移 (mm) */
#define FOOT_ROLL_OFFSET_MAX   180.0f   /* 横滚闭环最大差动 y 偏移 (mm) */
#define HEIGHT_OFFSET_MAX      30.0f   /* 高度指令最大 y 偏移 (mm) */

/* ─── 反馈归一化基准 ─── */
#define SPEED_MAX_RADPS        60.0f   /* 速度归一化基准 (rad/s) */
#define ROLL_MAX_RAD            0.8f  //最大roll补偿


void leg_cmd_solve(const Move_cmd_t *move_cmd,
    const Sensor_data_t *sensor,
    PID_Controller_t *pid_leg_speed,
    PID_Controller_t *pid_leg_roll,
    Foot_position_t *foot_position_left,
    Foot_position_t *foot_position_right){

    float speed_cur   = (sensor->motor_left_speed + sensor->motor_right_speed) / 2.0f;//平均速度
    float speed_norm  = speed_cur / SPEED_MAX_RADPS;
    float x_target    = pid_calculate(pid_leg_speed, move_cmd->target_speed, speed_norm);
    x_target          = CLAMP(x_target, -FOOT_X_OFFSET_MAX, FOOT_X_OFFSET_MAX);

    foot_position_left->x  = x_target;
    foot_position_right->x = -x_target;

    //横滚平衡: roll PID 调整双腿高度差，保持车身水平
    float roll_norm   = sensor->angle_roll / ROLL_MAX_RAD;
    float roll_output = pid_calculate(pid_leg_roll, move_cmd->target_roll, roll_norm);
    roll_output       = CLAMP(roll_output, -1.0f, 1.0f);

    foot_position_left->y  =  roll_output * FOOT_ROLL_OFFSET_MAX;
    foot_position_right->y = -roll_output * FOOT_ROLL_OFFSET_MAX;

    float height_offset = move_cmd->target_height * HEIGHT_OFFSET_MAX;
    foot_position_left->y  += height_offset;
    foot_position_right->y += height_offset;
}
