#include "leg_cmd_solve.h"
#include <math.h>

/* ─── 足端最大偏移量 ─── */
#define FOOT_X_OFFSET_MAX      30.0f   /* 速度闭环最大 x 偏移 (mm) */
#define FOOT_ROLL_OFFSET_MAX   20.0f   /* 横滚闭环最大差动 y 偏移 (mm) */
#define HEIGHT_OFFSET_MAX      30.0f   /* 高度指令最大 y 偏移 (mm) */

/* ─── 反馈归一化基准 ─── */
#define SPEED_MAX_RADPS        60.0f   /* 速度归一化基准 (rad/s) */
#define ROLL_MAX_RAD            0.35f  /* 横滚角归一化基准 (rad) ≈ 20° */

#define CLAMP(val, lo, hi)  fminf(fmaxf(val, lo), hi)

void leg_cmd_solve(const Move_cmd_t *move_cmd,
    const Sensor_data_t *sensor,
    PID_Controller_t *pid_leg_speed,
    PID_Controller_t *pid_leg_roll,
    Foot_position_t *foot_position_left,
    Foot_position_t *foot_position_right){

    /* ============================================================
     *  1. 速度闭环 → x 偏移（双脚同向）
     *     target_speed ∈ [-1, +1]（归一化）
     *     反馈：左右轮平均速度 (rad/s) 归一化到 [-1, +1]
     *     PID 输出直接作为 x 偏移量 (mm)
     * ============================================================ */
    float speed_cur   = (sensor->motor_left_speed + sensor->motor_right_speed) / 2.0f;
    float speed_norm  = speed_cur / SPEED_MAX_RADPS;
    float x_offset    = pid_calculate(pid_leg_speed, move_cmd->target_speed, speed_norm);
    x_offset          = CLAMP(x_offset, -FOOT_X_OFFSET_MAX, FOOT_X_OFFSET_MAX);

    foot_position_left->x  = x_offset;
    foot_position_right->x = x_offset;

    /* ============================================================
     *  2. 横滚闭环 → y 差动偏移
     *     target_roll ∈ [-1, +1]（归一化）
     *     反馈：angle_roll (rad) 归一化到 [-1, +1]
     *     PID 输出 ∈ [-1, +1]，再映射到 ±FOOT_ROLL_OFFSET_MAX
     * ============================================================ */
    float roll_norm   = sensor->angle_roll / ROLL_MAX_RAD;
    float roll_output = pid_calculate(pid_leg_roll, move_cmd->target_roll, roll_norm);
    roll_output       = CLAMP(roll_output, -1.0f, 1.0f);

    foot_position_left->y  =  roll_output * FOOT_ROLL_OFFSET_MAX;
    foot_position_right->y = -roll_output * FOOT_ROLL_OFFSET_MAX;

    /* ============================================================
     *  3. 高度控制（双脚同向）
     *     target_height ∈ [-1, +1]
     *     y 正方向为腿部伸展方向（足端远离车身）
     * ============================================================ */
    float height_offset = move_cmd->target_height * HEIGHT_OFFSET_MAX;
    foot_position_left->y  += height_offset;
    foot_position_right->y += height_offset;
}
