#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include "../../app/robot_control/types.h"

/** 髋关节电机角度 (弧度) */
typedef struct
{
    float left_motor_angle;  /**< 左电机角度 (rad), 从 +X 起算 */
    float right_motor_angle; /**< 右电机角度 (rad), 从 +X 起算 */
} Joint_angle_t;

int five_bar_forward(const Joint_angle_t *angles,
                    Foot_position_t *foot_position);

int five_bar_inverse(const Foot_position_t *foot_position,
                    Joint_angle_t *angles);

#endif /* KINEMATICS_H */
