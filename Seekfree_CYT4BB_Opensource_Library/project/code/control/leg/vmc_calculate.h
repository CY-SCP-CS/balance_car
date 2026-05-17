#ifndef VMC_CALCULATE_H
#define VMC_CALCULATE_H

#include "kinematics.h"
#include "jacobian.h"
#include "../../app/robot_control/types.h"

void vmc_calculate(const VMC_Config_t *vmc_cfg,
                                 const Foot_position_t *foot_tar,
                                 const Joint_angle_t *angles_cur,
                                 float joint_front_speed,
                                 float joint_back_speed,
                                 LegSide_t leg_side,
                                 Motor_cmd_duty_t *motor_cmd);

#endif /* VMC_CALCULATE_H */