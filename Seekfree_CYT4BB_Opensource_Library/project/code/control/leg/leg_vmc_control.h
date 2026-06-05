#ifndef LEG_VMC_CONTROL_H
#define LEG_VMC_CONTROL_H

#include "../../app/robot_control/types.h"
#include "kinematics.h"

/**
 * @brief 用 VMC（虚拟模型控制）完成腿部控制
 *
 * 与 PID 方案的差异：
 *  - 在任务空间 (足端 x/y) 做 PD 控制，通过 J^T 映射到关节扭矩
 *  - 无需 Jacobian 逆解算，天然处理 XY 耦合
 *
 * 修复重点：
 *  - 传感器角度 → 绝对物理角度的转换与 PID 方案一致
 *    (左前: π/2 + sensor, 左后: π/2 + (-sensor), 右腿带 RIGHT_ABS_*_SIGN)
 *  - 目标足端 = 标称足端正解 + leg_cmd_solve 输出的偏移量
 *  - VMC 参数通过结构体传入，可在遥控器上调节
 */
void leg_vmc_control(VMC_Config_t *vmc_cfg,
                     const Sensor_data_t *sensor,
                     const Foot_position_t *foot_left_cmd,
                     const Foot_position_t *foot_right_cmd,
                     Motor_cmd_duty_t *motor_cmd);

#endif /* LEG_VMC_CONTROL_H */
