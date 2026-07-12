#ifndef JUMP_H
#define JUMP_H

#include "../../common/types.h"
#include "types.h"
#include <stdbool.h>

/* ─── 初始化 ───
 *  在 robot_control_init() 中调用一次, 置为空闲状态。
 */
void jump_init(void);

/* ─── 启动跳跃 ───
 *  仅在 JUMP_IDLE 时有效, 触发后状态机从 SQUAT 开始运行。
 *  target_speed - 预留前进速度 (mm/s), 暂不生效
 */
void jump_start(float target_speed);

/* ─── 紧急中止 ───
 *  任何非空闲状态下调用, 恢复腿 PID 默认增益后进入 IDLE。
 */
void jump_stop(void);

/* ─── 查询 ─── */
bool jump_is_active(void);    /* 跳跃状态机是否正在运行 */

/* ─── 是否处于空中阶段 ───
 *  AIR_ASCEND / AIR_DESCEND 返回 true, 其余返回 false。
 *  调用者据此关停轮式平衡和 leg_cmd_solve。
 */
bool jump_is_airborne(void);

/* ─── 是否正在蹬地阶段 ───
 *  JUMP_LAUNCH 返回 true, 其余返回 false.
 *  调用者可在 LAUNCH 期间设置前向目标速度, 帮助向前位移.
 */
bool jump_is_launching(void);

/* ─── 是否在起跳前自稳阶段 ── */
bool jump_is_stabilizing(void);

/* ─── 是否在下蹲蓄力阶段 ── */
bool jump_is_squatting(void);

/* ─── 获取起跳前的前向接近速度 ───
 *  返回 jump_start() 传入的 target_speed, 调用者在 STABILIZE 阶段施加.
 */
float jump_get_approach_speed(void);

/* ─── 跳跃腿轨迹叠加 (每周期调用, 放入 control_task) ───
 *  在 leg_cmd_solve 之后调用, 叠加跳跃 Y 轨迹和 X 偏置到足端位置。
 *  不写电机 PWM, 只修改足端位置。
 *
 *  left/right - 当前足端位置 (leg_cmd_solve 的输出或空中阶段的 {0,0})
 *  sensor     - 当前传感器数据 (用于 accel_z 离地/触地检测)
 */
void jump_leg_overlay(Foot_position_t *left, Foot_position_t *right,
                      const Sensor_data_t *sensor);

#endif /* JUMP_H */
