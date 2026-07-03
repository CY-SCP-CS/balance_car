#ifndef JUMP_H
#define JUMP_H

#include "../../common/types.h"
#include "types.h"
#include <stdbool.h>

/* ─── 初始化 ───
 *  在 robot_control_init() 中调用一次, 初始化腿 PID 并置为空闲状态。
 */
void jump_init(void);

/* ─── 启动跳跃序列(连续跳 3 下) ───
 *  仅在 JUMP_IDLE 时有效, 触发后状态机从 JUMP_PREPARE 开始运行。
 */
void jump_start(void);

/* ─── 紧急中止 ───
 *  任何非空闲/结束状态下调⽤, 复位平衡 PID 后直接进⼊ JUMP_END。
 */
void jump_stop(void);

/* ─── 查询 ─── */
bool jump_is_active(void);   /* 跳跃序列是否正在运⾏ */
bool jump_is_done(void);     /* 跳跃是否已全部结束 */

/* ─── 每周期调⽤ (放⼊ control_task) ───
 *  sensor   - 当前传感器数据
 *  motor_cmd- 电机指令 (跳转过程中会写⼊腿 PWM 和轮 PWM)
 *
 *  jump_is_active() == true 时应在 control_task 中调⽤此函数
 *  代替常规的 pitch_balance_control + leg_cmd_solve + leg_pid_control。
 */
void jump_control(const Sensor_data_t *sensor, Motor_cmd_duty_t *motor_cmd);

#endif /* JUMP_H */
