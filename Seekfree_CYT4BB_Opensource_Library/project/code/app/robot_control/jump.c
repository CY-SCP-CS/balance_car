#include "jump.h"
#include "robot_control.h"
#include "../../control/leg/kinematics.h"
#include "../../control/leg/jacobian.h"
#include "../../control/leg/leg_pid_control.h"
#include "../../control/balance/pitch_balance.h"
#include "../../lib/pid/pid_calculate.h"
#include <stdbool.h>

/* =========================== 状态枚举 =========================== */
typedef enum {
    JUMP_IDLE = 0,      /* 空闲 */
    JUMP_PREPARE,       /* 准备：收腿下蹲，达到前进速度 */
    JUMP_TAKEOFF,       /* 起跳：快速伸腿蹬地 */
    JUMP_ASCEND,        /* 上升：空中收腿, 关闭俯仰平衡 */
    JUMP_DESCEND,       /* 下降：伸腿准备接地 */
    JUMP_LAND,          /* 着陆：缓冲并恢复平衡 */
    JUMP_END            /* 结束 */
} JumpState_t;

/* ===================== 可调参数 (1 kHz 周期) ===================== */

/* 各阶段持续时间 (控制周期数, 1 cycle = 1 ms) */
#define PREPARE_CYCLES      400     /* 400 ms 准备下蹲 */
#define TAKEOFF_CYCLES      100     /* 100 ms 蹬地爆发 */
#define ASCEND_CYCLES       200     /* 200 ms 空中收腿 */
#define DESCEND_CYCLES      100     /* 100 ms 伸腿准备接地 */
#define LAND_CYCLES         300     /* 300 ms 缓冲恢复 */

/* 足端位置偏移量 (mm, 相对标称位形)
 *   坐标系: +X 向前, +Y 向下
 *   Y < 0 : 足端上抬 ⇔ 收腿 ⇔ 重心降低 (下蹲 / 空中收腿)
 *   Y > 0 : 足端下压 ⇔ 伸腿 ⇔ 重心升高 (蹬地)
 */
#define OFF_SQUAT_X     0.0f
#define OFF_SQUAT_Y    -30.0f       /* 下蹲收腿 */

#define OFF_PUSH_X      0.0f
#define OFF_PUSH_Y      25.0f       /* 蹬地伸展 */

#define OFF_TUCK_X      0.0f
#define OFF_TUCK_Y     -35.0f       /* 空中紧收 */

#define OFF_REACH_X     5.0f
#define OFF_REACH_Y     15.0f       /* 伸腿找地 (略向前够) */

#define OFF_ABSORB_X    0.0f
#define OFF_ABSORB_Y   -15.0f       /* 着陆缓冲 */

#define TOTAL_JUMPS     3           /* 连续跳跃次数 */

/* ======================== 静态变量 ============================= */

static JumpState_t  g_state         = JUMP_IDLE;    /* 当前状态 */
static uint16_t     g_cycle         = 0;            /* 在当前状态的周期计数 */
static uint8_t      g_jump_num      = 0;            /* 已完成跳跃次数 */

/* 腿部关节 PID (与 robot_control.c 独立) */
static Leg_PID_t    g_leg_left_pid;
static Leg_PID_t    g_leg_right_pid;

/* 目标关节角度 (由足端偏移通过雅各比算出) */
static Leg_Target_t g_target_left;
static Leg_Target_t g_target_right;

/* ======================== 辅助函数 ============================= */

/** 复位俯仰平衡 PID 积分 (跳转至空中/落地时防止饱和) */
static void reset_balance(void) {
    robot_control_reset_balance_pid();
}

/** 执行腿部 PID 控制 (写 g_motor_cmd) */
static void run_leg_pid(const Sensor_data_t *sensor,
                        Motor_cmd_duty_t *motor_cmd)
{
    leg_pid_control(&g_leg_left_pid,  &g_target_left,
                    sensor, LEG_LEFT,  motor_cmd);
    leg_pid_control(&g_leg_right_pid, &g_target_right,
                    sensor, LEG_RIGHT, motor_cmd);
}

/** 俯仰平衡 + 偏航抑制 (地面相位用, 阻尼旋转) */
static void balance_with_yaw(const Sensor_data_t *sensor,
                             Motor_cmd_duty_t *motor_cmd)
{
    pitch_balance_control(sensor,
        &g_speed_pid, &g_pitch_angle_pid, &g_pitch_gyro_pid,
        motor_cmd);

    float yaw_out = pid_calculate(&g_yaw_pid, 0.0f, sensor->gyro_yaw);
    int   yaw_pwm = ROUND(yaw_out);
    motor_cmd->left_motor_pwm  += yaw_pwm;
    motor_cmd->right_motor_pwm += yaw_pwm;
}

/* ====================== 状态机逻辑 ============================= */

static void enter_state(JumpState_t new_state) {
    g_state = new_state;
    g_cycle = 0;
}

static void run_prepare(const Sensor_data_t *sensor,
                        Motor_cmd_duty_t *motor_cmd)
{
    /* 下蹲收腿, 保持俯仰平衡以维持前进速度 */
    Foot_position_t off = { OFF_SQUAT_X, OFF_SQUAT_Y };

    balance_with_yaw(sensor, motor_cmd);

    leg_offset_to_joint_target(LEG_LEFT,  &off, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &off, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= PREPARE_CYCLES) {
        enter_state(JUMP_TAKEOFF);
    }
}

static void run_takeoff(const Sensor_data_t *sensor,
                        Motor_cmd_duty_t *motor_cmd)
{
    /* 快速伸腿蹬地, 俯仰平衡保持车身姿态 */
    Foot_position_t off = { OFF_PUSH_X, OFF_PUSH_Y };

    balance_with_yaw(sensor, motor_cmd);

    leg_offset_to_joint_target(LEG_LEFT,  &off, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &off, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= TAKEOFF_CYCLES) {
        /* 进入空中前复位 PID 积分 */
        reset_balance();
        enter_state(JUMP_ASCEND);
    }
}

static void run_ascend(const Sensor_data_t *sensor,
                       Motor_cmd_duty_t *motor_cmd)
{
    /* 空中收腿: 俯仰平衡输出归零 (轮子无附着力) */
    Foot_position_t off = { OFF_TUCK_X, OFF_TUCK_Y };

    /* 仍调用平衡函数获取最新的 PID 输出, 但最终归零 */
    pitch_balance_control(sensor,
        &g_speed_pid, &g_pitch_angle_pid, &g_pitch_gyro_pid,
        motor_cmd);
    motor_cmd->left_motor_pwm  = 0;
    motor_cmd->right_motor_pwm = 0;

    leg_offset_to_joint_target(LEG_LEFT,  &off, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &off, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= ASCEND_CYCLES) {
        reset_balance();
        enter_state(JUMP_DESCEND);
    }
}

static void run_descend(const Sensor_data_t *sensor,
                        Motor_cmd_duty_t *motor_cmd)
{
    /* 伸腿准备接地, 轮子仍悬空 → 平衡输出归零 */
    Foot_position_t off = { OFF_REACH_X, OFF_REACH_Y };

    pitch_balance_control(sensor,
        &g_speed_pid, &g_pitch_angle_pid, &g_pitch_gyro_pid,
        motor_cmd);
    motor_cmd->left_motor_pwm  = 0;
    motor_cmd->right_motor_pwm = 0;

    leg_offset_to_joint_target(LEG_LEFT,  &off, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &off, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= DESCEND_CYCLES) {
        reset_balance();
        enter_state(JUMP_LAND);
    }
}

static void run_land(const Sensor_data_t *sensor,
                     Motor_cmd_duty_t *motor_cmd)
{
    /*
     * 着陆缓冲: 足端从 "前伸" 线性过渡到 "吸收位"
     * 俯仰平衡全程开启, 落地后立刻恢复平衡
     */
    const float ratio = (float)g_cycle / (float)LAND_CYCLES;
    const float t = (ratio > 1.0f) ? 1.0f : ratio;   /* 归一化时间 [0, 1] */

    Foot_position_t off;
    off.x = OFF_REACH_X + (OFF_ABSORB_X - OFF_REACH_X) * t;
    off.y = OFF_REACH_Y + (OFF_ABSORB_Y - OFF_REACH_Y) * t;

    balance_with_yaw(sensor, motor_cmd);

    leg_offset_to_joint_target(LEG_LEFT,  &off, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &off, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= LAND_CYCLES) {
        g_jump_num++;
        if (g_jump_num < TOTAL_JUMPS) {
            /* 连续跳下一级台阶 */
            enter_state(JUMP_PREPARE);
        } else {
            enter_state(JUMP_END);
        }
    }
}

static void run_end(const Sensor_data_t *sensor,
                    Motor_cmd_duty_t *motor_cmd)
{
    /* 恢复标称位形, 俯仰平衡正常工作 */
    Foot_position_t off = { 0.0f, 0.0f };

    balance_with_yaw(sensor, motor_cmd);

    leg_offset_to_joint_target(LEG_LEFT,  &off, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &off, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    /* 保持 500 ms 后自动切回空闲 */
    if (g_cycle >= 500) {
        enter_state(JUMP_IDLE);
    }
}

/* ======================== 公开接口 ============================= */

void jump_init(void) {
    /* 初始化腿部关节 PID (与 robot_control_init 一致) */
    leg_pid_init(&g_leg_left_pid,  1500.0f, 0.0f, 4.0f, 10000.0f, 0.0f);
    leg_pid_init(&g_leg_right_pid, 1500.0f, 0.0f, 4.0f, 10000.0f, 0.0f);

    g_state    = JUMP_IDLE;
    g_cycle    = 0;
    g_jump_num = 0;

    /* 标称腿位形 */
    g_target_left.front  = LEG_NOM_FRONT_L;
    g_target_left.back   = LEG_NOM_BACK_L;
    g_target_right.front = LEG_NOM_FRONT_R;
    g_target_right.back  = LEG_NOM_BACK_R;
}

void jump_start(void) {
    if (g_state == JUMP_IDLE) {
        g_jump_num = 0;
        enter_state(JUMP_PREPARE);
    }
}

void jump_stop(void) {
    /* 立即停止跳跃, 进入结束 */
    if (g_state != JUMP_IDLE && g_state != JUMP_END) {
        reset_balance();
        enter_state(JUMP_END);
    }
}

bool jump_is_active(void) {
    return (g_state != JUMP_IDLE);
}

bool jump_is_done(void) {
    return (g_state == JUMP_END) || (g_state == JUMP_IDLE);
}

void jump_control(const Sensor_data_t *sensor, Motor_cmd_duty_t *motor_cmd) {
    g_cycle++;

    switch (g_state) {
    case JUMP_PREPARE:
        run_prepare(sensor, motor_cmd);
        break;

    case JUMP_TAKEOFF:
        run_takeoff(sensor, motor_cmd);
        break;

    case JUMP_ASCEND:
        run_ascend(sensor, motor_cmd);
        break;

    case JUMP_DESCEND:
        run_descend(sensor, motor_cmd);
        break;

    case JUMP_LAND:
        run_land(sensor, motor_cmd);
        break;

    case JUMP_END:
        run_end(sensor, motor_cmd);
        break;

    case JUMP_IDLE:
    default:
        /* control_task 只在 jump_is_active() 时调用 jump_control(),
         * IDLE 时走正常的 pitch_balance_control + leg_cmd_solve 路径 */
        break;
    }
}
