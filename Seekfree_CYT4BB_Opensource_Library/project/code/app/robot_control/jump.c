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
    JUMP_BALANCE,       /* 预平衡：轮子平衡, 腿保持标称 */
    JUMP_PREPARE,       /* 准备：收腿下蹲 */
    JUMP_TAKEOFF,       /* 起跳：快速伸腿蹬地, 平衡线性衰减 */
    JUMP_ASCEND,        /* 上升：空中收腿, 平衡归零 */
    JUMP_DESCEND,       /* 下降：伸腿准备接地, 平衡归零 */
    JUMP_IMPACT,        /* 触地冲击：深收腿吸收冲击, 平衡全开 */
    JUMP_LAND,          /* 着陆：缓冲并恢复平衡 */
    JUMP_INTERVAL,      /* 跳跃间隔：正常平衡 10 秒 */
    JUMP_END            /* 结束 */
} JumpState_t;

/* ===================== 可调参数 (1 kHz 周期) ===================== */

/* 各阶段持续时间 (控制周期数, 1 cycle = 1 ms) */
#define BALANCE_CYCLES       500     /* 500 ms 预平衡 */
#define PREPARE_CYCLES       500     /* 500 ms 准备下蹲 */
#define TAKEOFF_CYCLES       150     /* 150 ms: 前100ms全推 + 后50ms过渡+离地检测 */
#define ASCEND_CYCLES        30      /* 30 ms 空中收腿 */
#define DESCEND_CYCLES       30      /* 30 ms: 预收→保持→IMPACT */
#define IMPACT_CYCLES        200     /* 200 ms 触地缓冲 */
#define INTERVAL_CYCLES      10000   /* 10 s 跳跃间隔 */
#define LAND_CYCLES          400     /* 400 ms 缓冲恢复 */

/* DESCEND: 纯时序, 伸腿已在 ASCEND 末尾完成, 这里只管预收 */
#define DESCEND_EXTEND_MS     5      /* 最后一点伸腿 */
#define DESCEND_RETRACT_END   15     /* 预收完成 */
#define ACCEL_FREEFALL_THRESHOLD  0.3f  /* accel_z 自由落体阈值, <此值判为离地 */
#define ROLL_LEVEL_GAIN           80.0f /* 额外 roll 修正增益 (mm/rad), 双腿不等高时加大 */

/* 足端位置偏移量 (mm, 相对标称位形)
 *   实际坐标: Y<0 → 伸腿(重心升), Y>0 → 收腿(重心降)
 */
#define OFF_PUSH_X      0.0f
#define OFF_PUSH_Y      -400.0f      /* 蹬地伸展 (故意超出关节极限, 保持 PID 饱和输出) */

#define OFF_SQUAT_X     0.0f
#define OFF_SQUAT_Y     120.0f       /* 下蹲收腿 */

#define OFF_TUCK_X      0.0f
#define OFF_TUCK_Y      85.0f        /* 空中紧收 */

#define OFF_REACH_X     0.0f
#define OFF_REACH_Y     -90.0f       /* 伸腿找地 */

#define OFF_PRE_X       0.0f
#define OFF_PRE_Y       30.0f        /* 预收腿位 (着陆前提前收, 准备缓冲) */

#define OFF_IMPACT_BASE 30.0f       /* 基础收腿深度 (mm) */
#define OFF_IMPACT_MAX  100.0f      /* 最大收腿深度 (mm) */
#define IMPACT_K_GYRO   25.0f       /* gyro 峰值 → 收腿深度的增益 */

#define LAND_FORWARD_PWM 300        /* 落地前倾补偿 (PWM), 抵消重心偏后 */

#define IMPACT_GYRO_DELTA_THRESHOLD  2.0f  /* 陀螺仪触地检测阈值 (rad/s) */

#define TOTAL_JUMPS     3           /* 连续跳跃次数 */

/* ======================== 静态变量 ============================= */

static JumpState_t  g_state         = JUMP_IDLE;    /* 当前状态 */
static uint16_t     g_cycle         = 0;            /* 在当前状态的周期计数 */
static uint8_t      g_jump_num      = 0;            /* 已完成跳跃次数 */
static float        g_jump_target_speed = 0.0f;     /* 预留前进速度 (mm/s) */
static float        g_impact_retract_y = 0.0f;      /* IMPACT 阶段计算出的收腿深度 */
static float        g_gyro_peak_carry  = 0.0f;      /* DESCEND→IMPACT 传递 gyro 峰值 */

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

/** 俯仰平衡 + 偏航差速抑制, 带线性权重
 *  weight = 1.0 全输出, 0.0 零输出, 中间线性插值
 */
static void balance_with_yaw_scaled(const Sensor_data_t *sensor,
                                    Motor_cmd_duty_t *motor_cmd,
                                    float weight)
{
    if (weight <= 0.0f) {
        motor_cmd->left_motor_pwm  = 0;
        motor_cmd->right_motor_pwm = 0;
        return;
    }

    pitch_balance_control(sensor,
        &g_speed_pid, &g_pitch_angle_pid, &g_pitch_gyro_pid,
        motor_cmd);

    /* 差速转向: 与 control_task 保持一致 */
    float yaw_out = pid_calculate(&g_yaw_pid, 0.0f, -sensor->gyro_yaw);
    int   yaw_pwm = ROUND(yaw_out);
    motor_cmd->left_motor_pwm  += yaw_pwm;
    motor_cmd->right_motor_pwm -= yaw_pwm;

    if (weight < 1.0f) {
        motor_cmd->left_motor_pwm  = (int)(motor_cmd->left_motor_pwm  * weight);
        motor_cmd->right_motor_pwm = (int)(motor_cmd->right_motor_pwm * weight);
    }
}

/* detect_impact 状态 (模块级, 支持跨状态复位) */
static float g_impact_prev_gyro = 0.0f;
static bool  g_impact_first_call = true;

static void detect_impact_reset(void) {
    g_impact_first_call = true;
}

/** 陀螺仪触地检测: 检测俯仰角速度突变 */
static bool detect_impact(const Sensor_data_t *sensor) {
    if (g_impact_first_call) {
        g_impact_prev_gyro = sensor->gyro_pitch;
        g_impact_first_call = false;
        return false;
    }
    float delta = sensor->gyro_pitch - g_impact_prev_gyro;
    g_impact_prev_gyro = sensor->gyro_pitch;
    return (fabsf(delta) > IMPACT_GYRO_DELTA_THRESHOLD);
}

/* ====================== 状态机逻辑 ============================= */

static void enter_state(JumpState_t new_state) {
    g_state = new_state;
    g_cycle = 0;
}

static void run_balance(const Sensor_data_t *sensor,
                        Motor_cmd_duty_t *motor_cmd)
{
    /* 预平衡: 轮子平衡, 腿保持标称 + 速度环 */
    Foot_position_t left  = { 0.0f, 0.0f };
    Foot_position_t right = { 0.0f, 0.0f };

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

    robot_control_leg_speed_feedback(sensor, &left, &right);
    /* 额外 roll 修正: 身体往哪边歪, 那边的腿就多伸一点 */
    {
        float roll_corr = sensor->angle_roll * ROLL_LEVEL_GAIN;
        left.y  += roll_corr;
        right.y -= roll_corr;
    }
    leg_offset_to_joint_target(LEG_LEFT,  &left,  &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &right, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= BALANCE_CYCLES) {
        reset_balance();
        enter_state(JUMP_PREPARE);
    }
}

static void run_prepare(const Sensor_data_t *sensor,
                        Motor_cmd_duty_t *motor_cmd)
{
    /* 下蹲收腿, 保持平衡, 双腿对称 */
    Foot_position_t left  = { OFF_SQUAT_X, OFF_SQUAT_Y };
    Foot_position_t right = { OFF_SQUAT_X, OFF_SQUAT_Y };

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

    leg_offset_to_joint_target(LEG_LEFT,  &left,  &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &right, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= PREPARE_CYCLES) {
        enter_state(JUMP_TAKEOFF);
    }
}

static void run_takeoff(const Sensor_data_t *sensor,
                        Motor_cmd_duty_t *motor_cmd)
{
    /* 快速伸腿蹬地, 双腿对称推地, 保持平衡 (轮子着地)
     *   前 100ms: 全力推地 (OFF_PUSH_Y = -400)
     *   后  30ms: 线性过渡到收腿 (OFF_TUCK_Y = 85), 避免切 ASCEND 时目标跳变
     */
    Foot_position_t left, right;
    left.x  = OFF_PUSH_X;
    right.x = OFF_PUSH_X;

    if (g_cycle <= 100) {
        left.y  = OFF_PUSH_Y;
        right.y = OFF_PUSH_Y;
    } else {
        float blend_t = (float)(g_cycle - 100) / 30.0f;
        if (blend_t > 1.0f) blend_t = 1.0f;
        float blended_y = OFF_PUSH_Y + (OFF_TUCK_Y - OFF_PUSH_Y) * blend_t;
        left.y  = blended_y;
        right.y = blended_y;
    }

    if (g_cycle == 1) {
        g_leg_left_pid.front.kp  = 10000.0f;
        g_leg_left_pid.front.ki  = 50.0f;
        g_leg_left_pid.back.kp   = 10000.0f;
        g_leg_left_pid.back.ki   = 50.0f;
        g_leg_right_pid.front.kp = 10000.0f;
        g_leg_right_pid.front.ki = 50.0f;
        g_leg_right_pid.back.kp  = 10000.0f;
        g_leg_right_pid.back.ki  = 50.0f;
        pid_reset(&g_leg_left_pid.front);
        pid_reset(&g_leg_left_pid.back);
        pid_reset(&g_leg_right_pid.front);
        pid_reset(&g_leg_right_pid.back);
    }

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

    leg_offset_to_joint_target(LEG_LEFT,  &left,  &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &right, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    /* 离地检测: 100ms 后用 accel_z 判自由落体 */
    bool airborne = false;
    if (g_cycle > 100) {
        airborne = (fabsf(sensor->accel_z) < ACCEL_FREEFALL_THRESHOLD);
    }

    if (airborne || g_cycle >= TAKEOFF_CYCLES) {
        /* 恢复正常 kp/ki, 进入空中前复位平衡 PID */
        g_leg_left_pid.front.kp  = 1200.0f;
        g_leg_left_pid.front.ki  = 0.0f;
        g_leg_left_pid.back.kp   = 1200.0f;
        g_leg_left_pid.back.ki   = 0.0f;
        g_leg_right_pid.front.kp = 1200.0f;
        g_leg_right_pid.front.ki = 0.0f;
        g_leg_right_pid.back.kp  = 1200.0f;
        g_leg_right_pid.back.ki  = 0.0f;
        reset_balance();
        enter_state(JUMP_ASCEND);
    }
}

static void run_ascend(const Sensor_data_t *sensor,
                       Motor_cmd_duty_t *motor_cmd)
{
    /* 离地后: 先收腿减少惯量, 再开始伸腿够地
     *   [0~15ms]: 收腿 tuck, 平衡衰减 1.0→0.0
     *   [15~30ms]: 伸腿 (tuck → reach), 平衡=0, DESCEND 前腿已伸出
     */
    Foot_position_t off;
    off.x = OFF_TUCK_X;

    if (g_cycle <= 15) {
        off.y = OFF_TUCK_Y;
    } else {
        float t = (float)(g_cycle - 15) / 15.0f;
        if (t > 1.0f) t = 1.0f;
        off.y = OFF_TUCK_Y + (OFF_REACH_Y - OFF_TUCK_Y) * t;
    }

    float weight;
    if (g_cycle <= 15) {
        weight = 1.0f - (float)g_cycle / 15.0f;
    } else {
        weight = 0.0f;
    }
    balance_with_yaw_scaled(sensor, motor_cmd, weight);

    leg_offset_to_joint_target(LEG_LEFT,  &off, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &off, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= ASCEND_CYCLES) {
        reset_balance();
        detect_impact_reset();
        enter_state(JUMP_DESCEND);
    }
}

static void run_descend(const Sensor_data_t *sensor,
                        Motor_cmd_duty_t *motor_cmd)
{
    /* 下落三阶段 (从最高点到落地约 60ms):
     *   [0~15ms]   宽限期: 快速伸腿, 不检测触地
     *   [0~20ms]   伸腿到底 (OFF_REACH_Y = -90)
     *   [20~55ms]  逐步预收 (-90 → +30), 着陆前腿已在缓冲姿态
     *   [55~70ms]  保持预收位, 等触地
     */
    Foot_position_t off;
    off.x = OFF_REACH_X;

    if (g_cycle == 1) {
        g_gyro_peak_carry = 0.0f;
        /* 快速伸腿: 提高 kp 确保在 20ms 内完成 */
        g_leg_left_pid.front.kp  = 10000.0f;
        g_leg_left_pid.back.kp   = 10000.0f;
        g_leg_right_pid.front.kp = 10000.0f;
        g_leg_right_pid.back.kp  = 10000.0f;
        pid_reset(&g_leg_left_pid.front);
        pid_reset(&g_leg_left_pid.back);
        pid_reset(&g_leg_right_pid.front);
        pid_reset(&g_leg_right_pid.back);
    }

    /* 持续追踪 gyro 峰值, 传递给 IMPACT 阶段 */
    float abs_gyro = fabsf(sensor->gyro_pitch);
    if (abs_gyro > g_gyro_peak_carry) g_gyro_peak_carry = abs_gyro;

    /* 纯时序: 伸腿→预收→保持 → IMPACT */
    if (g_cycle <= DESCEND_EXTEND_MS) {
        off.y = OFF_REACH_Y;
    }
    else if (g_cycle <= DESCEND_RETRACT_END) {
        float t = (float)(g_cycle - DESCEND_EXTEND_MS)
                / (float)(DESCEND_RETRACT_END - DESCEND_EXTEND_MS);
        off.y = OFF_REACH_Y + (OFF_PRE_Y - OFF_REACH_Y) * t;
    }
    else {
        off.y = OFF_PRE_Y;
    }

    balance_with_yaw_scaled(sensor, motor_cmd, 0.0f);

    leg_offset_to_joint_target(LEG_LEFT,  &off, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &off, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= DESCEND_CYCLES) {
        enter_state(JUMP_IMPACT);
    }
}

static void run_impact(const Sensor_data_t *sensor,
                       Motor_cmd_duty_t *motor_cmd)
{
    /* 触地缓冲: 立即收腿, 根据实时 gyro 连续调节深度 (主动阻尼) */
    static float gyro_peak = 0.0f;

    if (g_cycle == 1) {
        gyro_peak = g_gyro_peak_carry;
        g_gyro_peak_carry = 0.0f;

        /* 用 gyro 峰值计算初始收腿深度 */
        float retract_y = OFF_IMPACT_BASE + IMPACT_K_GYRO * gyro_peak;
        if (retract_y > OFF_IMPACT_MAX) retract_y = OFF_IMPACT_MAX;
        if (retract_y < OFF_IMPACT_BASE) retract_y = OFF_IMPACT_BASE;
        g_impact_retract_y = retract_y;

        /* 提高腿 PID 刚度, 快速响应冲击 */
        g_leg_left_pid.front.kp  = 10000.0f;
        g_leg_left_pid.back.kp   = 10000.0f;
        g_leg_right_pid.front.kp = 10000.0f;
        g_leg_right_pid.back.kp  = 10000.0f;
        reset_balance();
        robot_control_reset_leg_speed_pid();
        pid_reset(&g_yaw_pid);
        pid_reset(&g_yaw_angle_pid);
        pid_reset(&g_leg_left_pid.front);
        pid_reset(&g_leg_left_pid.back);
        pid_reset(&g_leg_right_pid.front);
        pid_reset(&g_leg_right_pid.back);
    }

    /* 持续追踪 gyro 峰值 */
    float abs_gyro = fabsf(sensor->gyro_pitch);
    if (abs_gyro > gyro_peak) gyro_peak = abs_gyro;

    /* 每周期根据实时 gyro 调整收腿深度 (主动阻尼)
     * 冲击大 → gyro 大 → 收腿深 → 吸收能量
     * 冲击消退 → gyro 小 → 收腿浅 → 恢复站立 */
    float live_retract = OFF_IMPACT_BASE + IMPACT_K_GYRO * abs_gyro;
    if (live_retract > OFF_IMPACT_MAX) live_retract = OFF_IMPACT_MAX;
    if (live_retract < OFF_IMPACT_BASE) live_retract = OFF_IMPACT_BASE;

    /* 低通滤波平滑, 避免收腿深度突变 */
    g_impact_retract_y += 0.5f * (live_retract - g_impact_retract_y);

    Foot_position_t left  = { 0.0f, g_impact_retract_y };
    Foot_position_t right = { 0.0f, g_impact_retract_y };

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);
    /* 落地瞬间前倾补偿: 平衡积分已被复位, PWM 偏置抵消重心偏后 */
    motor_cmd->left_motor_pwm  += LAND_FORWARD_PWM;
    motor_cmd->right_motor_pwm += LAND_FORWARD_PWM;

    leg_offset_to_joint_target(LEG_LEFT,  &left,  &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &right, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= IMPACT_CYCLES) {
        /* 恢复正常 kp */
        g_leg_left_pid.front.kp  = 1200.0f;
        g_leg_left_pid.back.kp   = 1200.0f;
        g_leg_right_pid.front.kp = 1200.0f;
        g_leg_right_pid.back.kp  = 1200.0f;
        enter_state(JUMP_LAND);
    }
}

static void run_land(const Sensor_data_t *sensor,
                     Motor_cmd_duty_t *motor_cmd)
{
    /*
     * 着陆恢复: 足端从收腿位线性过渡到标称位(0,0)
     * 平衡全开
     */
    const float ratio = (float)g_cycle / (float)LAND_CYCLES;
    const float t = (ratio > 1.0f) ? 1.0f : ratio;
    float x = 0.0f;
    float y = g_impact_retract_y * (1.0f - t);

    Foot_position_t left  = { x, y };
    Foot_position_t right = { x, y };

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

    /* 前倾补偿随 LAND 进度线性衰减: 平衡 PID 积分逐渐积累, 偏置逐步退出 */
    int forward = (int)(LAND_FORWARD_PWM * (1.0f - t));
    motor_cmd->left_motor_pwm  += forward;
    motor_cmd->right_motor_pwm += forward;

    robot_control_leg_speed_feedback(sensor, &left, &right);
    /* 额外 roll 修正 */
    {
        float roll_corr = sensor->angle_roll * ROLL_LEVEL_GAIN;
        left.y  += roll_corr;
        right.y -= roll_corr;
    }
    leg_offset_to_joint_target(LEG_LEFT,  &left,  &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &right, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= LAND_CYCLES) {
        g_jump_num++;
        if (g_jump_num < TOTAL_JUMPS) {
            enter_state(JUMP_INTERVAL);
        } else {
            enter_state(JUMP_END);
        }
    }
}

static void run_interval(const Sensor_data_t *sensor,
                         Motor_cmd_duty_t *motor_cmd)
{
    /* 跳跃间隔: 正常平衡站立 10 秒 */
    Foot_position_t left  = { 0.0f, 0.0f };
    Foot_position_t right = { 0.0f, 0.0f };

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

    robot_control_leg_speed_feedback(sensor, &left, &right);
    /* 额外 roll 修正 */
    {
        float roll_corr = sensor->angle_roll * ROLL_LEVEL_GAIN;
        left.y  += roll_corr;
        right.y -= roll_corr;
    }
    leg_offset_to_joint_target(LEG_LEFT,  &left,  &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &right, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= INTERVAL_CYCLES) {
        reset_balance();
        enter_state(JUMP_BALANCE);
    }
}

static void run_end(const Sensor_data_t *sensor,
                    Motor_cmd_duty_t *motor_cmd)
{
    /* 恢复标称位形, 俯仰平衡正常工作 */
    Foot_position_t off = { 0.0f, 0.0f };

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

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
    /* 腿 PID 参数与正常控制一致, 推地/收腿时临时拉高 kp */
    leg_pid_init(&g_leg_left_pid,  1200.0f, 0.0f, 4.0f, 10000.0f, 0.0f);
    leg_pid_init(&g_leg_right_pid, 1200.0f, 0.0f, 4.0f, 10000.0f, 0.0f);

    g_state    = JUMP_IDLE;
    g_cycle    = 0;
    g_jump_num = 0;

    /* 标称腿位形 */
    g_target_left.front  = LEG_NOM_FRONT_L;
    g_target_left.back   = LEG_NOM_BACK_L;
    g_target_right.front = LEG_NOM_FRONT_R;
    g_target_right.back  = LEG_NOM_BACK_R;
}

void jump_start(float target_speed) {
    if (g_state == JUMP_IDLE) {
        g_jump_num = 0;
        g_jump_target_speed = target_speed;
        /* 复位腿 PID, 从零开始跟踪 */
        pid_reset(&g_leg_left_pid.front);
        pid_reset(&g_leg_left_pid.back);
        pid_reset(&g_leg_right_pid.front);
        pid_reset(&g_leg_right_pid.back);
        enter_state(JUMP_BALANCE);
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
    case JUMP_BALANCE:
        run_balance(sensor, motor_cmd);
        break;

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

    case JUMP_IMPACT:
        run_impact(sensor, motor_cmd);
        break;

    case JUMP_LAND:
        run_land(sensor, motor_cmd);
        break;

    case JUMP_INTERVAL:
        run_interval(sensor, motor_cmd);
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
