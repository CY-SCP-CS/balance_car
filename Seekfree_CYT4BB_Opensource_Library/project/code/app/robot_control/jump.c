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
    JUMP_IDLE = 0,
    JUMP_SQUAT,         /* 下蹲蓄力 */
    JUMP_PUSH,          /* 全力蹬地, 直到离地 */
    JUMP_FLY_UP,        /* 上升: 收腿, 等顶点 */
    JUMP_FLY_DOWN,      /* 下降: 伸腿够地, 等触地 */
    JUMP_CUSHION,       /* 缓冲: 深收 → 按住 → 慢放 */
    JUMP_INTERVAL,      /* 跳跃间隔 */
    JUMP_END            /* 结束 */
} JumpState_t;

/* ===================== 可调参数 (1 kHz) ===================== */

#define SQUAT_CYCLES          500     /* 下蹲 500ms */
#define PUSH_TIMEOUT_CYCLES   200     /* 推地最多 200ms */
#define CUSHION_HOLD_CYCLES   200     /* 深收保持 200ms */
#define CUSHION_RELEASE_CYCLES 600    /* 缓慢释放 600ms */
#define CUSHION_RAMP_MS       30      /* 收腿渐变 30ms */
#define BALANCE_RAMP_MS       50      /* 平衡渐变 50ms */
#define INTERVAL_CYCLES       10000   /* 跳跃间隔 10s */

/* 飞行阶段 */
#define APOGEE_THRESHOLD      0.3f    /* accel_z < 此值 = 到顶 */
#define APOGEE_TIMEOUT_MS     150     /* 等顶点超时 */
#define EXTEND_MS             30      /* 伸腿时间 */
#define IMPACT_ACCEL          1.2f    /* accel_z > 此值 = 触地 */
#define FREEZE_MS             15      /* 伸腿后等 15ms 才检测触地, 防误触发 */
#define FLY_TIMEOUT_CYCLES    500     /* 飞行总超时 */

#define ACCEL_FREEFALL_THRESHOLD 0.3f /* 自由落体判据 */

/* 足端偏移 (mm), Y<0=伸腿 Y>0=收腿 */
#define OFF_PUSH_Y     -400.0f       /* 蹬地 */
#define OFF_SQUAT_Y     120.0f       /* 下蹲 */
#define OFF_TUCK_Y       85.0f       /* 空中收腿 */
#define OFF_REACH_Y     -60.0f       /* 伸腿够地 */
#define OFF_CUSHION_Y   130.0f       /* 缓冲最深收腿 */

#define FORWARD_PWM     300          /* 前倾补偿 */

#define TOTAL_JUMPS     3

/* ======================== 静态变量 ============================= */

static JumpState_t  g_state    = JUMP_IDLE;
static uint16_t     g_cycle    = 0;
static uint8_t      g_jump_num = 0;

static Leg_PID_t    g_leg_left_pid;
static Leg_PID_t    g_leg_right_pid;
static Leg_Target_t g_target_left;
static Leg_Target_t g_target_right;

/* ======================== 辅助函数 ============================= */

static void reset_balance(void) {
    robot_control_reset_balance_pid();
}

static void run_leg_pid(const Sensor_data_t *sensor,
                        Motor_cmd_duty_t *motor_cmd)
{
    leg_pid_control(&g_leg_left_pid,  &g_target_left,
                    sensor, LEG_LEFT,  motor_cmd);
    leg_pid_control(&g_leg_right_pid, &g_target_right,
                    sensor, LEG_RIGHT, motor_cmd);
}

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

    float yaw_out = pid_calculate(&g_yaw_pid, 0.0f, -sensor->gyro_yaw);
    int   yaw_pwm = ROUND(yaw_out);
    motor_cmd->left_motor_pwm  += yaw_pwm;
    motor_cmd->right_motor_pwm -= yaw_pwm;

    if (weight < 1.0f) {
        motor_cmd->left_motor_pwm  = (int)(motor_cmd->left_motor_pwm  * weight);
        motor_cmd->right_motor_pwm = (int)(motor_cmd->right_motor_pwm * weight);
    }
}

/** 线性插值: from → to, t in [0,1] */
static float lerp(float from, float to, float t) {
    if (t <= 0.0f) return from;
    if (t >= 1.0f) return to;
    return from + (to - from) * t;
}

static void enter_state(JumpState_t new_state) {
    g_state = new_state;
    g_cycle = 0;
}

/* ====================== 状态机 ============================= */

/* ── SQUAT: 下蹲蓄力 ── */
static void run_squat(const Sensor_data_t *sensor,
                      Motor_cmd_duty_t *motor_cmd)
{
    Foot_position_t pos = { 0.0f, OFF_SQUAT_Y };

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

    leg_offset_to_joint_target(LEG_LEFT,  &pos, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &pos, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= SQUAT_CYCLES) {
        reset_balance();
        enter_state(JUMP_PUSH);
    }
}

/* ── PUSH: 全力蹬地, 直到离地 ── */
static void run_push(const Sensor_data_t *sensor,
                     Motor_cmd_duty_t *motor_cmd)
{
    Foot_position_t pos = { 0.0f, OFF_PUSH_Y };

    if (g_cycle == 1) {
        /* 拉高 kp + ki, 全力推 */
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

    leg_offset_to_joint_target(LEG_LEFT,  &pos, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &pos, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    /* 离地: accel_z 掉到接近 0 */
    bool airborne = false;
    if (g_cycle > 80) {
        airborne = (fabsf(sensor->accel_z) < ACCEL_FREEFALL_THRESHOLD);
    }

    if (airborne || g_cycle >= PUSH_TIMEOUT_CYCLES) {
        /* 恢复 kp, 收 ki, 收腿过渡在 FLY_UP 里做 */
        g_leg_left_pid.front.kp  = 1200.0f;
        g_leg_left_pid.front.ki  = 0.0f;
        g_leg_left_pid.back.kp   = 1200.0f;
        g_leg_left_pid.back.ki   = 0.0f;
        g_leg_right_pid.front.kp = 1200.0f;
        g_leg_right_pid.front.ki = 0.0f;
        g_leg_right_pid.back.kp  = 1200.0f;
        g_leg_right_pid.back.ki  = 0.0f;
        reset_balance();
        enter_state(JUMP_FLY_UP);
    }
}

/* ── FLY_UP: 上升段, 腿从 PUSH 位收到 TUCK, 等顶点 ── */
static void run_fly_up(const Sensor_data_t *sensor,
                       Motor_cmd_duty_t *motor_cmd)
{
    static float start_y = OFF_PUSH_Y;
    float y;
    float weight;

    if (g_cycle == 1) {
        /* 记录当前位置作为过渡起点, 从 PUSH(-400) 线性到 TUCK(85) */
        start_y = OFF_PUSH_Y;  /* 简化: 从推地位开始 */
    }

    /* 30ms 内从 PUSH 位过渡到 TUCK */
    float t = (float)g_cycle / 30.0f;
    if (t > 1.0f) t = 1.0f;
    y = lerp(start_y, OFF_TUCK_Y, t);

    /* 平衡权重从 1 衰减到 0, 120ms 内 */
    weight = 1.0f - (float)g_cycle / 120.0f;
    if (weight < 0.0f) weight = 0.0f;

    Foot_position_t pos = { 0.0f, y };

    balance_with_yaw_scaled(sensor, motor_cmd, weight);

    leg_offset_to_joint_target(LEG_LEFT,  &pos, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &pos, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    /* 到顶: accel_z 降到阈值以下 */
    if (sensor->accel_z < APOGEE_THRESHOLD || g_cycle > APOGEE_TIMEOUT_MS) {
        enter_state(JUMP_FLY_DOWN);
    } else if (g_cycle > FLY_TIMEOUT_CYCLES) {
        enter_state(JUMP_CUSHION);
    }
}

/* ── FLY_DOWN: 下降段, 伸腿够地 → 保持 → 等触地 ── */
static void run_fly_down(const Sensor_data_t *sensor,
                         Motor_cmd_duty_t *motor_cmd)
{
    float y;

    if (g_cycle < EXTEND_MS) {
        /* 伸腿: TUCK(85) → REACH(-60) */
        float t = (float)g_cycle / EXTEND_MS;
        y = lerp(OFF_TUCK_Y, OFF_REACH_Y, t);
    } else {
        /* 保持伸展, 等触地 */
        y = OFF_REACH_Y;
    }

    Foot_position_t pos = { 0.0f, y };

    balance_with_yaw_scaled(sensor, motor_cmd, 0.0f);

    leg_offset_to_joint_target(LEG_LEFT,  &pos, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &pos, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    /* 触地检测: accel_z 尖峰 */
    bool impact = (sensor->accel_z > IMPACT_ACCEL) && (g_cycle > EXTEND_MS + FREEZE_MS);

    if (impact || g_cycle > FLY_TIMEOUT_CYCLES) {
        reset_balance();
        enter_state(JUMP_CUSHION);
    }
}

/* ── CUSHION: 深收 → 按住 → 慢放 ── */
static void run_cushion(const Sensor_data_t *sensor,
                        Motor_cmd_duty_t *motor_cmd)
{
    float y;
    float forward;
    float balance_weight;

    if (g_cycle == 1) {
        /* 拉高 kp, 快速收腿 */
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

    /* 腿目标: 前 30ms 从 REACH 渐变到 CUSHION, 避免突变 */
    if (g_cycle <= CUSHION_RAMP_MS) {
        float t = (float)g_cycle / (float)CUSHION_RAMP_MS;
        y = lerp(OFF_REACH_Y, OFF_CUSHION_Y, t);
        forward = t;
    } else if (g_cycle <= CUSHION_HOLD_CYCLES) {
        /* 深收保持 */
        y = OFF_CUSHION_Y;
        forward = 1.0f;
    } else {
        /* 缓慢释放到标称 (0,0) */
        float t = (float)(g_cycle - CUSHION_HOLD_CYCLES) / (float)CUSHION_RELEASE_CYCLES;
        if (t > 1.0f) t = 1.0f;
        y = lerp(OFF_CUSHION_Y, 0.0f, t);
        forward = 1.0f - t;
    }

    /* 平衡权重: 前 50ms 从 0.2 渐变到 1.0, 避免突然全开 */
    if (g_cycle <= BALANCE_RAMP_MS) {
        balance_weight = 0.2f + 0.8f * (float)g_cycle / (float)BALANCE_RAMP_MS;
    } else {
        balance_weight = 1.0f;
    }

    Foot_position_t pos = { 0.0f, y };

    balance_with_yaw_scaled(sensor, motor_cmd, balance_weight);
    motor_cmd->left_motor_pwm  += (int)(FORWARD_PWM * forward);
    motor_cmd->right_motor_pwm += (int)(FORWARD_PWM * forward);

    if (g_cycle > CUSHION_HOLD_CYCLES) {
        robot_control_leg_speed_feedback(sensor, &pos, &pos);
    }
    leg_offset_to_joint_target(LEG_LEFT,  &pos, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &pos, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= CUSHION_HOLD_CYCLES + CUSHION_RELEASE_CYCLES) {
        g_leg_left_pid.front.kp  = 1200.0f;
        g_leg_left_pid.back.kp   = 1200.0f;
        g_leg_right_pid.front.kp = 1200.0f;
        g_leg_right_pid.back.kp  = 1200.0f;
        g_jump_num++;
        if (g_jump_num < TOTAL_JUMPS) {
            enter_state(JUMP_INTERVAL);
        } else {
            enter_state(JUMP_END);
        }
    }
}

/* ── INTERVAL: 正常站立 ── */
static void run_interval(const Sensor_data_t *sensor,
                         Motor_cmd_duty_t *motor_cmd)
{
    Foot_position_t pos = { 0.0f, 0.0f };

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

    robot_control_leg_speed_feedback(sensor, &pos, &pos);
    leg_offset_to_joint_target(LEG_LEFT,  &pos, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &pos, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= INTERVAL_CYCLES) {
        reset_balance();
        enter_state(JUMP_SQUAT);
    }
}

/* ── END: 恢复, 然后空闲 ── */
static void run_end(const Sensor_data_t *sensor,
                    Motor_cmd_duty_t *motor_cmd)
{
    Foot_position_t pos = { 0.0f, 0.0f };

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

    leg_offset_to_joint_target(LEG_LEFT,  &pos, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &pos, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= 500) {
        enter_state(JUMP_IDLE);
    }
}

/* ======================== 公开接口 ============================= */

void jump_init(void) {
    leg_pid_init(&g_leg_left_pid,  1200.0f, 0.0f, 4.0f, 10000.0f, 0.0f);
    leg_pid_init(&g_leg_right_pid, 1200.0f, 0.0f, 4.0f, 10000.0f, 0.0f);

    g_state    = JUMP_IDLE;
    g_cycle    = 0;
    g_jump_num = 0;

    g_target_left.front  = LEG_NOM_FRONT_L;
    g_target_left.back   = LEG_NOM_BACK_L;
    g_target_right.front = LEG_NOM_FRONT_R;
    g_target_right.back  = LEG_NOM_BACK_R;
}

void jump_start(float target_speed) {
    if (g_state == JUMP_IDLE) {
        (void)target_speed;
        g_jump_num = 0;
        pid_reset(&g_leg_left_pid.front);
        pid_reset(&g_leg_left_pid.back);
        pid_reset(&g_leg_right_pid.front);
        pid_reset(&g_leg_right_pid.back);
        enter_state(JUMP_INTERVAL);
    }
}

void jump_stop(void) {
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
    case JUMP_SQUAT:    run_squat(sensor, motor_cmd);    break;
    case JUMP_PUSH:     run_push(sensor, motor_cmd);     break;
    case JUMP_FLY_UP:   run_fly_up(sensor, motor_cmd);   break;
    case JUMP_FLY_DOWN: run_fly_down(sensor, motor_cmd); break;
    case JUMP_CUSHION:  run_cushion(sensor, motor_cmd);  break;
    case JUMP_INTERVAL: run_interval(sensor, motor_cmd); break;
    case JUMP_END:      run_end(sensor, motor_cmd);      break;
    case JUMP_IDLE:
    default:
        break;
    }
}
