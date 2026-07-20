#include "track_elements.h"
#include "../../lib/pid/pid_calculate.h"
#include "../../common/types.h"
#include "robot_control.h"
#include <math.h>
#include <stddef.h>

/* =========================== 原地旋转720度 =========================== */

#define ROT720_SPEED      (3.5f * M_PI)   /* 450°/s */
#define ROT720_TARGET     (4.0f * M_PI)   /* 720° = 4π rad */
#define ROT720_MARGIN     (0.0f * M_PI)   /* 10% 冗余 ≈ 72° */
#define ROT720_LEAN_GAIN  0.35f           /* 压弯系数 (原0.2) */
#define ROT720_STOP_SPEED_MPS   0.03f
#define ROT720_STOP_HOLD_TIME_S 0.08f

typedef enum {
    ROTATE720_IDLE = 0,
    ROTATE720_BRAKING,
    ROTATE720_ACTIVE,
    ROTATE720_DONE
} Rotate720State_t;

static Rotate720State_t g_rotate720_state = ROTATE720_IDLE;
static float            g_rotate720_accum = 0.0f;
static float            g_rotate720_target = 0.0f;
static float            g_rotate720_stop_hold_s = 0.0f;
static bool             g_rotate720_target_valid = false;

static float rotate720_forward_speed_mps(const Sensor_data_t *sensor)
{
    float wheel_radius_m = LEG_WHEEL_RADIUS * 0.001f;
    float left_mps = sensor->motor_left_speed * wheel_radius_m;
    float right_mps = sensor->motor_right_speed * wheel_radius_m;

    return fabsf(0.5f * (left_mps + right_mps));
}

void track_rotate720_init(void)
{
    g_rotate720_state  = ROTATE720_IDLE;
    g_rotate720_accum  = 0.0f;
    g_rotate720_target = 0.0f;
    g_rotate720_stop_hold_s = 0.0f;
    g_rotate720_target_valid = false;
}

void track_rotate720_start(void)
{
    if (g_rotate720_state == ROTATE720_IDLE) {
        g_rotate720_state  = ROTATE720_BRAKING;
        g_rotate720_accum  = 0.0f;
        g_rotate720_target = 0.0f;
        g_rotate720_stop_hold_s = 0.0f;
        g_rotate720_target_valid = false;
    }
}

bool track_rotate720_is_active(void)
{
    return (g_rotate720_state == ROTATE720_BRAKING ||
            g_rotate720_state == ROTATE720_ACTIVE);
}

bool track_rotate720_is_done(void)
{
    return (g_rotate720_state == ROTATE720_DONE);
}

void track_rotate720_reset(void)
{
    g_rotate720_state  = ROTATE720_IDLE;
    g_rotate720_accum  = 0.0f;
    g_rotate720_target = 0.0f;
    g_rotate720_stop_hold_s = 0.0f;
    g_rotate720_target_valid = false;
}

void track_rotate720_update(Sensor_data_t *sensor, Move_cmd_t *cmd)
{
    if (g_rotate720_state != ROTATE720_BRAKING &&
        g_rotate720_state != ROTATE720_ACTIVE) {
        return;
    }

    cmd->target_speed = 0.0f;
    cmd->target_distance = 0.0f;

    if (!g_rotate720_target_valid) {
        g_rotate720_target = sensor->angle_yaw;
        g_rotate720_target_valid = true;
    }

    if (g_rotate720_state == ROTATE720_BRAKING) {
        cmd->target_direction = g_rotate720_target;
        cmd->target_roll = 0.0f;

        if (rotate720_forward_speed_mps(sensor) <= ROT720_STOP_SPEED_MPS) {
            g_rotate720_stop_hold_s += ROBOT_CONTROL_DT;
        } else {
            g_rotate720_stop_hold_s = 0.0f;
        }

        if (g_rotate720_stop_hold_s < ROT720_STOP_HOLD_TIME_S) {
            return;
        }

        g_rotate720_state = ROTATE720_ACTIVE;
        g_rotate720_accum = 0.0f;
        g_rotate720_target = sensor->angle_yaw;
    }

    /* 每周期递增目标角度, 不归一化 (yaw 误差归一化逻辑处理角度回绕) */
    g_rotate720_target += ROT720_SPEED * ROBOT_CONTROL_DT;
    cmd->target_direction = g_rotate720_target;

    /* 增强压弯: 旋转期间用更高系数 */
    float roll_from_turn = -ROT720_LEAN_GAIN * sensor->gyro_yaw / MAX_YAW_RATE;
    cmd->target_roll = CLAMP(roll_from_turn, -1.0f, 1.0f);

    /* 用实际 gyro 积分累积转角 (取绝对值, 方向由速率指令保证) */
    g_rotate720_accum += fabsf(sensor->gyro_yaw) * ROBOT_CONTROL_DT;

    if (g_rotate720_accum >= ROT720_TARGET + ROT720_MARGIN) {
        g_rotate720_state = ROTATE720_DONE;
    }
}

/* =========================== 单边桥与爬坡 =========================== */

#define BRIDGE_SPEED 0.6f   /* 归一化 target_speed */

static bool g_bridge_climb_active = false;

void track_bridge_climb_init(void)
{
    g_bridge_climb_active = false;
}

void track_bridge_climb_activate(void)
{
    g_bridge_climb_active = true;
}

void track_bridge_climb_deactivate(void)
{
    g_bridge_climb_active = false;
}

bool track_bridge_climb_is_active(void)
{
    return g_bridge_climb_active;
}

void track_bridge_climb_apply(Move_cmd_t *cmd)
{
    if (g_bridge_climb_active) {
        cmd->target_speed = BRIDGE_SPEED;
    }
}

/* =========================== 颠簸路段 =========================== */

/* ── 共用辅助: 线性插值 ── */
static float bumpy_lerp(float from, float to, float t) {
    if (t <= 0.0f) return from;
    if (t >= 1.0f) return to;
    return from + (to - from) * t;
}

/* ── 共享状态 (两种模式共用) ── */
static bool  g_bumpy_active      = false;
static float g_bumpy_yaw_bias    = 0.0f;
static float g_bumpy_left_lift   = 0.0f;
static float g_bumpy_right_lift  = 0.0f;

/* 轮速历史, 用于计算减速度 / 卡死检测 */
static float g_prev_left_speed  = 0.0f;
static float g_prev_right_speed = 0.0f;
static bool  g_speed_inited     = false;

/* ── 公开接口 (两种模式共用) ── */

void track_bumpy_init(void)
{
    g_bumpy_active      = false;
    g_bumpy_yaw_bias    = 0.0f;
    g_bumpy_left_lift   = 0.0f;
    g_bumpy_right_lift  = 0.0f;
    g_speed_inited      = false;
}

void track_bumpy_activate(void)
{
    g_bumpy_active  = true;
    g_speed_inited  = false;
}

void track_bumpy_deactivate(void)
{
    g_bumpy_active      = false;
    /* lift/yaw 不清零: deactivate 后 is_active()=false 不再读取,
     * 下次 activate 通过 g_speed_inited=false 重新初始化, 避免退出时阶跃冲击 */
}

bool track_bumpy_is_active(void)
{
    return g_bumpy_active;
}

float track_bumpy_get_yaw_bias(void)
{
    return g_bumpy_yaw_bias;
}

float track_bumpy_get_left_lift(void)
{
    return g_bumpy_left_lift;
}

float track_bumpy_get_right_lift(void)
{
    return g_bumpy_right_lift;
}


/* ================================================================
 *  模式 A: 优化交替偏航 (BUMPY_MODE == BUMPY_MODE_ALTERNATING)
 * ================================================================ */
#if BUMPY_MODE == BUMPY_MODE_ALTERNATING

typedef enum {
    BUMPY_YAW_RIGHT  = 0,
    BUMPY_LEFT_CLIMB,
    BUMPY_YAW_LEFT,
    BUMPY_RIGHT_CLIMB
} BumpyPhase_t;

/* ── 参数 (优化后) ── */
#define BUMPY_A_YAW_BIAS_DEG     5.0f
#define BUMPY_A_LIFT_Y           40.0f
#define BUMPY_A_CLIMB_MS         350
#define BUMPY_A_WAIT_TIMEOUT_MS  1100
#define BUMPY_A_IMPACT_DECEL     20.0f
#define BUMPY_A_SPEED            0.12f
#define BUMPY_A_RAMP_MS          80
#define BUMPY_A_CONFIRM_CYCLES   3
#define BUMPY_A_TOTAL_TIMEOUT_MS 12000

static BumpyPhase_t g_bumpy_phase       = BUMPY_YAW_RIGHT;
static uint16_t     g_bumpy_timer       = 0;
static uint16_t     g_bumpy_total_timer = 0;
static uint8_t      g_impact_confirm    = 0;    /* 碰撞确认计数 */

float track_bumpy_get_speed(void)
{
    return BUMPY_A_SPEED;
}

/* 模式 A: KP 不变, 空操作 */
void track_bumpy_apply_compliance(void)  { }
void track_bumpy_restore_stiffness(void) { }

void track_bumpy_update(const Sensor_data_t *sensor)
{
    if (!g_bumpy_active || sensor == NULL) return;

    float left_speed  = sensor->motor_left_speed;
    float right_speed = sensor->motor_right_speed;

    if (!g_speed_inited) {
        g_prev_left_speed  = left_speed;
        g_prev_right_speed = right_speed;
        g_speed_inited     = true;
        g_bumpy_phase      = BUMPY_YAW_RIGHT;
        g_bumpy_timer      = 0;
        g_bumpy_total_timer = 0;
        g_impact_confirm   = 0;
        return;
    }

    float dt = ROBOT_CONTROL_DT;
    float left_accel  = (left_speed  - g_prev_left_speed)  / dt;
    float right_accel = (right_speed - g_prev_right_speed) / dt;

    g_prev_left_speed  = left_speed;
    g_prev_right_speed = right_speed;

    g_bumpy_timer++;
    g_bumpy_total_timer++;

    /* 总超时保护: 12s 强制退出 */
    if (g_bumpy_total_timer >= BUMPY_A_TOTAL_TIMEOUT_MS) {
        g_bumpy_active     = false;
        g_bumpy_yaw_bias   = 0.0f;
        g_bumpy_left_lift  = 0.0f;
        g_bumpy_right_lift = 0.0f;
        return;
    }

    switch (g_bumpy_phase) {

    case BUMPY_YAW_RIGHT: {
        g_bumpy_yaw_bias   =  BUMPY_A_YAW_BIAS_DEG * DEG_TO_RAD;
        g_bumpy_left_lift  =  0.0f;
        g_bumpy_right_lift =  0.0f;

        bool left_impact  = (left_accel  < -BUMPY_A_IMPACT_DECEL);
        bool right_quiet  = (right_accel > -BUMPY_A_IMPACT_DECEL * 0.5f);

        if (left_impact && right_quiet) {
            g_impact_confirm++;
            if (g_impact_confirm >= BUMPY_A_CONFIRM_CYCLES) {
                g_bumpy_phase = BUMPY_LEFT_CLIMB;
                g_bumpy_timer = 0;
                g_impact_confirm = 0;
            }
        } else {
            g_impact_confirm = 0;
        }

        if (g_bumpy_timer >= BUMPY_A_WAIT_TIMEOUT_MS) {
            g_bumpy_phase = BUMPY_LEFT_CLIMB;
            g_bumpy_timer = 0;
            g_impact_confirm = 0;
        }
        break;
    }

    case BUMPY_LEFT_CLIMB: {
        g_bumpy_yaw_bias   =  BUMPY_A_YAW_BIAS_DEG * DEG_TO_RAD;
        g_bumpy_right_lift =  0.0f;

        /* 收腿/放腿渐变: 前80ms上拉, 后80ms下放, 中间保持 */
        uint16_t climb_ms = BUMPY_A_CLIMB_MS;
        uint16_t ramp     = BUMPY_A_RAMP_MS;

        if (g_bumpy_timer < ramp) {
            float t = (float)g_bumpy_timer / (float)ramp;
            g_bumpy_left_lift = bumpy_lerp(0.0f, BUMPY_A_LIFT_Y, t);
        } else if (g_bumpy_timer < climb_ms - ramp) {
            g_bumpy_left_lift = BUMPY_A_LIFT_Y;
        } else if (g_bumpy_timer < climb_ms) {
            uint16_t rel = g_bumpy_timer - (climb_ms - ramp);
            float t = (float)rel / (float)ramp;
            g_bumpy_left_lift = bumpy_lerp(BUMPY_A_LIFT_Y, 0.0f, t);
        } else {
            g_bumpy_left_lift = 0.0f;
        }

        if (g_bumpy_timer >= climb_ms) {
            g_bumpy_phase = BUMPY_YAW_LEFT;
            g_bumpy_timer = 0;
        }
        break;
    }

    case BUMPY_YAW_LEFT: {
        g_bumpy_yaw_bias   = -BUMPY_A_YAW_BIAS_DEG * DEG_TO_RAD;
        g_bumpy_left_lift  =  0.0f;
        g_bumpy_right_lift =  0.0f;

        bool right_impact = (right_accel < -BUMPY_A_IMPACT_DECEL);
        bool left_quiet   = (left_accel  > -BUMPY_A_IMPACT_DECEL * 0.5f);

        if (right_impact && left_quiet) {
            g_impact_confirm++;
            if (g_impact_confirm >= BUMPY_A_CONFIRM_CYCLES) {
                g_bumpy_phase = BUMPY_RIGHT_CLIMB;
                g_bumpy_timer = 0;
                g_impact_confirm = 0;
            }
        } else {
            g_impact_confirm = 0;
        }

        if (g_bumpy_timer >= BUMPY_A_WAIT_TIMEOUT_MS) {
            g_bumpy_phase = BUMPY_RIGHT_CLIMB;
            g_bumpy_timer = 0;
            g_impact_confirm = 0;
        }
        break;
    }

    case BUMPY_RIGHT_CLIMB: {
        g_bumpy_yaw_bias  = -BUMPY_A_YAW_BIAS_DEG * DEG_TO_RAD;
        g_bumpy_left_lift =  0.0f;

        uint16_t climb_ms = BUMPY_A_CLIMB_MS;
        uint16_t ramp     = BUMPY_A_RAMP_MS;

        if (g_bumpy_timer < ramp) {
            float t = (float)g_bumpy_timer / (float)ramp;
            g_bumpy_right_lift = bumpy_lerp(0.0f, BUMPY_A_LIFT_Y, t);
        } else if (g_bumpy_timer < climb_ms - ramp) {
            g_bumpy_right_lift = BUMPY_A_LIFT_Y;
        } else if (g_bumpy_timer < climb_ms) {
            uint16_t rel = g_bumpy_timer - (climb_ms - ramp);
            float t = (float)rel / (float)ramp;
            g_bumpy_right_lift = bumpy_lerp(BUMPY_A_LIFT_Y, 0.0f, t);
        } else {
            g_bumpy_right_lift = 0.0f;
        }

        if (g_bumpy_timer >= climb_ms) {
            g_bumpy_phase = BUMPY_YAW_RIGHT;
            g_bumpy_timer = 0;
        }
        break;
    }
    }
}


/* ================================================================
 *  模式 B: 柔顺+抬体+卡死脱困 (BUMPY_MODE == BUMPY_MODE_COMPLIANT)
 * ================================================================ */
#elif BUMPY_MODE == BUMPY_MODE_COMPLIANT

typedef enum {
    BUMPY_COMPLIANT = 0,
    BUMPY_ESCAPE_L,
    BUMPY_ESCAPE_R,
    BUMPY_COOLDOWN
} BumpyCState_t;

/* ── 参数 ── */
#define BUMPY_C_SPEED           0.12f
#define BUMPY_C_LEG_KP          400.0f
#define BUMPY_C_LEG_KD          1.0f
#define BUMPY_C_BODY_LIFT       15.0f   /* 双脚延伸抬体 (mm) */
#define BUMPY_C_STUCK_SPEED     0.8f    /* 卡死判定轮速阈值 (rad/s) */
#define BUMPY_C_STUCK_CYCLES    40      /* 40ms 确认窗口 */
#define BUMPY_C_ESCAPE_LIFT     30.0f   /* 被卡侧收腿量 (mm) */
#define BUMPY_C_ESCAPE_YAW_DEG  3.0f
#define BUMPY_C_ESCAPE_RAMP_MS  50
#define BUMPY_C_ESCAPE_MS       400     /* 脱困总时长 */
#define BUMPY_C_COOLDOWN_MS     500

/* 标称增益 (用于恢复) */
#define NOMINAL_LEG_KP  1200.0f
#define NOMINAL_LEG_KD  4.0f

static BumpyCState_t g_bumpy_c_state = BUMPY_COMPLIANT;
static uint16_t      g_bumpy_timer   = 0;
static uint16_t      g_stuck_left    = 0;   /* 左轮卡死连续计数 */
static uint16_t      g_stuck_right   = 0;   /* 右轮卡死连续计数 */

float track_bumpy_get_speed(void)
{
    return BUMPY_C_SPEED;
}

void track_bumpy_apply_compliance(void)
{
    g_leg_left_pid.front.kp  = BUMPY_C_LEG_KP;
    g_leg_left_pid.front.kd  = BUMPY_C_LEG_KD;
    g_leg_left_pid.back.kp   = BUMPY_C_LEG_KP;
    g_leg_left_pid.back.kd   = BUMPY_C_LEG_KD;
    g_leg_right_pid.front.kp = BUMPY_C_LEG_KP;
    g_leg_right_pid.front.kd = BUMPY_C_LEG_KD;
    g_leg_right_pid.back.kp  = BUMPY_C_LEG_KP;
    g_leg_right_pid.back.kd  = BUMPY_C_LEG_KD;
}

void track_bumpy_restore_stiffness(void)
{
    g_leg_left_pid.front.kp  = NOMINAL_LEG_KP;
    g_leg_left_pid.front.kd  = NOMINAL_LEG_KD;
    g_leg_left_pid.back.kp   = NOMINAL_LEG_KP;
    g_leg_left_pid.back.kd   = NOMINAL_LEG_KD;
    g_leg_right_pid.front.kp = NOMINAL_LEG_KP;
    g_leg_right_pid.front.kd = NOMINAL_LEG_KD;
    g_leg_right_pid.back.kp  = NOMINAL_LEG_KP;
    g_leg_right_pid.back.kd  = NOMINAL_LEG_KD;
}

void track_bumpy_update(const Sensor_data_t *sensor)
{
    if (!g_bumpy_active || sensor == NULL) return;

    float left_speed  = sensor->motor_left_speed;
    float right_speed = sensor->motor_right_speed;

    if (!g_speed_inited) {
        g_prev_left_speed  = left_speed;
        g_prev_right_speed = right_speed;
        g_speed_inited     = true;
        g_bumpy_c_state    = BUMPY_COMPLIANT;
        g_bumpy_timer      = 0;
        g_stuck_left       = 0;
        g_stuck_right      = 0;
        return;
    }

    g_prev_left_speed  = left_speed;
    g_prev_right_speed = right_speed;
    g_bumpy_timer++;

    switch (g_bumpy_c_state) {

    case BUMPY_COMPLIANT: {
        /* 双腿 +15mm 延伸抬体, 直线行驶 */
        g_bumpy_yaw_bias   = 0.0f;
        g_bumpy_left_lift  =  BUMPY_C_BODY_LIFT;
        g_bumpy_right_lift =  BUMPY_C_BODY_LIFT;

        /* 卡死检测: 轮速低于阈值连续 N 周期 */
        if (fabsf(left_speed) < BUMPY_C_STUCK_SPEED) {
            g_stuck_left++;
        } else {
            g_stuck_left = 0;
        }
        if (fabsf(right_speed) < BUMPY_C_STUCK_SPEED) {
            g_stuck_right++;
        } else {
            g_stuck_right = 0;
        }

        if (g_stuck_left >= BUMPY_C_STUCK_CYCLES) {
            g_bumpy_c_state = BUMPY_ESCAPE_L;
            g_bumpy_timer   = 0;
            g_stuck_left    = 0;
            g_stuck_right   = 0;
        } else if (g_stuck_right >= BUMPY_C_STUCK_CYCLES) {
            g_bumpy_c_state = BUMPY_ESCAPE_R;
            g_bumpy_timer   = 0;
            g_stuck_left    = 0;
            g_stuck_right   = 0;
        }
        break;
    }

    case BUMPY_ESCAPE_L: {
        /* 左轮被卡: 左侧收腿30mm + 偏右3°脱困, 50ms渐变, 末50ms释放 */
        uint16_t ramp     = BUMPY_C_ESCAPE_RAMP_MS;
        uint16_t escape_ms = BUMPY_C_ESCAPE_MS;

        if (g_bumpy_timer < ramp) {
            float t = (float)g_bumpy_timer / (float)ramp;
            g_bumpy_left_lift  = bumpy_lerp(BUMPY_C_BODY_LIFT,
                                            BUMPY_C_BODY_LIFT + BUMPY_C_ESCAPE_LIFT, t);
            g_bumpy_yaw_bias   = bumpy_lerp(0.0f, BUMPY_C_ESCAPE_YAW_DEG * DEG_TO_RAD, t);
        } else if (g_bumpy_timer < escape_ms - ramp) {
            g_bumpy_left_lift  = BUMPY_C_BODY_LIFT + BUMPY_C_ESCAPE_LIFT;
            g_bumpy_yaw_bias   = BUMPY_C_ESCAPE_YAW_DEG * DEG_TO_RAD;
        } else if (g_bumpy_timer < escape_ms) {
            uint16_t rel = g_bumpy_timer - (escape_ms - ramp);
            float t = (float)rel / (float)ramp;
            g_bumpy_left_lift  = bumpy_lerp(BUMPY_C_BODY_LIFT + BUMPY_C_ESCAPE_LIFT,
                                            BUMPY_C_BODY_LIFT, t);
            g_bumpy_yaw_bias   = bumpy_lerp(BUMPY_C_ESCAPE_YAW_DEG * DEG_TO_RAD, 0.0f, t);
        } else {
            g_bumpy_left_lift  = BUMPY_C_BODY_LIFT;
            g_bumpy_yaw_bias   = 0.0f;
        }
        g_bumpy_right_lift = BUMPY_C_BODY_LIFT;

        if (g_bumpy_timer >= escape_ms) {
            g_bumpy_c_state = BUMPY_COOLDOWN;
            g_bumpy_timer   = 0;
        }
        break;
    }

    case BUMPY_ESCAPE_R: {
        /* 右轮被卡: 右侧收腿30mm + 偏左3°脱困, 50ms渐变, 末50ms释放 */
        uint16_t ramp      = BUMPY_C_ESCAPE_RAMP_MS;
        uint16_t escape_ms = BUMPY_C_ESCAPE_MS;

        if (g_bumpy_timer < ramp) {
            float t = (float)g_bumpy_timer / (float)ramp;
            g_bumpy_right_lift = bumpy_lerp(BUMPY_C_BODY_LIFT,
                                            BUMPY_C_BODY_LIFT + BUMPY_C_ESCAPE_LIFT, t);
            g_bumpy_yaw_bias   = bumpy_lerp(0.0f, -BUMPY_C_ESCAPE_YAW_DEG * DEG_TO_RAD, t);
        } else if (g_bumpy_timer < escape_ms - ramp) {
            g_bumpy_right_lift = BUMPY_C_BODY_LIFT + BUMPY_C_ESCAPE_LIFT;
            g_bumpy_yaw_bias   = -BUMPY_C_ESCAPE_YAW_DEG * DEG_TO_RAD;
        } else if (g_bumpy_timer < escape_ms) {
            uint16_t rel = g_bumpy_timer - (escape_ms - ramp);
            float t = (float)rel / (float)ramp;
            g_bumpy_right_lift = bumpy_lerp(BUMPY_C_BODY_LIFT + BUMPY_C_ESCAPE_LIFT,
                                            BUMPY_C_BODY_LIFT, t);
            g_bumpy_yaw_bias   = bumpy_lerp(-BUMPY_C_ESCAPE_YAW_DEG * DEG_TO_RAD, 0.0f, t);
        } else {
            g_bumpy_right_lift = BUMPY_C_BODY_LIFT;
            g_bumpy_yaw_bias   = 0.0f;
        }
        g_bumpy_left_lift = BUMPY_C_BODY_LIFT;

        if (g_bumpy_timer >= escape_ms) {
            g_bumpy_c_state = BUMPY_COOLDOWN;
            g_bumpy_timer   = 0;
        }
        break;
    }

    case BUMPY_COOLDOWN: {
        /* 冷却期: 恢复抬体位, 直线行驶, 防反复触发 */
        g_bumpy_yaw_bias   = 0.0f;
        g_bumpy_left_lift  = BUMPY_C_BODY_LIFT;
        g_bumpy_right_lift = BUMPY_C_BODY_LIFT;

        if (g_bumpy_timer >= BUMPY_C_COOLDOWN_MS) {
            g_bumpy_c_state = BUMPY_COMPLIANT;
            g_bumpy_timer   = 0;
            g_stuck_left    = 0;
            g_stuck_right   = 0;
        }
        break;
    }
    }
}

#else
#error "BUMPY_MODE must be BUMPY_MODE_ALTERNATING or BUMPY_MODE_COMPLIANT"
#endif
