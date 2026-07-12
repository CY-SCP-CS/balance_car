#include "track_elements.h"
#include "../../lib/pid/pid_calculate.h"
#include "../../common/types.h"
#include "robot_control.h"
#include <math.h>

/* =========================== 原地旋转720度 =========================== */

#define ROT720_SPEED      (3.5f * M_PI)   /* 450°/s */
#define ROT720_TARGET     (4.0f * M_PI)   /* 720° = 4π rad */
#define ROT720_MARGIN     (0.0f * M_PI)   /* 10% 冗余 ≈ 72° */
#define ROT720_LEAN_GAIN  0.35f           /* 压弯系数 (原0.2) */

typedef enum {
    ROTATE720_IDLE = 0,
    ROTATE720_ACTIVE,
    ROTATE720_DONE
} Rotate720State_t;

static Rotate720State_t g_rotate720_state = ROTATE720_IDLE;
static float            g_rotate720_accum = 0.0f;
static float            g_rotate720_target = 0.0f;

void track_rotate720_init(void)
{
    g_rotate720_state  = ROTATE720_IDLE;
    g_rotate720_accum  = 0.0f;
    g_rotate720_target = 0.0f;
}

void track_rotate720_start(void)
{
    if (g_rotate720_state == ROTATE720_IDLE) {
        g_rotate720_state  = ROTATE720_ACTIVE;
        g_rotate720_accum  = 0.0f;
        g_rotate720_target = 0.0f;
    }
}

bool track_rotate720_is_active(void)
{
    return (g_rotate720_state == ROTATE720_ACTIVE);
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
}

void track_rotate720_update(Sensor_data_t *sensor, Move_cmd_t *cmd)
{
    if (g_rotate720_state != ROTATE720_ACTIVE) return;

    /* 首周期用当前 yaw 初始化移动目标 */
    if (g_rotate720_accum == 0.0f) {
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

#define BUMPY_CLEARANCE_Y  60.0f    /* 收腿量 (mm) */
#define BUMPY_KP           400.0f   /* 软Kp (原1200) */
#define BUMPY_KD           10.0f    /* 大阻尼 (原4) */
#define BUMPY_SPEED        0.2f     /* 归一化慢速 */
#define NOMINAL_KP         1200.0f  /* 标称Kp */
#define NOMINAL_KD         4.0f     /* 标称Kd */

static bool g_bumpy_active = false;

void track_bumpy_init(void)
{
    g_bumpy_active = false;
}

void track_bumpy_activate(void)
{
    g_bumpy_active = true;
}

void track_bumpy_deactivate(void)
{
    g_bumpy_active = false;
}

bool track_bumpy_is_active(void)
{
    return g_bumpy_active;
}

float track_bumpy_get_clearance_offset(void)
{
    return BUMPY_CLEARANCE_Y;
}

void track_bumpy_apply_leg_gains(Leg_PID_t *left, Leg_PID_t *right)
{
    left->front.kp  = BUMPY_KP;
    left->front.kd  = BUMPY_KD;
    left->back.kp   = BUMPY_KP;
    left->back.kd   = BUMPY_KD;
    right->front.kp = BUMPY_KP;
    right->front.kd = BUMPY_KD;
    right->back.kp  = BUMPY_KP;
    right->back.kd  = BUMPY_KD;
}

void track_bumpy_restore_leg_gains(Leg_PID_t *left, Leg_PID_t *right)
{
    left->front.kp  = NOMINAL_KP;
    left->front.kd  = NOMINAL_KD;
    left->back.kp   = NOMINAL_KP;
    left->back.kd   = NOMINAL_KD;
    right->front.kp = NOMINAL_KP;
    right->front.kd = NOMINAL_KD;
    right->back.kp  = NOMINAL_KP;
    right->back.kd  = NOMINAL_KD;
}
