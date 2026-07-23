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

#define UPHILL_SPEED              -0.4f   /* 爬坡速度 */
#define UPHILL_TIMEOUT_MS         10000   /* 爬坡超时 */
#define SINGLE_BRIDGE_SPEED       -0.3f   /* 单边桥速度 */
#define SINGLE_BRIDGE_TIMEOUT_MS  4000    /* 单边桥超时 */
#define SINGLE_BRIDGE_ROLL_KP     -20.0f  /* 单边桥 roll PID KP, 保持车身水平 */

static bool     g_bridge_climb_active  = false;
static bool     g_bridge_use_roll      = false;  /* 单边桥: 启用 roll 闭环 */
static float    g_bridge_target_speed  = 0.0f;
static uint16_t g_bridge_timeout_ms    = 0;
static uint16_t g_bridge_climb_timer   = 0;

void track_bridge_climb_init(void)
{
    g_bridge_climb_active = false;
    g_bridge_use_roll     = false;
    g_bridge_climb_timer  = 0;
}

/* ── 爬坡: 无 roll, 速度 -0.5 ── */
void track_bridge_climb_activate(void)
{
    g_bridge_climb_active = true;
    g_bridge_use_roll     = false;
    g_bridge_target_speed = UPHILL_SPEED;
    g_bridge_timeout_ms   = UPHILL_TIMEOUT_MS;
    g_bridge_climb_timer  = 0;
}

/* ── 单边桥: 有 roll, 速度 -0.3 ── */
void track_single_bridge_activate(void)
{
    g_bridge_climb_active = true;
    g_bridge_use_roll     = true;
    g_bridge_target_speed = SINGLE_BRIDGE_SPEED;
    g_bridge_timeout_ms   = SINGLE_BRIDGE_TIMEOUT_MS;
    g_bridge_climb_timer  = 0;
    g_leg_roll_pid.kp     = SINGLE_BRIDGE_ROLL_KP;
    pid_reset(&g_leg_roll_pid);
}

void track_bridge_climb_deactivate(void)
{
    g_bridge_climb_active = false;
    g_bridge_climb_timer  = 0;
    if (g_bridge_use_roll) {
        g_leg_roll_pid.kp = 0.0f;
        g_bridge_use_roll = false;
    }
}

void track_single_bridge_deactivate(void)
{
    track_bridge_climb_deactivate();  /* 共用退出逻辑 (含 roll 恢复) */
}

bool track_bridge_climb_is_active(void)
{
    return g_bridge_climb_active;
}

void track_bridge_climb_apply(Move_cmd_t *cmd)
{
    if (g_bridge_climb_active) {
        cmd->target_speed = g_bridge_target_speed;
        g_bridge_climb_timer++;
        if (g_bridge_climb_timer >= g_bridge_timeout_ms) {
            track_bridge_climb_deactivate();
        }
    }
}

/* =========================== 颠簸路段 ===========================
 *  策略: 双腿伸腿(负Y)抬高底盘 + 平衡PID等比缩小 + 低速通过
 *  适用于密集路肩: 2cm高 / 2.5cm宽, 间距 ~10cm, 总长 >=1m
 *  倾角保护在 robot_control.c 中 conditionally 关闭 */

/* ── 辅助: 线性插值 ── */
static float bumpy_lerp(float from, float to, float t) {
    if (t <= 0.0f) return from;
    if (t >= 1.0f) return to;
    return from + (to - from) * t;
}

/* ── 参数 ── */
#define BUMPY_SPEED               -0.08f
#define BUMPY_LEG_KP              1100.0f  /* 接近标称, 撑住车身维持抬体位 */
#define BUMPY_LEG_KD              3.0f
#define BUMPY_LEG_EXTEND          -120.0f  /* 双腿伸腿 (mm), 负=伸腿=提底盘 */
#define BUMPY_RAMP_MS             200      /* 伸腿渐变时间, 行程大适当延长 */
#define BUMPY_TIMEOUT_MS          10000    /* 总超时 10s (1m / 0.14 ≈ 7s + 余量) */

/* 平衡 PID: 腿伸长后等效摆长变大, 增益等比缩小防振荡 */
/*          标称 COM 高度 ~150mm, 伸腿 -120mm 后 ~270mm, 缩放比 sqrt(150/270) ≈ 0.745 */
#define BUMPY_BALANCE_ANGLE_KP    18.6f    /* 标称 25.0 * 0.745 */
#define BUMPY_BALANCE_GYRO_KP     -820.0f  /* 标称 -1100 * 0.745 */
#define BUMPY_BALANCE_GYRO_KD    2.2f     /* 标称 3.0 * 0.745 */

/* 标称值 (用于恢复) */
#define NOMINAL_LEG_KP            1200.0f
#define NOMINAL_LEG_KD            4.0f
#define NOMINAL_BALANCE_ANGLE_KP  25.0f
#define NOMINAL_BALANCE_GYRO_KP   -1100.0f
#define NOMINAL_BALANCE_GYRO_KD  3.0f

/* ── 状态 ── */
static bool     g_bumpy_active     = false;
static float    g_bumpy_yaw_bias   = 0.0f;
static float    g_bumpy_left_lift  = 0.0f;
static float    g_bumpy_right_lift = 0.0f;
static bool     g_speed_inited     = false;
static uint16_t g_bumpy_timer      = 0;

/* ── 公开接口 ── */

void track_bumpy_init(void)
{
    g_bumpy_active     = false;
    g_bumpy_yaw_bias   = 0.0f;
    g_bumpy_left_lift  = 0.0f;
    g_bumpy_right_lift = 0.0f;
    g_speed_inited     = false;
}

void track_bumpy_activate(void)
{
    g_bumpy_active = true;
    g_speed_inited = false;
}

void track_bumpy_deactivate(void)
{
    g_bumpy_active = false;
}

bool track_bumpy_is_active(void)
{
    return g_bumpy_active;
}

float track_bumpy_get_speed(void)
{
    return BUMPY_SPEED;
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

/* ── 增益切换 ── */

void track_bumpy_apply_compliance(void)
{
    g_leg_left_pid.front.kp  = BUMPY_LEG_KP;
    g_leg_left_pid.front.kd  = BUMPY_LEG_KD;
    g_leg_left_pid.back.kp   = BUMPY_LEG_KP;
    g_leg_left_pid.back.kd   = BUMPY_LEG_KD;
    g_leg_right_pid.front.kp = BUMPY_LEG_KP;
    g_leg_right_pid.front.kd = BUMPY_LEG_KD;
    g_leg_right_pid.back.kp  = BUMPY_LEG_KP;
    g_leg_right_pid.back.kd  = BUMPY_LEG_KD;
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

void track_bumpy_apply_balance_pid(void)
{
    g_pitch_angle_pid.kp = BUMPY_BALANCE_ANGLE_KP;
    g_pitch_gyro_pid.kp  = BUMPY_BALANCE_GYRO_KP;
    g_pitch_gyro_pid.kd  = BUMPY_BALANCE_GYRO_KD;
}

void track_bumpy_restore_balance_pid(void)
{
    g_pitch_angle_pid.kp = NOMINAL_BALANCE_ANGLE_KP;
    g_pitch_gyro_pid.kp  = NOMINAL_BALANCE_GYRO_KP;
    g_pitch_gyro_pid.kd  = NOMINAL_BALANCE_GYRO_KD;
}

/* ── 状态机 ── */

void track_bumpy_update(const Sensor_data_t *sensor)
{
    if (!g_bumpy_active || sensor == NULL) return;

    if (!g_speed_inited) {
        g_speed_inited    = true;
        g_bumpy_timer     = 0;
        g_bumpy_yaw_bias  = 0.0f;
        g_bumpy_left_lift  = 0.0f;
        g_bumpy_right_lift = 0.0f;
        return;
    }

    g_bumpy_timer++;

    /* 总超时保护 */
    if (g_bumpy_timer >= BUMPY_TIMEOUT_MS) {
        g_bumpy_active     = false;
        g_bumpy_yaw_bias   = 0.0f;
        g_bumpy_left_lift  = 0.0f;
        g_bumpy_right_lift = 0.0f;
        return;
    }

    /* 直线行驶 */
    g_bumpy_yaw_bias = 0.0f;

    /* 双腿伸腿: 渐变 0 → LEG_EXTEND (负值 = 伸腿 = 提底盘) */
    if (g_bumpy_timer < BUMPY_RAMP_MS) {
        float t = (float)g_bumpy_timer / (float)BUMPY_RAMP_MS;
        float lift = bumpy_lerp(0.0f, BUMPY_LEG_EXTEND, t);
        g_bumpy_left_lift  = lift;
        g_bumpy_right_lift = lift;
    } else {
        g_bumpy_left_lift  = BUMPY_LEG_EXTEND;
        g_bumpy_right_lift = BUMPY_LEG_EXTEND;
    }
}
