#include "jump.h"
#include "robot_control.h"
#include <math.h>
#include <stdbool.h>

/* =========================== 状态枚举 =========================== */
typedef enum {
    JUMP_IDLE = 0,
    JUMP_STABILIZE,     /* 自稳等待 (着地, 3秒) */
    JUMP_SQUAT,         /* 下蹲蓄力 (着地) */
    JUMP_LAUNCH,        /* 蹬地伸腿 (着地, 至离地) */
    JUMP_AIR_ASCEND,    /* 上升段: 收腿, 等顶点 */
    JUMP_AIR_DESCEND,   /* 下降段: 伸腿够地, 等触地 */
    JUMP_LAND           /* 缓冲着地 */
} JumpState_t;

/* ===================== 可调参数 ===================== */

/* Y 偏移 (mm, 叠加到 leg_cmd_solve 的 Y 上) */
#define JUMP_SQUAT_Y        100.0f   /* 下蹲收腿深度 */
#define JUMP_PUSH_Y        -400.0f   /* 蹬地伸腿 (饱和在关节限位) */
#define JUMP_TUCK_Y          80.0f   /* 空中收腿 */
#define JUMP_REACH_Y        -60.0f   /* 空中伸腿够地 */
#define JUMP_CUSHION_DELTA   90.0f   /* 落地相对缓冲深度 */

/* 前向推进 */
#define JUMP_FORWARD_BIAS   -0.0f   /* 蹬地时脚在髋后方的偏移 (mm) *///颠簸路段为0

/* 时序 (1kHz cycles) */
#define STABILIZE_DURATION      1500    /* 单次起跳前自稳 1.5 秒 */
#define MULTI_STABILIZE_DURATION 500    /* 多级跳跃时每跳自稳最小 500ms */
#define MULTI_STABILIZE_MAX     1200    /* 多级跳跃自稳超时 (防死等) */
#define SQUAT_DURATION          400
#define MULTI_SQUAT_PRE_DELAY    80     /* 连跳时下蹲前预稳定 80ms, 确保基线干净 */
#define LAUNCH_RAMP_MS         50      /* 伸腿渐变时间 */
#define LAUNCH_TIMEOUT        200
#define TUCK_RAMP_MS           30
#define REACH_RAMP_MS          30
#define CUSHION_RAMP_MS        80
#define CUSHION_HOLD_MS       200
#define CUSHION_RELEASE_MS    400
#define BALANCE_RAMP_MS        50

/* 飞行阶段检测阈值 */
#define APOGEE_THRESHOLD      0.3f    /* accel_z < 此值 = 到顶点 */
#define APOGEE_TIMEOUT_MS     150     /* 等顶点超时 */
#define IMPACT_ACCEL          1.3f    /* accel_z > 此值 = 触地 */
#define FREEZE_MS             2      /* 伸腿后冻结期, 防误触发 */
#define FLY_TIMEOUT_CYCLES    500     /* 飞行总超时 */
#define ACCEL_FREEFALL_THRESHOLD 0.3f /* 自由落体判据 */

/* 稳定判据 (STABILIZE 阶段, 连跳时确保车身不晃再下蹲) */
#define STABLE_GYRO_THRESHOLD   0.6f   /* rad/s, 俯仰角速度低于此值认为稳定 */
#define STABLE_CONSECUTIVE       40    /* 连续稳定周期数 (40ms) */

/* 腿 PID 临时高增益 (LAUNCH 阶段减少力冲击持续时间) */
#define LAUNCH_KP             10000.0f
#define LAUNCH_KI             50.0f
#define NOMINAL_KP            1200.0f
#define NOMINAL_KI            0.0f

/* ======================== 静态变量 ============================= */

static JumpState_t  g_state    = JUMP_IDLE;
static uint16_t     g_cycle    = 0;
static float        g_approach_speed = 0.0f;  /* jump_start 传入的前向速度 */
static uint8_t      g_jump_remaining = 0;     /* 剩余跳跃次数 */
static uint8_t      g_jump_total     = 0;     /* 初始跳跃总次数, 用于区分首跳 */

/* LAND: 触地时记录的腿 Y 位置, 用于相对缓冲 */
static float g_land_y0 = 0.0f;

/* 离地瞬间捕获的足端 X (含重心补偿), 空中阶段保持 */
static float g_air_x_left  = 0.0f;
static float g_air_x_right = 0.0f;

/* SQUAT 开始时捕获的足端 Y 基线 (左右取均值, 保证两腿同步) */
static float g_base_y = 0.0f;

/* ======================== 辅助函数 ============================= */

static float lerp(float from, float to, float t) {
    if (t <= 0.0f) return from;
    if (t >= 1.0f) return to;
    return from + (to - from) * t;
}

static void enter_state(JumpState_t new_state) {
    g_state = new_state;
    g_cycle = 0;
}

/** 临时拉高腿 PID 增益 (LAUNCH 阶段) */
static void leg_pid_set_launch_gains(void) {
    g_leg_left_pid.front.kp  = LAUNCH_KP;
    g_leg_left_pid.front.ki  = LAUNCH_KI;
    g_leg_left_pid.back.kp   = LAUNCH_KP;
    g_leg_left_pid.back.ki   = LAUNCH_KI;
    g_leg_right_pid.front.kp = LAUNCH_KP;
    g_leg_right_pid.front.ki = LAUNCH_KI;
    g_leg_right_pid.back.kp  = LAUNCH_KP;
    g_leg_right_pid.back.ki  = LAUNCH_KI;
}

/** 恢复腿 PID 标称增益 */
static void leg_pid_restore_nominal_gains(void) {
    g_leg_left_pid.front.kp  = NOMINAL_KP;
    g_leg_left_pid.front.ki  = NOMINAL_KI;
    g_leg_left_pid.back.kp   = NOMINAL_KP;
    g_leg_left_pid.back.ki   = NOMINAL_KI;
    g_leg_right_pid.front.kp = NOMINAL_KP;
    g_leg_right_pid.front.ki = NOMINAL_KI;
    g_leg_right_pid.back.kp  = NOMINAL_KP;
    g_leg_right_pid.back.ki  = NOMINAL_KI;
}

/* ====================== 状态机 ============================= */

/* ── STABILIZE: 自稳等待 (着地) ──
 *   首跳: 纯计时, 等车身稳定 (1.5s)。
 *   连跳: 最小时间 + 车身稳定判据 (gyro_pitch 连续低于阈值),
 *        确保落地振荡衰减后再下蹲起跳。
 *   正常控制运行, 不做任何叠加。
 */
static void run_stabilize(const Sensor_data_t *sensor,
                          Foot_position_t *left, Foot_position_t *right)
{
    (void)left;
    (void)right;

    bool is_first = (g_jump_remaining == g_jump_total);

    if (is_first) {
        /* 首跳: 纯计时, 平地长时间自稳 */
        if (g_cycle >= STABILIZE_DURATION) {
            enter_state(JUMP_SQUAT);
        }
        return;
    }

    /* ── 连跳: 最小时间 + 稳定判据 ── */
    if (g_cycle < MULTI_STABILIZE_DURATION) {
        return;  /* 未到最小时间, 继续等 */
    }

    /* 超时保护: 即使一直不稳也强制进入下蹲 */
    if (g_cycle >= MULTI_STABILIZE_MAX) {
        enter_state(JUMP_SQUAT);
        return;
    }

    /* 稳定判据: 俯仰角速度连续低于阈值 */
    {
        static uint16_t stable_cnt = 0;

        /* 刚过最小时间时重置计数器, 防上次残留 */
        if (g_cycle == MULTI_STABILIZE_DURATION) {
            stable_cnt = 0;
        }

        if (fabsf(sensor->gyro_pitch) < STABLE_GYRO_THRESHOLD) {
            stable_cnt++;
        } else {
            stable_cnt = 0;
        }

        if (stable_cnt >= STABLE_CONSECUTIVE) {
            stable_cnt = 0;
            enter_state(JUMP_SQUAT);
        }
    }
}

/* ── SQUAT: 下蹲蓄力 (着地, ~400ms) ──
 *   leg_cmd_solve 照常运行 (提供 X 基准).
 *   Y = 基线(含roll补偿) + 收腿偏移, 保留重心不对称.
 *
 *   首跳: 立即捕获基线并开始下蹲.
 *   连跳: 先等 MULTI_SQUAT_PRE_DELAY (预稳定), 再捕获基线并下蹲,
 *        避免落地残余振荡污染基线.
 */
static void run_squat(const Sensor_data_t *sensor,
                      Foot_position_t *left, Foot_position_t *right)
{
    (void)sensor;

    bool is_first = (g_jump_remaining == g_jump_total);

    /* 连跳预稳定期: 不修改足端, 等车身姿态收敛再捕获基线 */
    if (!is_first) {
        if (g_cycle <= MULTI_SQUAT_PRE_DELAY) {
            if (g_cycle == MULTI_SQUAT_PRE_DELAY) {
                g_base_y = (left->y + right->y) * 0.5f;
            }
            return;
        }
    } else {
        /* 首跳: cycle 1 立即捕获基线 */
        if (g_cycle == 1) {
            g_base_y = (left->y + right->y) * 0.5f;
        }
    }

    /* 下蹲运动期 */
    uint16_t squat_elapsed = is_first ? g_cycle : (g_cycle - MULTI_SQUAT_PRE_DELAY);
    float t = (float)squat_elapsed / (float)SQUAT_DURATION;
    if (t > 1.0f) t = 1.0f;
    float dy = lerp(0.0f, JUMP_SQUAT_Y, t);

    left->y  = g_base_y + dy;
    right->y = g_base_y + dy;

    if (squat_elapsed >= SQUAT_DURATION) {
        enter_state(JUMP_LAUNCH);
    }
}

/* ── LAUNCH: 蹬地 (着地, max ~200ms) ──
 *   leg_cmd_solve 照常运行. 平衡照常运行.
 *   叠加: 腿 Y 从 squat 渐变到 PUSH (50ms 渐变, 给平衡环反应时间).
 *        腿 X 加对称前向偏置 JUMP_FORWARD_BIAS.
 *   腿 PID 临时拉高增益.
 *   退出: |accel_z| < 0.3g (离地) 或超时. 离地时复位平衡 PID.
 */
static void run_launch(const Sensor_data_t *sensor,
                       Foot_position_t *left, Foot_position_t *right)
{
    if (g_cycle == 1) {
        leg_pid_set_launch_gains();
    }

    /* 腿 Y: 50ms 渐变, 基线 + 蹬地偏移 */
    float ramp_t = (float)g_cycle / (float)LAUNCH_RAMP_MS;
    if (ramp_t > 1.0f) ramp_t = 1.0f;
    float dy = lerp(JUMP_SQUAT_Y, JUMP_PUSH_Y, ramp_t);

    left->x  += JUMP_FORWARD_BIAS;
    right->x -= JUMP_FORWARD_BIAS;
    left->y  = g_base_y + dy;
    right->y = g_base_y + dy;

    if (g_cycle == 1) {
        g_air_x_left  = left->x;   /* 捕获足端 X (含 FORWARD_BIAS), 空中阶段保持 */
        g_air_x_right = right->x;
    }

    /* 离地检测: 80ms 后启用 */
    bool airborne = false;
    if (g_cycle > 80) {
        airborne = (fabsf(sensor->accel_z) < ACCEL_FREEFALL_THRESHOLD);
    }

    if (airborne || g_cycle >= LAUNCH_TIMEOUT) {
        leg_pid_restore_nominal_gains();
        robot_control_reset_balance_pid();
        robot_control_reset_leg_speed_pid();
        enter_state(JUMP_AIR_ASCEND);
    }
}

/* ── AIR_ASCEND: 上升段 (空中, ~150ms) ──
 *   airborne=true: 轮式平衡关停, leg_cmd_solve 跳过.
 *   设置绝对足端位置: Y 从 PUSH 渐变到 TUCK (收腿过障碍).
 *   X 保持离地瞬间捕获的重心补偿值.
 *   退出: accel_z < APOGEE_THRESHOLD (到抛物线顶点).
 */
static void run_air_ascend(const Sensor_data_t *sensor,
                           Foot_position_t *left, Foot_position_t *right)
{
    float t = (float)g_cycle / (float)TUCK_RAMP_MS;
    if (t > 1.0f) t = 1.0f;
    float y = lerp(JUMP_PUSH_Y, JUMP_TUCK_Y, t);

    left->x  = g_air_x_left;
    right->x = g_air_x_right;
    left->y  = y;
    right->y = y;

    if (sensor->accel_z < APOGEE_THRESHOLD || g_cycle > APOGEE_TIMEOUT_MS) {
        enter_state(JUMP_AIR_DESCEND);
    }
}

/* ── AIR_DESCEND: 下降段 (空中, max ~500ms) ──
 *   airborne=true: 轮式平衡关停, leg_cmd_solve 跳过.
 *   设置绝对足端位置: Y 从 TUCK 渐变到 REACH (伸腿够地).
 *   X 保持离地瞬间捕获的重心补偿值.
 *   退出: accel_z > 1.2g (触地冲击) 或超时.
 *   有 15ms 冻结期防误触发.
 */
static void run_air_descend(const Sensor_data_t *sensor,
                            Foot_position_t *left, Foot_position_t *right)
{
    float y;

    if (g_cycle < (uint16_t)REACH_RAMP_MS) {
        float t = (float)g_cycle / (float)REACH_RAMP_MS;
        y = lerp(JUMP_TUCK_Y, JUMP_REACH_Y, t);
    } else {
        y = JUMP_REACH_Y;
    }

    left->x  = g_air_x_left;
    right->x = g_air_x_right;
    left->y  = y;
    right->y = y;

    bool impact = (sensor->accel_z > IMPACT_ACCEL)
                  && (g_cycle > (uint16_t)(REACH_RAMP_MS + FREEZE_MS));

    if (impact || g_cycle > FLY_TIMEOUT_CYCLES) {
        g_land_y0 = y;  /* 记录触地时的腿 Y */
        robot_control_reset_leg_speed_pid();  /* 清除空中阶段残留, leg_cmd_solve 即将恢复 */
        enter_state(JUMP_LAND);
    }
}

/* LAND: 缓冲着地 (~630ms) ──
 *   airborne=false: 轮式平衡和 leg_cmd_solve 恢复运行.
 *   首周期将 g_land_y0 从绝对触地 Y 转换为相对 leg_cmd_solve 的偏移,
 *   后续作为 delta 叠加到 leg_cmd_solve 输出上, 释放到 0 时平滑交还.
 *   子阶段 A (80ms):  delta 从 y0 渐变到 y0 + CUSHION_DELTA (收腿缓冲).
 *   子阶段 B (200ms): 保持缓冲位.
 *   子阶段 C (400ms): delta 渐变回 0 (leg_cmd_solve 完全接管).
 *   X 来自 leg_cmd_solve 的速度环输出 (自动刹车).
 */
static void run_land(const Sensor_data_t *sensor,
                     Foot_position_t *left, Foot_position_t *right)
{
    (void)sensor;
    float delta;

    /* 首周期: 将触地绝对 Y 转换为相对 leg_cmd_solve 的偏移量 */
    if (g_cycle == 1) {
        g_land_y0 = g_land_y0 - (left->y + right->y) * 0.5f;
    }

    if (g_cycle <= CUSHION_RAMP_MS) {
        /* A: 从触地偏移渐变收腿缓冲 */
        float t = (float)g_cycle / (float)CUSHION_RAMP_MS;
        delta = lerp(g_land_y0, g_land_y0 + JUMP_CUSHION_DELTA, t);
    } else if (g_cycle <= CUSHION_RAMP_MS + CUSHION_HOLD_MS) {
        /* B: 保持缓冲位 */
        delta = g_land_y0 + JUMP_CUSHION_DELTA;
    } else {
        /* C: 渐变回 0, 完全交还 leg_cmd_solve */
        uint16_t rel = g_cycle - CUSHION_RAMP_MS - CUSHION_HOLD_MS;
        float t = (float)rel / (float)CUSHION_RELEASE_MS;
        if (t > 1.0f) t = 1.0f;
        delta = lerp(g_land_y0 + JUMP_CUSHION_DELTA, 0.0f, t);
    }

    left->y  += delta;
    right->y += delta;

    if (g_cycle >= CUSHION_RAMP_MS + CUSHION_HOLD_MS + CUSHION_RELEASE_MS) {
        if (g_jump_remaining > 1) {
            /* 还有剩余跳跃: 回到 STABILIZE 走完整流程, 保证每跳一致 */
            g_jump_remaining--;
            enter_state(JUMP_STABILIZE);
        } else {
            g_jump_remaining = 0;
            g_jump_total     = 0;
            enter_state(JUMP_IDLE);
        }
    }
}

/* ======================== 公开接口 ============================= */

void jump_init(void) {
    g_state   = JUMP_IDLE;
    g_cycle   = 0;
    g_land_y0 = 0.0f;
    g_jump_remaining = 0;
    g_jump_total     = 0;
}

void jump_start(float target_speed, uint8_t count) {
    if (g_state == JUMP_IDLE && count > 0) {
        g_approach_speed = target_speed;
        g_jump_remaining = count;
        g_jump_total     = count;
        enter_state(JUMP_STABILIZE);
    }
}

void jump_stop(void) {
    if (g_state != JUMP_IDLE) {
        leg_pid_restore_nominal_gains();
        robot_control_reset_balance_pid();
        robot_control_reset_leg_speed_pid();
        g_jump_remaining = 0;
        g_jump_total     = 0;
        enter_state(JUMP_IDLE);
    }
}

bool jump_is_active(void) {
    return (g_state != JUMP_IDLE);
}

bool jump_is_airborne(void) {
    return (g_state == JUMP_AIR_ASCEND || g_state == JUMP_AIR_DESCEND);
}

bool jump_is_launching(void) {
    return (g_state == JUMP_LAUNCH);
}

bool jump_is_stabilizing(void) {
    return (g_state == JUMP_STABILIZE);
}

bool jump_is_squatting(void) {
    return (g_state == JUMP_SQUAT);
}

bool jump_is_landing_cushion(void) {
    /* LAND 阶段的收腿缓冲 (A) + 保持 (B) 期间, 轮子应锁死防前冲.
     * 释放阶段 (C) 起平衡控制恢复. */
    return (g_state == JUMP_LAND)
        && (g_cycle <= CUSHION_RAMP_MS + CUSHION_HOLD_MS);
}

uint8_t jump_get_remaining(void) {
    return g_jump_remaining;
}

float jump_get_approach_speed(void) {
    return g_approach_speed;
}

void jump_leg_overlay(Foot_position_t *left, Foot_position_t *right,
                      const Sensor_data_t *sensor) {
    g_cycle++;

    switch (g_state) {
    case JUMP_STABILIZE:   run_stabilize(sensor, left, right);   break;
    case JUMP_SQUAT:       run_squat(sensor, left, right);       break;
    case JUMP_LAUNCH:      run_launch(sensor, left, right);      break;
    case JUMP_AIR_ASCEND:  run_air_ascend(sensor, left, right);  break;
    case JUMP_AIR_DESCEND: run_air_descend(sensor, left, right); break;
    case JUMP_LAND:        run_land(sensor, left, right);        break;
    case JUMP_IDLE:
    default:
        break;
    }
}
