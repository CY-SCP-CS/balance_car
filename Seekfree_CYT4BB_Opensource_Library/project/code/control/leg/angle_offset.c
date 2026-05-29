#include "angle_offset.h"
#include <string.h>

const AngleOffset_Config_t g_angle_offset_default_cfg = {
    .homing_pwm           = 600,         /* 标定时撞限位用的占空比 (20%) */
    .stall_cycles         = 80,           /* 判定堵转的时长 (周期数)       */
    .settle_cycles        = 100,          /* 每个关节开始前等待 100ms      */
    .timeout_cycles       = 30000,        /* 整体超时 30s                 */
    .stall_threshold      = 0.003f,       /* 每周期角度变化 < 0.003 rad 判堵（≈0.17°/周期） */
    .dir                  = { 1, -1, -1, -1 }, /* 撞限位方向 (+1/-1)       */
};

/* ─── 内部状态 ───────────────────────────────────────────────────── */

typedef enum {
    CALIB_STATE_IDLE,
    CALIB_STATE_LF,
    CALIB_STATE_LB,
    CALIB_STATE_RF,
    CALIB_STATE_RB,
    CALIB_STATE_DONE
} CalibState_t;

#define STATE_TO_JOINT(state)  ((JointID_t)((state) - CALIB_STATE_LF))

typedef enum {
    SUB_SETTLE,         /* 等待编码器数据稳定                        */
    SUB_MOVING,         /* 朝限位运动，等待角度变化确认电机已动       */
    SUB_STALL_CHECK     /* 监测堵转（顶到限位）                      */
} CalibSubState_t;

static struct
{
    CalibState_t          state;            /* 标定状态 */
    CalibState_t          last_state;       /* 最后一个要标定的状态 */
    CalibSubState_t       sub_state;        /* 标定子状态 */
    AngleOffset_Config_t  cfg;
    float                 offset_deg[JOINT_COUNT];  /* 各关节零位偏移 */
    bool                  all_done;         /* 全部完成 */
    bool                  fault;            /* 超时出错 */
    int                   settle_counter;   /* 等待计数器 */
    int                   stall_counter;    /* 堵转计数器 */
    int                   timeout_counter;  /* 超时计数器 */
    float                 prev_angle;       /* 上一次的角度值 */
} g_calib;


static float get_joint_angle(const Sensor_data_t *sensor, JointID_t joint)
{
    switch (joint) {
    case JOINT_LEFT_FRONT:  return sensor->joint_left_front_angle;
    case JOINT_LEFT_BACK:   return sensor->joint_left_back_angle;
    case JOINT_RIGHT_FRONT: return sensor->joint_right_front_angle;
    case JOINT_RIGHT_BACK:  return sensor->joint_right_back_angle;
    default:                return 0.0f;
    }
}

static void set_joint_pwm(Motor_cmd_duty_t *motor_cmd, JointID_t joint, int pwm)
{
    switch (joint) {
    case JOINT_LEFT_FRONT:  motor_cmd->left_front_joint_pwm  = pwm; break;
    case JOINT_LEFT_BACK:   motor_cmd->left_back_joint_pwm   = pwm; break;
    case JOINT_RIGHT_FRONT: motor_cmd->right_front_joint_pwm = pwm; break;
    case JOINT_RIGHT_BACK:  motor_cmd->right_back_joint_pwm  = pwm; break;
    default:                                                       break;
    }
}

static void clear_all_pwm(Motor_cmd_duty_t *m)
{
    m->left_front_joint_pwm = 0;
    m->left_back_joint_pwm  = 0;
    m->right_front_joint_pwm = 0;
    m->right_back_joint_pwm  = 0;
}

static void apply_homing_pwm(Motor_cmd_duty_t *motor_cmd, JointID_t joint)
{
    int dir = g_calib.cfg.dir[joint];
    if (dir == 0) dir = 1;
    set_joint_pwm(motor_cmd, joint, (int)g_calib.cfg.homing_pwm * dir);
}

static inline float abs_delta(float curr, float prev)
{
    float d = curr - prev;
    return (d < 0.0f) ? -d : d;
}

static void enter_new_joint(void)
{
    g_calib.sub_state      = SUB_SETTLE;
    g_calib.settle_counter = 0;
    g_calib.stall_counter  = 0;
    g_calib.prev_angle     = 0.0f;
}

static void advance_state(Motor_cmd_duty_t *motor_cmd)
{
    set_joint_pwm(motor_cmd, STATE_TO_JOINT(g_calib.state), 0);

    g_calib.state = (CalibState_t)((int)g_calib.state + 1);

    if (g_calib.state > g_calib.last_state) {
        g_calib.state = CALIB_STATE_DONE;
        g_calib.all_done = true;
    } else {
        enter_new_joint();
    }
}

/* ─── 公开 API ───────────────────────────────────────────────────── */

void angle_offset_start(const AngleOffset_Config_t *cfg)
{
    memset(&g_calib, 0, sizeof(g_calib));
    g_calib.state = CALIB_STATE_IDLE;

    g_calib.cfg = cfg ? *cfg : g_angle_offset_default_cfg;

    /* 参数钳位 */
    if (g_calib.cfg.homing_pwm < 0)
        g_calib.cfg.homing_pwm = -g_calib.cfg.homing_pwm;
    if (g_calib.cfg.homing_pwm > 10000)
        g_calib.cfg.homing_pwm = 10000;
    if (g_calib.cfg.stall_cycles == 0)
        g_calib.cfg.stall_cycles = 1;
    if (g_calib.cfg.stall_threshold <= 0.0f)
        g_calib.cfg.stall_threshold = 0.003f;

    g_calib.last_state = CALIB_STATE_RB;
    g_calib.state = CALIB_STATE_LF;
    enter_new_joint();
}

void angle_offset_start_leg(LegSide_t leg, const AngleOffset_Config_t *cfg)
{
    memset(&g_calib, 0, sizeof(g_calib));
    g_calib.state = CALIB_STATE_IDLE;

    g_calib.cfg = cfg ? *cfg : g_angle_offset_default_cfg;

    /* 参数钳位 */
    if (g_calib.cfg.homing_pwm < 0)
        g_calib.cfg.homing_pwm = -g_calib.cfg.homing_pwm;
    if (g_calib.cfg.homing_pwm > 10000)
        g_calib.cfg.homing_pwm = 10000;
    if (g_calib.cfg.stall_cycles == 0)
        g_calib.cfg.stall_cycles = 1;
    if (g_calib.cfg.stall_threshold <= 0.0f)
        g_calib.cfg.stall_threshold = 0.003f;

    if (leg == LEG_LEFT) {
        g_calib.state      = CALIB_STATE_LF;
        g_calib.last_state = CALIB_STATE_LB;
    } else {
        g_calib.state      = CALIB_STATE_RF;
        g_calib.last_state = CALIB_STATE_RB;
    }
    enter_new_joint();
}

void angle_offset_process(const Sensor_data_t *sensor, Motor_cmd_duty_t *motor_cmd)
{
    if (g_calib.state < CALIB_STATE_LF || g_calib.state >= CALIB_STATE_DONE)
        return;

    /* ── 整体超时 ── */
    g_calib.timeout_counter++;
    if (g_calib.cfg.timeout_cycles > 0
        && g_calib.timeout_counter >= (int)g_calib.cfg.timeout_cycles) {
        g_calib.fault = true;
        g_calib.state = CALIB_STATE_DONE;
        clear_all_pwm(motor_cmd);
        return;
    }

    JointID_t current_joint = STATE_TO_JOINT(g_calib.state);
    float     current_angle = get_joint_angle(sensor, current_joint);

    switch (g_calib.sub_state) {

    case SUB_SETTLE:
        g_calib.prev_angle = current_angle;
        g_calib.settle_counter++;
        clear_all_pwm(motor_cmd);
        if (g_calib.settle_counter >= (int)g_calib.cfg.settle_cycles)
            g_calib.sub_state = SUB_MOVING;
        break;

    case SUB_MOVING: {
        float delta = abs_delta(current_angle, g_calib.prev_angle);

        if (delta >= g_calib.cfg.stall_threshold) {
            g_calib.sub_state     = SUB_STALL_CHECK;
            g_calib.stall_counter = 0;
        } else if (++g_calib.stall_counter > 3000) {
            /* 3 秒还没动，记录当前位置并跳过 */
            g_calib.offset_deg[current_joint] = current_angle;
            advance_state(motor_cmd);
            return;
        }

        g_calib.prev_angle = current_angle;
        apply_homing_pwm(motor_cmd, current_joint);
        break;
    }

    case SUB_STALL_CHECK: {
        float delta = abs_delta(current_angle, g_calib.prev_angle);

        if (delta < g_calib.cfg.stall_threshold)
            g_calib.stall_counter++;
        else
            g_calib.stall_counter = 0;

        g_calib.prev_angle = current_angle;

        if (g_calib.stall_counter >= (int)g_calib.cfg.stall_cycles) {
            g_calib.offset_deg[current_joint] = current_angle;
            advance_state(motor_cmd);
            return;
        }

        apply_homing_pwm(motor_cmd, current_joint);
        break;
    }
    }
}

bool angle_offset_is_done(void)
{
    return g_calib.all_done;
}

bool angle_offset_has_fault(void)
{
    return g_calib.fault;
}

void angle_offset_apply_to_sensor(Sensor_data_t *sensor)
{
    sensor->joint_left_front_angle  -= g_calib.offset_deg[JOINT_LEFT_FRONT];
    sensor->joint_left_back_angle   -= g_calib.offset_deg[JOINT_LEFT_BACK];
    sensor->joint_right_front_angle -= g_calib.offset_deg[JOINT_RIGHT_FRONT];
    sensor->joint_right_back_angle  -= g_calib.offset_deg[JOINT_RIGHT_BACK];
}
