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
#define TAKEOFF_CYCLES       100     /* 100 ms 蹬地伸展 */
#define ASCEND_CYCLES        120     /* 120 ms 空中收腿 */
#define DESCEND_CYCLES       300     /* 300 ms 伸腿准备接地 */
#define IMPACT_CYCLES        200     /* 200 ms 触地缓冲 */
#define INTERVAL_CYCLES      10000   /* 10 s 跳跃间隔 */
#define LAND_CYCLES          400     /* 400 ms 缓冲恢复 */

/* 足端位置偏移量 (mm, 相对标称位形)
 *   坐标系: +X 向前, +Y 向下
 *   Y < 0 : 足端上抬 ⇔ 收腿 ⇔ 重心降低 (下蹲 / 空中收腿)
 *   Y > 0 : 足端下压 ⇔ 伸腿 ⇔ 重心升高 (蹬地)
 */
#define OFF_PUSH_X      0.0f
#define OFF_PUSH_Y      -400.0f      /* 蹬地伸展 (故意超出关节极限, 保持 PID 饱和输出) */

#define OFF_SQUAT_X     0.0f
#define OFF_SQUAT_Y     120.0f       /* 下蹲收腿 */

#define OFF_TUCK_X      0.0f
#define OFF_TUCK_Y      85.0f        /* 空中紧收 */

#define OFF_REACH_X     0.0f
#define OFF_REACH_Y     -70.0f       /* 伸腿找地 */

#define OFF_IMPACT_X    0.0f
#define OFF_IMPACT_Y    30.0f        /* 收腿缓冲 */

#define OFF_ABSORB_X    0.0f
#define OFF_ABSORB_Y    50.0f        /* 着陆缓冲 */

#define IMPACT_GYRO_DELTA_THRESHOLD  2.0f  /* 陀螺仪触地检测阈值 (rad/s) */

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

/** 陀螺仪触地检测: 检测俯仰角速度突变 */
static bool detect_impact(const Sensor_data_t *sensor) {
    static float prev_gyro = 0.0f;
    static bool  first_call = true;
    if (first_call) {
        prev_gyro = sensor->gyro_pitch;
        first_call = false;
        return false;
    }
    float delta = sensor->gyro_pitch - prev_gyro;
    prev_gyro = sensor->gyro_pitch;
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
    /* 下蹲收腿, 保持平衡 + 速度环 */
    Foot_position_t left  = { OFF_SQUAT_X, OFF_SQUAT_Y };
    Foot_position_t right = { OFF_SQUAT_X, OFF_SQUAT_Y };

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

    robot_control_leg_speed_feedback(sensor, &left, &right);
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
    /* 快速伸腿蹬地, 保持平衡 + 速度环 (轮子着地) */
    Foot_position_t left  = { OFF_PUSH_X, OFF_PUSH_Y };
    Foot_position_t right = { OFF_PUSH_X, OFF_PUSH_Y };

    if (g_cycle == 1) {
        g_leg_left_pid.front.kp  = 10000.0f;
        g_leg_left_pid.back.kp   = 10000.0f;
        g_leg_right_pid.front.kp = 10000.0f;
        g_leg_right_pid.back.kp  = 10000.0f;
        pid_reset(&g_leg_left_pid.front);
        pid_reset(&g_leg_left_pid.back);
        pid_reset(&g_leg_right_pid.front);
        pid_reset(&g_leg_right_pid.back);
    }

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

    robot_control_leg_speed_feedback(sensor, &left, &right);
    leg_offset_to_joint_target(LEG_LEFT,  &left,  &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &right, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    if (g_cycle >= TAKEOFF_CYCLES) {
        /* 恢复正常 kp, 进入空中前复位平衡 PID */
        g_leg_left_pid.front.kp  = 1200.0f;
        g_leg_left_pid.back.kp   = 1200.0f;
        g_leg_right_pid.front.kp = 1200.0f;
        g_leg_right_pid.back.kp  = 1200.0f;
        reset_balance();
        enter_state(JUMP_ASCEND);
    }
}

static void run_ascend(const Sensor_data_t *sensor,
                       Motor_cmd_duty_t *motor_cmd)
{
    /* 空中收腿: 平衡线性衰减到零 */
    Foot_position_t off = { OFF_TUCK_X, OFF_TUCK_Y };

    float weight = 1.0f - (float)g_cycle / (float)ASCEND_CYCLES;
    if (weight < 0.0f) weight = 0.0f;
    balance_with_yaw_scaled(sensor, motor_cmd, weight);

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
    /* 伸腿准备接地, 平衡关闭 (空中不需要平衡) */
    Foot_position_t off = { OFF_REACH_X, OFF_REACH_Y };

    if (g_cycle == 1) {
        pid_reset(&g_leg_left_pid.front);
        pid_reset(&g_leg_left_pid.back);
        pid_reset(&g_leg_right_pid.front);
        pid_reset(&g_leg_right_pid.back);
    }

    balance_with_yaw_scaled(sensor, motor_cmd, 0.0f);

    leg_offset_to_joint_target(LEG_LEFT,  &off, &g_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &off, &g_target_right);
    run_leg_pid(sensor, motor_cmd);

    /* 陀螺仪触地检测 或 超时 */
    if (detect_impact(sensor) || g_cycle >= DESCEND_CYCLES) {
        enter_state(JUMP_IMPACT);
    }
}

static void run_impact(const Sensor_data_t *sensor,
                       Motor_cmd_duty_t *motor_cmd)
{
    /* 深收腿吸收冲击: 足端 +30mm, 平衡全开 */
    Foot_position_t left  = { OFF_IMPACT_X, OFF_IMPACT_Y };
    Foot_position_t right = { OFF_IMPACT_X, OFF_IMPACT_Y };

    if (g_cycle == 1) {
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

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

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
     * 着陆恢复: 足端从收腿位(+30mm) 线性过渡到标称位(0,0)
     * 平衡全开
     */
    const float ratio = (float)g_cycle / (float)LAND_CYCLES;
    const float t = (ratio > 1.0f) ? 1.0f : ratio;
    float x = OFF_IMPACT_X * (1.0f - t);
    float y = OFF_IMPACT_Y * (1.0f - t);

    Foot_position_t left  = { x, y };
    Foot_position_t right = { x, y };

    if (g_cycle == 1) {
        pid_reset(&g_leg_left_pid.front);
        pid_reset(&g_leg_left_pid.back);
        pid_reset(&g_leg_right_pid.front);
        pid_reset(&g_leg_right_pid.back);
    }

    balance_with_yaw_scaled(sensor, motor_cmd, 1.0f);

    robot_control_leg_speed_feedback(sensor, &left, &right);
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

void jump_start(void) {
    if (g_state == JUMP_IDLE) {
        g_jump_num = 0;
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
