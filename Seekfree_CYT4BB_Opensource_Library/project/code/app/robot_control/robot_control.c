#include "robot_control.h"
#include "small_driver_uart_control.h"
#include "jump.h"
#include "../../lib/pid/pid_calculate.h"
#include "../../common/types.h"
#include "../../hmi/ui/page_remote.h"
#include "../navigation/nav_engine.h"
#include "../../control/leg/angle_offset.h"
#include "../../control/leg/leg_pid_control.h"
#include "../../control/leg/leg_vmc_control.h"
#include "../../control/leg/kinematics.h"
#include "../../control/leg/jacobian.h"
#include "../../control/balance/pitch_balance.h"
#include <math.h>

static Foot_position_t foot_position_left;

static Foot_position_t foot_position_right;

/* ─── 控制模式选择 ───
 *  USE_VMC = 0 : 当前 PID 方案 (默认)
 *  USE_VMC = 1 : 修复后的 VMC 方案 (需现场调参)
 */
#define USE_VMC 0

/* 腿部关节 PID 控制器 */
static Leg_PID_t g_leg_left_pid, g_leg_right_pid;

/* 腿部关节目标角度 (相对限位的偏移量, rad) */
static Leg_Target_t g_leg_target_left, g_leg_target_right;

#if USE_VMC
static VMC_Config_t g_vmc_config;
#endif

PID_Controller_t g_pitch_angle_pid, g_pitch_gyro_pid, g_speed_pid;
PID_Controller_t g_yaw_angle_pid, g_yaw_pid;
static PID_Controller_t g_leg_speed_pid, g_leg_roll_pid;


void robot_control_init(void){
    //pitch的PID
    pid_init(&g_pitch_angle_pid, 25.0f, 0.0f, 0.0f, ROBOT_CONTROL_DT, 270.0f, 0.0f);
    pid_init(&g_pitch_gyro_pid,  -1100.0f, 0.0f, 3.00f, ROBOT_CONTROL_DT, 10000.0f, 3000.0f);
    pid_init(&g_speed_pid,       -0.0f, 0.0f, 0.00f, ROBOT_CONTROL_DT, 2000.0f, 500.0f);
    //偏航串级: 外环角度, 内环角速度
    pid_init(&g_yaw_angle_pid,    5.0f, 0.5f, 0.0f, ROBOT_CONTROL_DT, MAX_YAW_RATE, 1.0f);
    pid_init(&g_yaw_pid,       2150.0f, 10.0f, 0.0f, ROBOT_CONTROL_DT, 10000.0f, 2000.0f);
    //针对腿位置的PID，需要在完整的车上调试
    pid_init(&g_leg_speed_pid, 450.0f, 0.8f, 0.5f, ROBOT_CONTROL_DT, 60.0f, 10.0f);
    pid_init(&g_leg_roll_pid,  -20.0f, 0.0f, 0.0f, ROBOT_CONTROL_DT, 1.0f, 0.5f);

    //关节角度的PID，需要在完整的车上调试
    leg_pid_init(&g_leg_left_pid,  1200.0f, 0.0f, 4.0f, 10000.0f, 0.0f);
    leg_pid_init(&g_leg_right_pid, 1200.0f, 0.0f, 4.0f, 10000.0f, 0.0f);

#if USE_VMC
    g_vmc_config.kp = 0.25f;
    g_vmc_config.kd = 0.0000f;
#endif

    //初始偏移，根据实际情况改一下重心
    g_leg_target_left.front  = 0.4f;
    g_leg_target_left.back   = -0.4f;
    g_leg_target_right.front = 0.4f;
    g_leg_target_right.back  = -0.4f;

    remote_param_bind(0, &g_pitch_angle_pid.kp);
    remote_param_bind(1, &g_pitch_gyro_pid.kp);
    remote_param_bind(2, &g_pitch_gyro_pid.kd);
    remote_param_bind(3, &g_speed_pid.kp);
    remote_param_bind(4, &g_leg_left_pid.front.kp);
    remote_param_bind(5, &g_leg_roll_pid.kp);
    remote_param_bind(6, &g_yaw_pid.kp);
    remote_param_bind(7, &g_yaw_pid.kd);

#if USE_VMC
    remote_param_bind(6, &g_vmc_config.kp);
    remote_param_bind(7, &g_vmc_config.kd);
#endif

    jump_init();

}

/*---------------------------------------------------------------------------*/
void sensor_update(const Sensor_data_t *sensor){
    g_sensor_data = *sensor;
}

/*---------------------------------------------------------------------------*/
void command_update(const Move_cmd_t *cmd){
    g_move_cmd = *cmd;
}

/*---------------------------------------------------------------------------*/
void robot_control_reset_balance_pid(void){
    pid_reset(&g_pitch_angle_pid);
    pid_reset(&g_pitch_gyro_pid);
    pid_reset(&g_speed_pid);
    pitch_balance_reset_statics();
}

/*---------------------------------------------------------------------------*/
/** 腿位置速度环反馈: 根据轮速调整足端 X, 根据横滚调整足端 Y */
void robot_control_leg_speed_feedback(const Sensor_data_t *sensor,
    Foot_position_t *left, Foot_position_t *right)
{
    Move_cmd_t dummy = { .target_speed = 0.0f, .target_roll = 0.0f, .target_height = 0.0f };
    Foot_position_t fb_left, fb_right;
    leg_cmd_solve(&dummy, sensor, &g_leg_speed_pid, &g_leg_roll_pid,
                  &fb_left, &fb_right);
    left->x  += fb_left.x;
    left->y  += fb_left.y;
    right->x += fb_right.x;
    right->y += fb_right.y;
}

/*---------------------------------------------------------------------------*/
/** 将足端偏移通过雅可比逆解转换为关节目标角度 */
void leg_offset_to_joint_target(LegSide_t side,
    const Foot_position_t *foot_pos, Leg_Target_t *target)
{
    float nom_front, nom_back, sign_front, sign_back;

    if (side == LEG_LEFT) {
        nom_front  = LEG_NOM_FRONT_L;
        nom_back   = LEG_NOM_BACK_L;
        sign_front = 1.0f;
        sign_back  = 1.0f;
    } else {
        nom_front  = LEG_NOM_FRONT_R;
        nom_back   = LEG_NOM_BACK_R;
        sign_front = RIGHT_ABS_FRONT_SIGN;
        sign_back  = RIGHT_ABS_BACK_SIGN;
    }

    Joint_angle_t abs_nominal;
    float J[2][2], dth1, dth2;

    abs_nominal.left_motor_angle  = M_PI_2 + sign_front * nom_front;
    abs_nominal.right_motor_angle = M_PI_2 + sign_back  * nom_back;

    if (five_bar_jacobian(&abs_nominal, J) == 0 &&
        five_bar_jacobian_solve(J, &dth1, &dth2,
            foot_pos->x, foot_pos->y) == 0) {
        target->front = nom_front + sign_front * dth1;
        target->back  = nom_back  + sign_back  * dth2;
    } else {
        /* 奇异 → 保持标称 */
        target->front = nom_front;
        target->back  = nom_back;
    }

    /* 关节限位: 不发出超过180°的目标 */
    target->front = CLAMP(target->front, -SAFE_JOINT_MAX_RAD, SAFE_JOINT_MAX_RAD);
    target->back  = CLAMP(target->back,  -SAFE_JOINT_MAX_RAD, SAFE_JOINT_MAX_RAD);
}

/*---------------------------------------------------------------------------*/
/** 安全检查: 倾角/关节角度超限时切断所有电机输出, 返回 false
 *  故障会锁存, 一旦触发则后续所有周期永久停机, 需复位重启.
 */
static bool g_safety_fault = false;

static bool safety_check(const Sensor_data_t *sensor, Motor_cmd_duty_t *motor_cmd)
{
    /* ── 倾角保护 ── */
    float pitch_deg = sensor->angle_pitch * RAD_TO_DEG;
    float roll_deg  = sensor->angle_roll  * RAD_TO_DEG;

    if (g_safety_fault) {
        goto fault;
    }

    if (fabsf(pitch_deg) > SAFE_PITCH_MAX_DEG ||
        fabsf(roll_deg)  > SAFE_ROLL_MAX_DEG) {
        g_safety_fault = true;
        goto fault;
    }

    /* ── 关节角度保护 (相对限位的转角) ── */
    if (fabsf(sensor->joint_left_front_angle)  > SAFE_JOINT_MAX_RAD ||
        fabsf(sensor->joint_left_back_angle)   > SAFE_JOINT_MAX_RAD ||
        fabsf(sensor->joint_right_front_angle) > SAFE_JOINT_MAX_RAD ||
        fabsf(sensor->joint_right_back_angle)  > SAFE_JOINT_MAX_RAD) {
        g_safety_fault = true;
        goto fault;
    }

    return true;

fault:
    motor_cmd->left_motor_pwm       = 0;
    motor_cmd->right_motor_pwm      = 0;
    motor_cmd->left_front_joint_pwm  = 0;
    motor_cmd->left_back_joint_pwm   = 0;
    motor_cmd->right_front_joint_pwm = 0;
    motor_cmd->right_back_joint_pwm  = 0;
    return false;
}

/*---------------------------------------------------------------------------*/
void control_task(void){
    Sensor_data_t sensor_local = g_sensor_data;
    Move_cmd_t     cmd_local   = g_move_cmd;

    if (!safety_check(&sensor_local, &g_motor_cmd)) {
      
        return;
    }

    if (jump_is_active()) {
        jump_control(&sensor_local, &g_motor_cmd);
        return;
    }

    //pitch平衡环
    pitch_balance_control(&sensor_local, &g_speed_pid, &g_pitch_angle_pid,
        &g_pitch_gyro_pid, &g_motor_cmd);

    //差速转向: 串级控制 (外环角度, 内环角速度)
    {
        static float target_yaw_angle = 0.0f;
        static bool  yaw_init = true;
        static float prev_target_roll = 0.0f;  /* 检测 roll 指令跳变, 复位 yaw PID 积分 */

        if (yaw_init) {
            target_yaw_angle = sensor_local.angle_yaw;
            yaw_init = false;
        }

        float target_yaw_rate = cmd_local.target_roll * MAX_YAW_RATE;
        target_yaw_angle += target_yaw_rate * ROBOT_CONTROL_DT;

        /* 角度归一化到 [-PI, PI], 防止多次旋转后 target 与 IMU yaw 跨周期累积 */
        while (target_yaw_angle >  M_PI) target_yaw_angle -= 2.0f * M_PI;
        while (target_yaw_angle < -M_PI) target_yaw_angle += 2.0f * M_PI;

        /* roll 指令跳变时复位 yaw PID, 防止积分饱和导致乱转 */
        if (cmd_local.target_roll != prev_target_roll) {
            pid_reset(&g_yaw_angle_pid);
            pid_reset(&g_yaw_pid);
            prev_target_roll = cmd_local.target_roll;
        }

        /* 外环: 角度误差 → 角速度修正 (IMU绝对偏航, 无漂移) */
        float rate_correction = pid_calculate(&g_yaw_angle_pid, target_yaw_angle, sensor_local.angle_yaw);

        /* 内环: 前馈角速度 + 角度修正 → 差模 PWM */
        float yaw_out = pid_calculate(&g_yaw_pid, target_yaw_rate + rate_correction, -sensor_local.gyro_yaw);
        int   yaw_pwm = ROUND(yaw_out);
        g_motor_cmd.left_motor_pwm  -= yaw_pwm;
        g_motor_cmd.right_motor_pwm -= yaw_pwm;
    }

    //计算目标足端位置
    leg_cmd_solve(&cmd_local, &sensor_local, &g_leg_speed_pid, &g_leg_roll_pid,
        &foot_position_left, &foot_position_right);

    //腿的控制
#if USE_VMC
    /* ── VMC 方案（修复了角度偏置/符号/目标计算） ── */
    leg_vmc_control(&g_vmc_config, &sensor_local,
                    &foot_position_left, &foot_position_right,
                    &g_motor_cmd);
#else
    leg_offset_to_joint_target(LEG_LEFT,  &foot_position_left,  &g_leg_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &foot_position_right, &g_leg_target_right);

    leg_pid_control(&g_leg_left_pid, &g_leg_target_left,
                    &sensor_local, LEG_LEFT, &g_motor_cmd);

    leg_pid_control(&g_leg_right_pid, &g_leg_target_right,
                    &sensor_local, LEG_RIGHT, &g_motor_cmd);
#endif
}

/*---------------------------------------------------------------------------*/
/* 画方测试: 前进 1 秒 → 右转 90° → 重复 8 次 → 停止 */
#define SQUARE_FWD_MS      1000    /* 前进时长 (ms) */
#define SQUARE_TURN_MS     1000    /* 右转 90° 时长 (ms) */
#define SQUARE_FWD_SPEED   0.3f    /* 前进速度 */
#define SQUARE_TURN_ROLL  -0.524f  /* 右转 roll: -π/2 / MAX_YAW_RATE ≈ -0.524 */
#define SQUARE_COUNT       8       /* 重复次数 */

static void square_test(Move_cmd_t *cmd) {
    static uint8_t  side   = 0;   /* 当前第几边 (0~7, 共8边) */
    static uint8_t  phase  = 0;   /* 0=前进, 1=转弯 */
    static uint32_t timer  = 0;   /* 当前相位已过周期数 */

    if (side >= SQUARE_COUNT) {
        cmd->target_speed  = 0.0f;
        cmd->target_roll   = 0.0f;
        return;
    }

    if (phase == 0) {
        /* 前进 */
        cmd->target_speed  = SQUARE_FWD_SPEED;
        cmd->target_roll   = 0.0f;
        if (timer >= SQUARE_FWD_MS) {
            phase = 1;
            timer = 0;
        }
    } else {
        /* 右转 */
        cmd->target_speed  = 0.0f;
        cmd->target_roll   = SQUARE_TURN_ROLL;
        if (timer >= SQUARE_TURN_MS) {
            phase = 0;
            side++;
            timer = 0;
        }
    }
    timer++;
}

/*---------------------------------------------------------------------------*/
void sensor_cmd_update(const Ctrl_Input_t *ctrl, Sensor_data_t *sensor, Move_cmd_t *cmd){

    /* --- 传感器数据桥接 --- */
    sensor->angle_pitch    = ctrl->body_pitch;
    sensor->angle_roll     = ctrl->body_roll;
    sensor->angle_yaw      = ctrl->body_yaw;
    sensor->gyro_pitch     = ctrl->gyro_pitch_rate;
    sensor->gyro_yaw       = ctrl->gyro_yaw_rate;
    sensor->gyro_roll      = ctrl->gyro_roll_rate;
    sensor->accel_z        = 0.0f;
    //驱动轮速度 (UART4) 和关节角度 (UART6 左腿, UART3 右腿)
    sensor->motor_left_speed  = (float)small_driver_value.receive_left_speed_data * RPM_TO_RADPS;
    sensor->motor_right_speed = (float)small_driver_value.receive_right_speed_data * RPM_TO_RADPS * RIGHT_MOTOR_DIR;

    sensor->joint_left_front_angle  = (float)small_driver_value_leg_left.receive_left_location_data * DEG_TO_RAD;
    sensor->joint_left_back_angle   = (float)small_driver_value_leg_left.receive_right_location_data * DEG_TO_RAD;
    sensor->joint_right_front_angle = (float)small_driver_value_leg_right.receive_left_location_data * DEG_TO_RAD;
    sensor->joint_right_back_angle  = (float)small_driver_value_leg_right.receive_right_location_data * DEG_TO_RAD;

    //应用标定
    if (angle_offset_is_done()) {
        angle_offset_apply_to_sensor(sensor);
    }

    /* 关节速度由位置差分获得（驱动板持续回传位置数据） */
    {
        static float prev_lf = 0.0f, prev_lb = 0.0f, prev_rf = 0.0f, prev_rb = 0.0f;
        static bool first = true;
        if (first) {
            sensor->joint_left_front_speed  = 0.0f;
            sensor->joint_left_back_speed   = 0.0f;
            sensor->joint_right_front_speed = 0.0f;
            sensor->joint_right_back_speed  = 0.0f;
            first = false;
        } else {
            float dt = ROBOT_CONTROL_DT;
            sensor->joint_left_front_speed  = (sensor->joint_left_front_angle  - prev_lf) / dt;
            sensor->joint_left_back_speed   = (sensor->joint_left_back_angle   - prev_lb) / dt;
            sensor->joint_right_front_speed = (sensor->joint_right_front_angle - prev_rf) / dt;
            sensor->joint_right_back_speed  = (sensor->joint_right_back_angle  - prev_rb) / dt;
        }
        prev_lf = sensor->joint_left_front_angle;
        prev_lb = sensor->joint_left_back_angle;
        prev_rf = sensor->joint_right_front_angle;
        prev_rb = sensor->joint_right_back_angle;
    }

    cmd->target_height = 0.0f;

    /* 画方测试 */
    square_test(cmd);
}
