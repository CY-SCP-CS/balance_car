#include "pitch_balance.h"
#include "math.h"

/* ============================================================================
 * balance_control — 纯平衡控制
 * 角度外环 (P) → 角速度内环 (PD) 串级 + 重力前馈补偿
 * 返回平衡 PWM, 由调用方与速度环输出叠加
 * ============================================================================ */
float balance_control(const Sensor_data_t *sensor,
    PID_Controller_t *pid_angle, PID_Controller_t *pid_gyro)
{
    float angle_cur = sensor->angle_pitch - PITCH_ANGLE_OFFSET_DEG * DEG_TO_RAD;
    float gyro_cur  = sensor->gyro_pitch;

    /* 角度外环: 目标角度 = 0 (直立点) */
    float gyro_target = pid_calculate(pid_angle, 0.0f, angle_cur);

    /* 角速度内环: 跟踪角度环输出的角速度目标 */
    float pwm = pid_calculate(pid_gyro, gyro_target, gyro_cur);

    /* 重力前馈补偿 */
    pwm += GRAVITY_COMP_GAIN * sinf(angle_cur);

    return pwm;
}

/* ============================================================================
 * speed_control — 独立速度控制
 * 满频运行 (1000Hz), 与平衡环并联叠加
 * 刹车: 强比例前馈直接注入制动 PWM, 绕过 PID 慢速积分
 * ============================================================================ */
#define BRAKE_FF_GAIN 7000.0f  /* 刹车前馈增益, 克服平衡环的反向推力 */

float speed_control(const Sensor_data_t *sensor,
    PID_Controller_t *pid_speed, float target_speed)
{
    float speed_cur  = (sensor->motor_left_speed + sensor->motor_right_speed) / 2.0f;
    float speed_norm = speed_cur / 60.0f;  /* 归一化到与 target_speed 同一量纲 */

    float pwm = pid_calculate(pid_speed, target_speed, speed_norm);

    /* 刹车前馈注入: 目标=0 且车速>阈值时, 叠加强比例制动 */
    if (fabsf(target_speed) < 0.01f && fabsf(speed_norm) > 0.03f) {
        pwm += -BRAKE_FF_GAIN * speed_norm;
    }

    return pwm;
}

/* ============================================================================
 * 以下为旧版本代码, 保留但未使用
 * ============================================================================ */

static uint8_t pb_counter      = 0;
static float   pb_speed_output = 0.0f;

void pitch_balance_reset_statics(void) {
    pb_counter      = 0;
    pb_speed_output = 0.0f;
}

#define MAX_STATIC_ANGLE ((1.5f) * DEG_TO_RAD)

void pitch_balance_control_fuzzy_pid(const Sensor_data_t *sensor,
    PID_Controller_t *pid_speed, PID_Controller_t *pid_angle, PID_Controller_t *pid_gyro,
    Motor_cmd_duty_t *motor_cmd, float target_speed){

    // ==========================================
    // 1. 数据采样与处理（满频运行，不降频）
    // ==========================================
    float speed_cur = (sensor->motor_left_speed + sensor->motor_right_speed) / 2.0f;
    float speed_norm = speed_cur / 60.0f;
    float angle_cur = sensor->angle_pitch - PITCH_ANGLE_OFFSET_DEG * DEG_TO_RAD;
    float gyro_cur  = sensor->gyro_pitch;

    // ==========================================
    // 2. 简易模糊自适应逻辑
    // ==========================================
    float abs_angle = fabsf(angle_cur);
    float speed_scale = 1.0f;
    float speed_kd_bias = 0.0f;

    if (abs_angle < MAX_STATIC_ANGLE) {
        float static_ratio = 1.0f - (abs_angle / MAX_STATIC_ANGLE);
        speed_scale = 1.0f;
        speed_kd_bias = 0.2f * static_ratio;
    } else {
        speed_scale = 1.0f - ((abs_angle - MAX_STATIC_ANGLE) / ((5.0f) * DEG_TO_RAD));
        if (speed_scale < 0.1f) speed_scale = 0.1f;
        speed_kd_bias = 0.0f;
    }

    // ==========================================
    // 3. 速度环计算
    // ==========================================
    float original_kd = pid_speed->kd;
    pid_speed->kd = original_kd + speed_kd_bias;

    float speed_output = pid_calculate(pid_speed, target_speed, speed_norm);
    pid_speed->kd = original_kd;

    speed_output *= speed_scale;

    // ==========================================
    // 4. 姿态串级环计算
    // ==========================================
    float gyro_target = pid_calculate(pid_angle, 0.0f, angle_cur);
    float pwm_base = pid_calculate(pid_gyro, gyro_target, gyro_cur) + speed_output;

    // ==========================================
    // 5. 前馈重力补偿与最终输出
    // ==========================================
    float ff_gravity = GRAVITY_COMP_GAIN * sinf(angle_cur);
    pwm_base += ff_gravity;

    motor_cmd->left_motor_pwm  = ROUND(pwm_base);
    motor_cmd->right_motor_pwm = ROUND(-pwm_base);
}
