#include "pitch_balance.h"
#include "math.h"

/* 降频 static 变量 — 文件作用域，供外部复位 */
static uint8_t pb_counter      = 0;
static float   pb_gyro_target  = 0.0f;
static float   pb_speed_output = 0.0f;

void pitch_balance_reset_statics(void) {
    pb_counter      = 0;
    pb_gyro_target  = 0.0f;
    pb_speed_output = 0.0f;
}

void pitch_balance_control(const Sensor_data_t *sensor_data,
    PID_Controller_t *pid_speed, PID_Controller_t *pid_angle, PID_Controller_t *pid_gyro,
    Motor_cmd_duty_t *motor_cmd){

    if (pb_counter == 0) {
        float speed_cur = (sensor_data->motor_left_speed + sensor_data->motor_right_speed) / 2.0f;
        pb_speed_output = pid_calculate(pid_speed, 0.0f, speed_cur);

        float angle_cur = sensor_data->angle_pitch - PITCH_ANGLE_OFFSET_DEG * DEG_TO_RAD;
        float angle_output = pid_calculate(pid_angle, 0.0f, angle_cur);

        pb_gyro_target = angle_output;
    }

    pb_counter = (pb_counter + 1) % 10;

    float gyro_cur = sensor_data->gyro_pitch;
    float pwm_base = pid_calculate(pid_gyro, pb_gyro_target, gyro_cur) + pb_speed_output;

    float angle_cur = sensor_data->angle_pitch - PITCH_ANGLE_OFFSET_DEG * DEG_TO_RAD;
    float ff_gravity = GRAVITY_COMP_GAIN * sinf(angle_cur);
    pwm_base += ff_gravity;

    motor_cmd->left_motor_pwm  = ROUND(pwm_base);
    motor_cmd->right_motor_pwm = ROUND(-pwm_base);
}

#define MAX_STATIC_ANGLE ((1.5f) * DEG_TO_RAD) // 认为车体算得上”静止”的最大角度偏差（约1.5度）

void pitch_balance_control_fuzzy_pid(const Sensor_data_t *sensor_data,
    PID_Controller_t *pid_speed, PID_Controller_t *pid_angle, PID_Controller_t *pid_gyro,
    Motor_cmd_duty_t *motor_cmd){

    // ==========================================
    // 1. 数据采样与处理（满频运行，不降频）
    // ==========================================
    float speed_cur = (sensor_data->motor_left_speed + sensor_data->motor_right_speed) / 2.0f;
    float angle_cur = sensor_data->angle_pitch - PITCH_ANGLE_OFFSET_DEG * DEG_TO_RAD;
    float gyro_cur  = sensor_data->gyro_pitch;

    // ==========================================
    // 2. 简易模糊自适应逻辑（解决无法静止的关键）
    // ==========================================
    float abs_angle = fabsf(angle_cur);
    float speed_scale = 1.0f; // 速度环整体威力因子
    float speed_kd_bias = 0.0f; // 速度环的动态阻尼

    if (abs_angle < MAX_STATIC_ANGLE) {
        // 【状态A：车体很接近直立点，需要死死静止】
        // 角度越小，速度环权重越大，且人为注入一个高阻尼（模拟模糊Kd增大），防止前后洗衣服
        float static_ratio = 1.0f - (abs_angle / MAX_STATIC_ANGLE); // 0.0 ~ 1.0
        speed_scale = 1.0f;

        // 当接近绝对静止时，额外给速度环提供 -0.1 ~ -0.3 的临时 D 增量
        // 这一项相当于模糊 PID 里的 Delta_Kd，用来吸收原地的低频晃动
        speed_kd_bias = 0.2f * static_ratio;
    } else {
        // 【状态B：车体处于运动状态或受到扰动】
        // 随着角度变大，线性削弱速度环。如果角度过大，速度环完全让位给平衡环
        speed_scale = 1.0f - ((abs_angle - MAX_STATIC_ANGLE) / ((5.0f) * DEG_TO_RAD));
        if (speed_scale < 0.1f) speed_scale = 0.1f; // 至少保留10%防飞车
        speed_kd_bias = 0.0f;
    }

    // ==========================================
    // 3. 速度环计算
    // ==========================================
    // 动态修改速度环的 D 参数以吸收原地晃动
    float original_kd = pid_speed->kd;
    pid_speed->kd = original_kd + speed_kd_bias; // 注入模糊阻尼

    float speed_output = pid_calculate(pid_speed, 0.0f, speed_cur);
    pid_speed->kd = original_kd; // 恢复原参数防止污染下一次计算

    // 应用模糊权重
    speed_output *= speed_scale;

    // ==========================================
    // 4. 姿态串级环计算
    // ==========================================
    // 角度外环
    float gyro_target = pid_calculate(pid_angle, 0.0f, angle_cur);

    // 角速度内环 + 速度环并联叠加
    float pwm_base = pid_calculate(pid_gyro, gyro_target, gyro_cur) + speed_output;

    // ==========================================
    // 5. 前馈重力补偿与最终输出
    // ==========================================
    float ff_gravity = GRAVITY_COMP_GAIN * sinf(angle_cur);
    pwm_base += ff_gravity;

    motor_cmd->left_motor_pwm  = ROUND(pwm_base);
    motor_cmd->right_motor_pwm = ROUND(-pwm_base);
}