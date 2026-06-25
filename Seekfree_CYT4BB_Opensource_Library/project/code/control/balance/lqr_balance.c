#include "lqr_balance.h"
#include "../leg/kinematics.h"
#include "math.h"

/* ─── LQR 默认参数（固定增益模式，兼容旧行为）───────────────────────── */
/* 这些是通过离线求解 Riccati 方程得到的名义值，需根据实际车体调校 */

/* 状态反馈增益 K = [k_angle, k_gyro, k_speed, k_int] */
#define LQR_K_ANGLE_DEFAULT     (-150.0f)   /* 角度反馈增益 */
#define LQR_K_GYRO_DEFAULT      (-25.0f)    /* 角速度反馈增益 */
#define LQR_K_SPEED_DEFAULT     (-8.0f)     /* 速度反馈增益 */
#define LQR_K_INT_DEFAULT       (-0.5f)     /* 速度积分增益 */

#define LQR_ANGLE_OFFSET        0.0f         /* IMU 水平安装偏置 (rad) */
#define LQR_GRAVITY_COMP_DEFAULT (50.0f)     /* 重力前馈系数 */
#define LQR_INTEGRAL_LIMIT      500.0f       /* 积分限幅 */
#define LQR_CONTROL_DT          0.001f       /* 控制周期 (s) */

/* ─── 腿长相关常量 ─────────────────────────────────────────────────── */
#define L_MIN_M                 (LEG_LENGTH_MIN_MM * 0.001f)  /* m */
#define L_MAX_M                 (LEG_LENGTH_MAX_MM * 0.001f)  /* m */
#define L_NOMINAL_M             (LEG_LENGTH_STANDARD * 0.001f)/* m */
#define L_FILTER_WINDOW         5           /* 滑动平均窗长度 */

/*──────────────────────────────────────────────────────────────────────*/
/* Horner 法求多项式值：c[0] + x*(c[1] + x*(c[2] + x*c[3]))           */
static inline float poly_eval(const float c[LQR_POLY_ORDER], float x)
{
    return c[0] + x * (c[1] + x * (c[2] + x * c[3]));
}

/*──────────────────────────────────────────────────────────────────────*/
/* 对 L 做滑动平均滤波                                                    */
static float L_filter(LQR_balance_t *lqr, float L_raw)
{
    lqr->L_buffer[lqr->L_buffer_idx] = L_raw;
    lqr->L_buffer_idx = (lqr->L_buffer_idx + 1) % L_FILTER_WINDOW;
    if (lqr->L_buffer_count < L_FILTER_WINDOW)
        lqr->L_buffer_count++;

    float sum = 0.0f;
    for (uint8_t i = 0; i < lqr->L_buffer_count; i++)
        sum += lqr->L_buffer[i];
    return sum / (float)lqr->L_buffer_count;
}

/*──────────────────────────────────────────────────────────────────────*/
/* 更新调度增益：根据 L_norm 查多项式得到当前最优 K                       */
static void update_scheduled_gains(LQR_balance_t *lqr)
{
    float x = lqr->L_norm;
    lqr->k_angle        = poly_eval(lqr->k_angle_coeffs, x);
    lqr->k_gyro         = poly_eval(lqr->k_gyro_coeffs, x);
    lqr->k_speed        = poly_eval(lqr->k_speed_coeffs, x);
    lqr->k_int          = poly_eval(lqr->k_int_coeffs, x);
    lqr->k_gravity_comp = poly_eval(lqr->k_gravity_coeffs, x);
}

/*──────────────────────────────────────────────────────────────────────*/
/* 初始化（固定增益模式，兼容旧行为）                                    */
void lqr_balance_init(LQR_balance_t *lqr)
{
    lqr->k_angle          = LQR_K_ANGLE_DEFAULT;
    lqr->k_gyro           = LQR_K_GYRO_DEFAULT;
    lqr->k_speed          = LQR_K_SPEED_DEFAULT;
    lqr->k_int            = LQR_K_INT_DEFAULT;

    lqr->angle_offset     = LQR_ANGLE_OFFSET;
    lqr->speed_integral   = 0.0f;
    lqr->integral_limit   = LQR_INTEGRAL_LIMIT;

    lqr->k_gravity_comp   = LQR_GRAVITY_COMP_DEFAULT;

    lqr->dt               = LQR_CONTROL_DT;
    lqr->target_speed     = 0.0f;

    /* 增益调度相关初始化为 0 */
    lqr->scheduling_enabled = 0;
    lqr->com_height        = L_NOMINAL_M + ROBOT_COM_OFFSET;
    lqr->L_norm            = CLAMP((lqr->com_height - L_MIN_M)
                                   / (L_MAX_M - L_MIN_M), 0.0f, 1.0f);
    lqr->L_buffer_idx      = 0;
    lqr->L_buffer_count    = 0;
    for (int i = 0; i < LQR_POLY_ORDER; i++) {
        lqr->k_angle_coeffs[i]   = 0.0f;
        lqr->k_gyro_coeffs[i]    = 0.0f;
        lqr->k_speed_coeffs[i]   = 0.0f;
        lqr->k_int_coeffs[i]     = 0.0f;
        lqr->k_gravity_coeffs[i] = 0.0f;
    }
}

/*──────────────────────────────────────────────────────────────────────*/
/* 初始化（增益调度模式）                                                */
void lqr_balance_init_scheduled(LQR_balance_t *lqr,
    const float k_angle_c[LQR_POLY_ORDER],
    const float k_gyro_c[LQR_POLY_ORDER],
    const float k_speed_c[LQR_POLY_ORDER],
    const float k_int_c[LQR_POLY_ORDER],
    const float k_gravity_c[LQR_POLY_ORDER])
{
    /* 先做基本初始化 */
    lqr_balance_init(lqr);

    /* 填入多项式系数 */
    for (int i = 0; i < LQR_POLY_ORDER; i++) {
        lqr->k_angle_coeffs[i]   = k_angle_c[i];
        lqr->k_gyro_coeffs[i]    = k_gyro_c[i];
        lqr->k_speed_coeffs[i]   = k_speed_c[i];
        lqr->k_int_coeffs[i]     = k_int_c[i];
        lqr->k_gravity_coeffs[i] = k_gravity_c[i];
    }

    /* 启用调度并计算初始增益 */
    lqr->scheduling_enabled = 1;
    update_scheduled_gains(lqr);
}

/*──────────────────────────────────────────────────────────────────────*/
void lqr_balance_reset(LQR_balance_t *lqr)
{
    lqr->speed_integral = 0.0f;
}

/*──────────────────────────────────────────────────────────────────────*/
/* 根据关节角度更新 COM 高度和调度增益                                   */
void lqr_balance_update_geometry(LQR_balance_t *lqr,
    const Sensor_data_t *sensor_data)
{
    Joint_angle_t angles_L, angles_R;
    Foot_position_t foot_L, foot_R;
    float foot_y_L_mm, foot_y_R_mm, foot_y_avg_m;
    float L_raw;

    /* 左腿正运动学 */
    angles_L.left_motor_angle  = sensor_data->joint_left_front_angle;
    angles_L.right_motor_angle = sensor_data->joint_left_back_angle;
    if (five_bar_forward(&angles_L, &foot_L) != 0) {
        /* 奇异位置，保持上次有效值 */
        return;
    }
    foot_y_L_mm = foot_L.y;

    /* 右腿正运动学 */
    angles_R.left_motor_angle  = sensor_data->joint_right_front_angle;
    angles_R.right_motor_angle = sensor_data->joint_right_back_angle;
    if (five_bar_forward(&angles_R, &foot_R) != 0) {
        return;
    }
    foot_y_R_mm = foot_R.y;

    /* 双足平均 y（mm → m），y 正方向朝下 */
    foot_y_avg_m = (foot_y_L_mm + foot_y_R_mm) * 0.5f * 0.001f;

    /* 摆长 L = 足端到髋距离 + COM 偏移量 */
    L_raw = foot_y_avg_m + ROBOT_COM_OFFSET;

    /* 限幅防外推 */
    L_raw = CLAMP(L_raw, L_MIN_M, L_MAX_M);

    /* 滑动平均滤波 */
    lqr->com_height = L_filter(lqr, L_raw);

    /* 归一化腿长 [0, 1] */
    lqr->L_norm = (lqr->com_height - L_MIN_M) / (L_MAX_M - L_MIN_M);
    lqr->L_norm = CLAMP(lqr->L_norm, 0.0f, 1.0f);

    /* 如果调度使能，更新增益 */
    if (lqr->scheduling_enabled) {
        update_scheduled_gains(lqr);
    }
}

/*──────────────────────────────────────────────────────────────────────*/
void lqr_balance_control(LQR_balance_t *lqr,
    const Sensor_data_t *sensor_data,
    Motor_cmd_duty_t *motor_cmd)
{
    /* ── 1. 读取当前状态 ────────────────────────────────────────────── */
    float angle = sensor_data->angle_pitch - lqr->angle_offset;   /* θ */
    float gyro  = sensor_data->gyro_pitch;                         /* θ̇ */
    float speed = (sensor_data->motor_left_speed
                 + sensor_data->motor_right_speed) * 0.5f;        /* φ̇ */

    /* ── 2. 速度误差积分（消除稳态速度静差） ────────────────────────── */
    float speed_error = speed - lqr->target_speed;
    lqr->speed_integral += speed_error * lqr->dt;

    /* 积分限幅 */
    if (lqr->speed_integral >  lqr->integral_limit)
        lqr->speed_integral =  lqr->integral_limit;
    if (lqr->speed_integral < -lqr->integral_limit)
        lqr->speed_integral = -lqr->integral_limit;

    /* ── 3. LQR 状态反馈 u = -K·x ──────────────────────────────────── */
    float u = -(lqr->k_angle * angle
              + lqr->k_gyro  * gyro
              + lqr->k_speed * speed_error
              + lqr->k_int   * lqr->speed_integral);

    /* ── 4. 重力前馈补偿（抵消非线性重力矩） ──────────────────────── */
    float ff_gravity = lqr->k_gravity_comp * sinf(angle);
    u += ff_gravity;

    /* ── 5. 输出 PWM ───────────────────────────────────────────────── */
    motor_cmd->left_motor_pwm  = ROUND(u);
    motor_cmd->right_motor_pwm = ROUND(-u);
}
