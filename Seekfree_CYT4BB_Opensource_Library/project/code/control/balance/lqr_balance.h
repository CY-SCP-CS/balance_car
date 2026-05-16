#ifndef LQR_BALANCE_H
#define LQR_BALANCE_H

#include <stdint.h>
#include "../../app/robot_control/types.h"

/* LQR 状态向量维度 */
#define LQR_STATE_DIM   4   /* [angle, gyro, speed, speed_integral] */

/* 增益调度多项式阶数 */
#define LQR_POLY_ORDER  4   /* 3 阶多项式，4 个系数 */

/* LQR 控制器状态 */
typedef struct
{
    /* ─── LQR 增益矩阵 K（1×4），由离线计算填入 ─── */
    float k_angle;              /* 角度增益 */
    float k_gyro;               /* 角速度增益 */
    float k_speed;              /* 速度增益 */
    float k_int;                /* 速度积分增益 */

    /* ─── 增益调度多项式系数 (Horner: c0 + x*(c1 + x*(c2 + x*c3))) ─── */
    float k_angle_coeffs[LQR_POLY_ORDER];   /* 角度增益多项式系数 */
    float k_gyro_coeffs[LQR_POLY_ORDER];    /* 角速度增益多项式系数 */
    float k_speed_coeffs[LQR_POLY_ORDER];   /* 速度增益多项式系数 */
    float k_int_coeffs[LQR_POLY_ORDER];     /* 积分增益多项式系数 */
    float k_gravity_coeffs[LQR_POLY_ORDER]; /* 重力前馈多项式系数 */

    /* ─── 当前 COM 高度 / 归一化腿长 ─── */
    float com_height;           /* 当前摆长 L (m) */
    float L_norm;               /* 归一化腿长 [0, 1] */

    /* ─── 调度使能 ─── */
    uint8_t scheduling_enabled; /* 0=固定增益, 1=增益调度 */

    /* ─── 状态反馈中间变量 ─── */
    float angle_offset;         /* IMU 安装偏置 (rad) */
    float speed_integral;       /* 速度积分项，用于消除静差 */
    float integral_limit;       /* 积分限幅 */

    /* ─── 前馈补偿 ─── */
    float k_gravity_comp;       /* 重力前馈系数（调度关闭时使用） */

    /* ─── 控制周期 ─── */
    float dt;

    /* ─── 目标值 ─── */
    float target_speed;         /* 目标速度 (rad/s) */

    /* ─── 滑动平均滤波 ─── */
    float L_buffer[10];         /* COM 高度滑动平均缓冲区 */
    uint8_t L_buffer_idx;       /* 缓冲区索引 */
    uint8_t L_buffer_count;     /* 缓冲区有效数据计数 */
} LQR_balance_t;

/* ─── API ──────────────────────────────────────────────────────────── */

/* 初始化（固定增益模式，兼容旧行为） */
void lqr_balance_init(LQR_balance_t *lqr);

/* 初始化（增益调度模式）*/
void lqr_balance_init_scheduled(LQR_balance_t *lqr,
    const float k_angle_c[LQR_POLY_ORDER],
    const float k_gyro_c[LQR_POLY_ORDER],
    const float k_speed_c[LQR_POLY_ORDER],
    const float k_int_c[LQR_POLY_ORDER],
    const float k_gravity_c[LQR_POLY_ORDER]);

/* 重置积分项 */
void lqr_balance_reset(LQR_balance_t *lqr);

/* 根据关节角度更新 COM 高度和调度增益 */
void lqr_balance_update_geometry(LQR_balance_t *lqr,
    const Sensor_data_t *sensor_data);

/* LQR 控制主函数 */
void lqr_balance_control(LQR_balance_t *lqr,
    const Sensor_data_t *sensor_data,
    Motor_cmd_duty_t *motor_cmd);

#endif /* LQR_BALANCE_H */
