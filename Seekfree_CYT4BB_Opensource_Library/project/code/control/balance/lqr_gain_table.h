/*
 * lqr_gain_table.h
 * 自动生成 — 请勿手动修改
 * 生成命令: python calc_lqr_gains.py
 *
 * 增益调度多项式系数（3 阶, Horner 格式）
 *   gain = c[0] + L_norm*(c[1] + L_norm*(c[2] + L_norm*c[3]))
 * 其中 L_norm = (L - L_min) / (L_max - L_min),  L_norm ∈ [0,1]
 */

#ifndef LQR_GAIN_TABLE_H
#define LQR_GAIN_TABLE_H

#include "lqr_balance.h"

/* 增益调度多项式系数 */
static const float g_k_angle_coeffs[LQR_POLY_ORDER] = {
    -2.15513660e+01f,
    -1.37393635e+01f,
    +3.46774807e+00f,
    -1.92097466e+00f
};

static const float g_k_gyro_coeffs[LQR_POLY_ORDER] = {
    -4.32912556e+00f,
    -1.42380417e+00f,
    -1.82770083e-01f,
    -3.10458837e-01f
};

static const float g_k_speed_coeffs[LQR_POLY_ORDER] = {
    -4.32726773e+00f,
    +3.24203952e+00f,
    -3.69297039e+00f,
    +1.61885334e+00f
};

static const float g_k_int_coeffs[LQR_POLY_ORDER] = {
    -8.97168424e-01f,
    -1.99875292e-01f,
    +1.77644420e-01f,
    -6.05150892e-02f
};

static const float g_k_gravity_coeffs[LQR_POLY_ORDER] = {
    +5.00000000e+01f,
    +7.50000000e+01f,
    +6.29201707e-13f,
    -6.07550343e-13f
};

#endif /* LQR_GAIN_TABLE_H */
