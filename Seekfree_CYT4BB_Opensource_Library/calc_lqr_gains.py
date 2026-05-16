#!/usr/bin/env python3
"""
离线 LQR 增益计算 + 多项式拟合脚本
====================================

基于五杆轮腿机器人的物理参数，对不同腿长 L 求解 LQR 最优增益，
并对增益-腿长曲线做 3 阶多项式拟合，输出 C 头文件供嵌入式端使用。

依赖: numpy, scipy

用法:
    python calc_lqr_gains.py

输出: lqr_gain_table.h （包含多项式系数的 C 头文件）

模型:
    轮式倒立摆，小角度线性化
    状态 x = [θ, θ̇, φ̇, ∫φ̇·dt]^T
    输入 u = τ (轮力矩)
"""

import numpy as np
from scipy import linalg
from pathlib import Path

# ─── 物理参数（须与 types.h 一致）───────────────────────────────────
M      = 2.5      # 车体质量 (kg), 不含轮
m_w    = 0.1      # 单个轮质量 (kg)
I_b    = 0.015    # 车体俯仰惯量 (kg*m^2)
I_w    = 0.0001   # 单个轮惯量 (kg*m^2)
g      = 9.81     # 重力加速度 (m/s^2)
r_w    = 0.020    # 轮半径 (m)
L_min  = 0.050    # 最小摆长 (m)
L_max  = 0.220    # 最大摆长 (m)
N_pts  = 20       # 采样点数

# ─── LQR 权重（需根据实物调试调整）───────────────────────────────────
# 状态: [θ, θ̇, φ̇, ∫φ̇·dt]
Q = np.diag([400.0,   # 角度权重  (rad^-2)
              20.0,   # 角速度权重 (rad/s)^-2
               5.0,   # 轮速权重  (rad/s)^-2
               1.0])  # 积分权重  (rad*s)^-2
R = np.array([1.0])    # 输入权重 (N*m)^-2

# 离散化周期
dt = 0.001  # 1 kHz


def build_abcd(L):
    """构建连续时间状态空间矩阵 A, B (4x4, 4x1)

    状态: x = [θ, θ̇, φ̇, ∫φ̇·dt]
    输入: u = τ
    """
    M_t  = M + 2.0 * m_w           # 总质量
    I_wr = 2.0 * I_w / (r_w * r_w)  # 轮惯量折算

    Delta = (I_b + M * L * L) * (I_wr + M_t * r_w * r_w) \
            - (M * L * r_w) ** 2

    A21 = M * g * L * (I_wr + M_t * r_w * r_w) / Delta
    A31 = -(M * M) * g * L * L * r_w / Delta

    B2  = -(I_wr + M_t * r_w * r_w + M * L * r_w) / Delta
    B3  = (I_b + M * L * L + M * L * r_w) / Delta

    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [A21, 0.0, 0.0, 0.0],
        [A31, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ])
    B = np.array([[0.0], [B2], [B3], [0.0]])
    C = np.eye(4)
    D = np.zeros((4, 1))

    return A, B, C, D


def discretize(A, B, dt):
    """零阶保持离散化 (ZOH)"""
    n = A.shape[0]
    M_mat = np.block([[A, B], [np.zeros((1, n)), np.zeros((1, 1))]]) * dt
    M_exp = linalg.expm(M_mat)
    Ad = M_exp[:n, :n]
    Bd = M_exp[:n, n:]
    return Ad, Bd


def dlqr(Ad, Bd, Q, R):
    """离散 LQR 求解, 返回增益矩阵 K (1x4)"""
    P = linalg.solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
    return K


def main():
    # 等间距采样腿长
    L_values = np.linspace(L_min, L_max, N_pts)

    # 存储每个 L 对应的 K 矩阵行
    K_rows = np.zeros((N_pts, 4))  # [k_angle, k_gyro, k_speed, k_int]

    print("=" * 60)
    print(f"{'L (mm)':>8} {'k_angle':>12} {'k_gyro':>12} {'k_speed':>12} {'k_int':>12}")
    print("-" * 60)

    for i, L in enumerate(L_values):
        A, B, _, _ = build_abcd(L)
        Ad, Bd = discretize(A, B, dt)
        K = dlqr(Ad, Bd, Q, R)
        K_rows[i] = K.flatten()
        print(f"{L*1000:8.1f} {K[0,0]:12.4f} {K[0,1]:12.4f} {K[0,2]:12.4f} {K[0,3]:12.4f}")

    print("-" * 60)

    # 多项式拟合（3 阶：4 个系数）
    # 使用归一化腿长 L_norm = (L - L_min) / (L_max - L_min) 作为自变量
    L_norm = (L_values - L_min) / (L_max - L_min)
    poly_order = 3

    names = ["k_angle", "k_gyro", "k_speed", "k_int"]
    coeffs_all = np.zeros((4, poly_order + 1))  # 4 组 × 4 系数

    print("\n多项式系数 (低次→高次, Horner 格式):")

    for j in range(4):
        # numpy polyfit 返回从高次到低次的系数
        coeffs_highfirst = np.polyfit(L_norm, K_rows[:, j], poly_order)
        # 转为 Horner 格式: c[0] + x*(c[1] + x*(c[2] + x*c[3]))
        # 即从低次到高次
        coeffs_lowfirst = coeffs_highfirst[::-1]
        coeffs_all[j] = coeffs_lowfirst
        print(f"{names[j]:>10}: "
              f"{coeffs_lowfirst[0]:+.6e} "
              f"{coeffs_lowfirst[1]:+.6e} "
              f"{coeffs_lowfirst[2]:+.6e} "
              f"{coeffs_lowfirst[3]:+.6e}")

    # ─── 重力前馈系数拟合 ────────────────────────────────────
    # 重力前馈与摆长线性相关: k_gravity ~ M*g*L (近似)
    # 但实际前馈系数还包含电机力矩常数折算，这里做多项式拟合
    L_norm_fine = np.linspace(0, 1, N_pts)
    # 重力前馈参考值（归一化到实际调试范围）
    grav_base = 50.0   # 标称重力前馈系数
    grav_scale = 1.5   # 随腿长变化比例
    k_gravity_ref = grav_base * (1.0 + grav_scale * L_norm_fine)
    grav_coeffs_high = np.polyfit(L_norm_fine, k_gravity_ref, poly_order)
    grav_coeffs = grav_coeffs_high[::-1]
    print(f"{'k_gravity':>10}: "
          f"{grav_coeffs[0]:+.6e} "
          f"{grav_coeffs[1]:+.6e} "
          f"{grav_coeffs[2]:+.6e} "
          f"{grav_coeffs[3]:+.6e}")

    # ─── 生成 C 头文件 ────────────────────────────────────────
    header = """/*
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
""".lstrip()

    for j, name in enumerate(names):
        header += f"static const float g_{name}_coeffs[LQR_POLY_ORDER] = {{\n"
        for k in range(4):
            header += f"    {coeffs_all[j, k]:+.8e}f"
            if k < 3:
                header += ","
            header += "\n"
        header += "};\n\n"

    # 重力前馈系数
    header += "static const float g_k_gravity_coeffs[LQR_POLY_ORDER] = {\n"
    for k in range(4):
        header += f"    {grav_coeffs[k]:+.8e}f"
        if k < 3:
            header += ","
        header += "\n"
    header += "};\n\n"

    header += """#endif /* LQR_GAIN_TABLE_H */
"""

    out_path = Path(__file__).parent / "project" / "code" / "control" / "balance" / "lqr_gain_table.h"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(header)
    print(f"\n已生成: {out_path}")


if __name__ == "__main__":
    main()
