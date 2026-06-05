#include "jacobian.h"
#include <math.h>

int five_bar_jacobian(const Joint_angle_t *angles, float J[2][2]){
    const float half_dist = LEG_JOINT_DISTANCE * 0.5f;
    const float thigh_len = LEG_THIGH;
    const float shank_len = LEG_SHANK;

    /* ---- 1. 计算膝关节位置 (复用正运动学逻辑) ---- */
    float sin_left = sinf(angles->left_motor_angle);
    float cos_left = cosf(angles->left_motor_angle);
    float sin_right = sinf(angles->right_motor_angle);
    float cos_right = cosf(angles->right_motor_angle);

    float left_knee_x  = -half_dist + thigh_len * cos_left;
    float left_knee_y  =  thigh_len * sin_left;
    float right_knee_x =  half_dist + thigh_len * cos_right;
    float right_knee_y =  thigh_len * sin_right;

    /* ---- 2. 计算足端位置 (复用正运动学逻辑确定足端坐标) ---- */
    float knee_dx = right_knee_x - left_knee_x;
    float knee_dy = right_knee_y - left_knee_y;
    float knee_dist = sqrtf(knee_dx * knee_dx + knee_dy * knee_dy);

    if (knee_dist > 2.0f * shank_len || knee_dist < 0.1f) return -1;

    float half_knee_dist = knee_dist * 0.5f;
    float foot_h = sqrtf(fmaxf(0.0f, shank_len * shank_len - half_knee_dist * half_knee_dist));

    float mid_knee_x = (left_knee_x + right_knee_x) * 0.5f;
    float mid_knee_y = (left_knee_y + right_knee_y) * 0.5f;

    /* 法向量指向下方 (与 Forward 保持一致) */
    float unit_nx = -knee_dy / knee_dist;
    float unit_ny =  knee_dx / knee_dist;

    float foot_x = mid_knee_x + foot_h * unit_nx;
    float foot_y = mid_knee_y + foot_h * unit_ny;

    /* ---- 3. 构建雅可比矩阵计算所需的几何向量 ---- */
    /*
     * A * V_foot = B * Omega_motor
     * A = [ (Foot-LeftKnee)^T ]   B = [ (Foot-LeftKnee)^T * V_LeftKnee_Unit  * thigh ]
     *     [ (Foot-RightKnee)^T]       [ (Foot-RightKnee)^T * V_RightKnee_Unit * thigh ]
     */

    /* 向量：膝关节 -> 足端 (即小腿方向) */
    float l_shank_x = foot_x - left_knee_x;
    float l_shank_y = foot_y - left_knee_y;
    float r_shank_x = foot_x - right_knee_x;
    float r_shank_y = foot_y - right_knee_y;

    /* 矩阵 B 的对角元素: thigh * (w_hat cross shank_vector) */
    /* 简化为：thigh * (-sin(th)*shank_x + cos(th)*shank_y) */
    float b11 = thigh_len * (-sin_left * l_shank_x + cos_left * l_shank_y);
    float b22 = thigh_len * (-sin_right * r_shank_x + cos_right * r_shank_y);

    /* 矩阵 A 的行列式 (detA = l_shank_x * r_shank_y - l_shank_y * r_shank_x) */
    float detA = l_shank_x * r_shank_y - l_shank_y * r_shank_x;

    /* 奇异性检查：当两小腿共线时，无法通过电机解耦足端运动 */
    if (fabsf(detA) < 0.1f) return -1;

    float inv_detA = 1.0f / detA;

    /* ---- 4. 最终解算 J = A^-1 * B ---- */
    /* J[0][0] = dx/dtheta1, J[0][1] = dx/dtheta2 */
    /* J[1][0] = dy/dtheta1, J[1][1] = dy/dtheta2 */
    J[0][0] =  inv_detA * r_shank_y * b11;
    J[0][1] = -inv_detA * l_shank_y * b22;
    J[1][0] = -inv_detA * r_shank_x * b11;
    J[1][1] =  inv_detA * l_shank_x * b22;

    return 0;
}

/*---------------------------------------------------------------------------*/
/* 对 2×2 雅可比求逆并求解 J * dθ = [dx; dy] *///AI给的，实测没问题
int five_bar_jacobian_solve(const float J[2][2],
                            float *dtheta1, float *dtheta2,
                            float dx, float dy) {
    float det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (fabsf(det) < 1e-8f) return -1;

    float inv_det = 1.0f / det;
    *dtheta1 = inv_det * ( J[1][1] * dx - J[0][1] * dy);
    *dtheta2 = inv_det * (-J[1][0] * dx + J[0][0] * dy);
    return 0;
}
