#include "kinematics.h"
#include <math.h>

/*---------------------------------------------------------------------------*/
/* 正运动学：已知电机角 -> 求足端坐标 */
int five_bar_forward(const Joint_angle_t *angles, Foot_position_t *foot) {
    const float half_distance = LEG_JOINT_DISTANCE * 0.5f;
    const float thigh_len = LEG_THIGH;
    const float shank_len = LEG_SHANK;

    /* 1. 计算左右膝关节位置 (Knee Joints) */
    /* 左电机坐标 (-half_distance, 0), 右电机坐标 (half_distance, 0) */
    float left_knee_x  = -half_distance + thigh_len * cosf(angles->left_motor_angle);
    float left_knee_y  = thigh_len * sinf(angles->left_motor_angle);
    float right_knee_x = half_distance + thigh_len * cosf(angles->right_motor_angle);
    float right_knee_y = thigh_len * sinf(angles->right_motor_angle);

    /* 2. 计算膝关节间的直线距离 */
    float knee_distance_x = right_knee_x - left_knee_x;
    float knee_distance_y = right_knee_y - left_knee_y;
    float knee_distance_sq = knee_distance_x * knee_distance_x + knee_distance_y * knee_distance_y;
    float knee_distance = sqrtf(knee_distance_sq);

    /* 几何检查：防止超出物理极限（小腿无法够到彼此，或靠得太近导致奇异点） */
    if (knee_distance > 2.0f * shank_len || knee_distance < 0.01f) return -1;

    /* 3. 计算足端 (Foot) 位置 */
    float half_knee_distance = knee_distance * 0.5f;
    float foot_height_sq = shank_len * shank_len - half_knee_distance * half_knee_distance;
    float foot_height = sqrtf(fmaxf(0.0f, foot_height_sq));

    /* 膝关节连线中点 */
    float mid_knee_x = (left_knee_x + right_knee_x) * 0.5f;
    float mid_knee_y = (left_knee_y + right_knee_y) * 0.5f;

    /* 计算单位法向量 (垂直于膝关节连线并指向下方) */
    /* 始终确保法向量朝向 Y 轴正方向(向下) */
    float unit_nx = -knee_distance_y / knee_distance;
    float unit_ny =  knee_distance_x / knee_distance;
    
    if (unit_ny < 0.0f) { // 防止极端情况下法向量反向
        unit_nx = -unit_nx;
        unit_ny = -unit_ny;
    }

    foot->x = mid_knee_x + foot_height * unit_nx;
    foot->y = mid_knee_y + foot_height * unit_ny;

    return 0;
}

/* 逆运动学：已知足端坐标 -> 求电机角 */
int five_bar_inverse(const Foot_position_t *foot, Joint_angle_t *angles) {
    const float half_distance = LEG_JOINT_DISTANCE * 0.5f;
    const float thigh_len = LEG_THIGH;
    const float shank_len = LEG_SHANK;
    const float thigh_sq = thigh_len * thigh_len;
    const float shank_sq = shank_len * shank_len;

    /* --- 左侧电机计算 --- */
    /* 向量：从左电机指向足端 */
    float left_to_foot_x = foot->x - (-half_distance); 
    float left_to_foot_y = foot->y - 0.0f;
    float left_dist_sq = left_to_foot_x * left_to_foot_x + left_to_foot_y * left_to_foot_y;
    float left_dist    = sqrtf(left_dist_sq);

    /* 边界检查 */
    if (left_dist > (thigh_len + shank_len) || left_dist < fabsf(thigh_len - shank_len) || left_dist < 0.001f)
        return -1;

    /* 左电机到足端连线的绝对角度 */
    float base_angle_left = atan2f(left_to_foot_y, left_to_foot_x); 
    /* 余弦定理求大腿与连线的夹角 */
    float cos_alpha_left = (thigh_sq + left_dist_sq - shank_sq) / (2.0f * thigh_len * left_dist);
    float alpha_left = acosf(fminf(1.0f, fmaxf(-1.0f, cos_alpha_left))); 

    /* --- 右侧电机计算 --- */
    /* 向量：从右电机指向足端 */
    float right_to_foot_x = foot->x - half_distance;
    float right_to_foot_y = foot->y - 0.0f;
    float right_dist_sq = right_to_foot_x * right_to_foot_x + right_to_foot_y * right_to_foot_y;
    float right_dist    = sqrtf(right_dist_sq);

    /* 边界检查 */
    if (right_dist > (thigh_len + shank_len) || right_dist < fabsf(thigh_len - shank_len) || right_dist < 0.001f)
        return -1;

    /* 右电机到足端连线的绝对角度 */
    float base_angle_right = atan2f(right_to_foot_y, right_to_foot_x); 
    float cos_alpha_right = (thigh_sq + right_dist_sq - shank_sq) / (2.0f * thigh_len * right_dist);
    float alpha_right = acosf(fminf(1.0f, fmaxf(-1.0f, cos_alpha_right))); 

    angles->left_motor_angle  = base_angle_left - alpha_left;
    angles->right_motor_angle = base_angle_right + alpha_right;

    return 0;
}