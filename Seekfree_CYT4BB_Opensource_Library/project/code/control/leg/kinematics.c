#include "kinematics.h"
#include <math.h>

/*---------------------------------------------------------------------------*/
/* 正运动学：已知电机角 -> 求足端坐标 */
int five_bar_forward(const Joint_angle_t *angles,
                    Foot_position_t *foot){
    const float half_distance = LEG_JOINT_DISTANCE * 0.5f;
    const float thigh_len = LEG_THIGH;
    const float shank_len = LEG_SHANK;

    /* 1. 计算左右膝关节位置 (Knee Joints) */
    float left_knee_x  = -half_distance + thigh_len * cosf(angles->left_motor_angle);
    float left_knee_y  =  thigh_len * sinf(angles->left_motor_angle);
    float right_knee_x =  half_distance + thigh_len * cosf(angles->right_motor_angle);
    float right_knee_y =  thigh_len * sinf(angles->right_motor_angle);

    /* 2. 计算膝关节间的直线距离 */
    float knee_distance_x = right_knee_x - left_knee_x;
    float knee_distance_y = right_knee_y - left_knee_y;
    float knee_distance = sqrtf(knee_distance_x * knee_distance_x 
        + knee_distance_y * knee_distance_y);

    /* 几何检查：防止超出物理极限（两小腿无法合拢） */
    if (knee_distance > 2.0f * shank_len || knee_distance < 0.1f) return -1;

    /* 3. 计算足端 (Foot) 位置 */
    /* 足端相对于膝关节连线中点的垂直偏移高度 */
    float half_knee_distance = knee_distance * 0.5f;
    float foot_height_sq = shank_len * shank_len - half_knee_distance * half_knee_distance;
    float foot_height = sqrtf(fmaxf(0.0f, foot_height_sq));

    /* 膝关节连线中点 */
    float mid_knee_x = (left_knee_x + right_knee_x) * 0.5f;
    float mid_knee_y = (left_knee_y + right_knee_y) * 0.5f;

    /* 计算单位法向量 (垂直于膝关节连线) */
    /* 轮腿通常 y 轴向下，这里固定选择指向下方的解 */
    float unit_nx = -knee_distance_y / knee_distance;
    float unit_ny =  knee_distance_x / knee_distance;

    foot->x = mid_knee_x + foot_height * unit_nx;
    foot->y = mid_knee_y + foot_height * unit_ny;

    return 0;
}

/* 逆运动学：已知足端坐标 -> 求电机角 */
int five_bar_inverse(const Foot_position_t *foot,
                    Joint_angle_t *angles){
    const float half_distance = LEG_JOINT_DISTANCE * 0.5f;
    const float thigh_len = LEG_THIGH;
    const float shank_len = LEG_SHANK;
    const float thigh_sq = thigh_len * thigh_len;
    const float shank_sq = shank_len * shank_len;

    /* --- 左侧电机计算 --- */
    float left_joint_x_rel = foot->x + half_distance;
    float left_joint_y_rel = foot->y;   /* 左电机位置相对于足端的坐标 */
    float left_mtf_distance_sq = left_joint_x_rel * left_joint_x_rel + left_joint_y_rel * left_joint_y_rel;
    float left_mtf_distance    = sqrtf(left_mtf_distance_sq);   /* 左电机到足端的距离 */

    if (left_mtf_distance > (thigh_len + shank_len)
        || left_mtf_distance < fabsf(thigh_len - shank_len))
        return -1;

    float base_angle_left = atan2f(left_joint_y_rel, left_joint_x_rel); /* 左电机-足端连线与水平线的夹角 */
    float cos_alpha_left = (thigh_sq + left_mtf_distance_sq - shank_sq)
                            / (2.0f * thigh_len * left_mtf_distance);
    float alpha_left = acosf(fminf(1.0f, fmaxf(-1.0f, cos_alpha_left))); /* 左电机-足端连线与大腿的夹角 */

    /* --- 右侧电机计算 --- */
    float right_joint_x_rel = foot->x - half_distance;
    float right_joint_y_rel = foot->y;
    float right_mtf_distance_sq = right_joint_x_rel * right_joint_x_rel + right_joint_y_rel * right_joint_y_rel;
    float right_mtf_distance    = sqrtf(right_mtf_distance_sq);

    if (right_mtf_distance > (thigh_len + shank_len)
        || right_mtf_distance < fabsf(thigh_len - shank_len))
        return -1;

    float base_angle_right = atan2f(right_joint_y_rel, right_joint_x_rel); /* 右电机-足端连线与水平线的夹角 */
    float cos_alpha_right = (thigh_sq + right_mtf_distance_sq - shank_sq) / (2.0f * thigh_len * right_mtf_distance);
    float alpha_right = acosf(fminf(1.0f, fmaxf(-1.0f, cos_alpha_right))); /* 右电机-足端连线与大腿的夹角 */

    angles->left_motor_angle = base_angle_left + alpha_left;
    angles->right_motor_angle = base_angle_right - alpha_right;

    return 0;
}