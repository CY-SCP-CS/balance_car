#include "leg_vmc_control.h"
#include "jacobian.h"
#include "vmc_calculate.h"
#include <math.h>

/* ─── 标称位形偏置 (与 PID 方案保持一致) ─── */
#define LEG_NOM_FRONT_L      0.5f
#define LEG_NOM_BACK_L      -0.5f
#define LEG_NOM_FRONT_R     -0.5f
#define LEG_NOM_BACK_R       0.5f

/* 右腿镜像安装符号 (与 PID 方案保持一致) */
#define RIGHT_ABS_FRONT_SIGN  -1.0f
#define RIGHT_ABS_BACK_SIGN   -1.0f

#ifndef M_PI
#define M_PI  3.141592653589793f
#endif
#define M_PI_2  (M_PI / 2.0f)

/* VMC 虚拟刚度/阻尼默认值 */
#define VMC_KP_DEFAULT       0.3f
#define VMC_KD_DEFAULT       0.001f

/* ─── 内部工具 ─────────────────────────────────────────────── */

/**
 * @brief 将一条腿的校准后传感器读数转换为运动学使用的绝对角度
 *
 * 坐标系约定（与 kinematics.c 相同）：
 *   - 坐标原点 = 两电机连线中点
 *   - +X = 前方, +Y = 下方 (重力方向)
 *   - 角度从 +X 起算, CCW 为正
 *
 * 物理安装：限位处传感器读数为 0（经 angle_offset 标定后），
 * 限位对应的绝对角度 ≈ π/2（大腿竖直向下）。
 * 因此: 绝对角度 = π/2 + (传感器读数 × 符号因子)
 *
 * 左腿：前关节直接读数，后关节取反（电机安装方向导致）
 * 右腿：镜像安装，前/后关节均乘 RIGHT_ABS_*_SIGN
 */
static void sensor_to_abs_angle(const Sensor_data_t *sensor,
                                LegSide_t side,
                                Joint_angle_t *abs_cur) {
    if (side == LEG_LEFT) {
        abs_cur->left_motor_angle  = (float)M_PI_2 + sensor->joint_left_front_angle;
        abs_cur->right_motor_angle = (float)M_PI_2 + (-sensor->joint_left_back_angle);
    } else {
        abs_cur->left_motor_angle  = (float)M_PI_2 + RIGHT_ABS_FRONT_SIGN * sensor->joint_right_front_angle;
        abs_cur->right_motor_angle = (float)M_PI_2 + RIGHT_ABS_BACK_SIGN  * sensor->joint_right_back_angle;
    }
}

/**
 * @brief 获取标称位形的绝对角度
 */
static void get_nominal_abs(LegSide_t side, Joint_angle_t *abs_nom) {
    if (side == LEG_LEFT) {
        abs_nom->left_motor_angle  = (float)M_PI_2 + LEG_NOM_FRONT_L;
        abs_nom->right_motor_angle = (float)M_PI_2 + LEG_NOM_BACK_L;
    } else {
        abs_nom->left_motor_angle  = (float)M_PI_2 + RIGHT_ABS_FRONT_SIGN * LEG_NOM_FRONT_R;
        abs_nom->right_motor_angle = (float)M_PI_2 + RIGHT_ABS_BACK_SIGN  * LEG_NOM_BACK_R;
    }
}

/* ─── VMC 腿部控制核心 ─────────────────────────────────────── */

void leg_vmc_control(VMC_Config_t *vmc_cfg,
                     const Sensor_data_t *sensor,
                     const Foot_position_t *foot_left_cmd,
                     const Foot_position_t *foot_right_cmd,
                     Motor_cmd_duty_t *motor_cmd) {
    /* 静态变量：关节角速度用差分法计算 */
    static float prev_abs[4] = {0, 0, 0, 0};
    static int   first_call  = 1;

    /* ─────────────── 左腿 ─────────────── */
    {
        /* 1. 传感器 → 绝对角度 */
        Joint_angle_t abs_cur;
        sensor_to_abs_angle(sensor, LEG_LEFT, &abs_cur);

        /* 2. 关节角速度 (rad/s) */
        float dth_front, dth_back;
        if (first_call) {
            dth_front = 0.0f;
            dth_back  = 0.0f;
            prev_abs[0] = abs_cur.left_motor_angle;
            prev_abs[1] = abs_cur.right_motor_angle;
        } else {
            dth_front = (abs_cur.left_motor_angle  - prev_abs[0]) / ROBOT_CONTROL_DT;
            dth_back  = (abs_cur.right_motor_angle - prev_abs[1]) / ROBOT_CONTROL_DT;
            prev_abs[0] = abs_cur.left_motor_angle;
            prev_abs[1] = abs_cur.right_motor_angle;
        }

        /* 3. 标称足端位置 (标称位形下的正解) */
        Joint_angle_t abs_nom;
        Foot_position_t nominal_foot;
        get_nominal_abs(LEG_LEFT, &abs_nom);
        if (five_bar_forward(&abs_nom, &nominal_foot) != 0) {
            /* 标称位形奇异？不应当发生，但以防万一 */
            motor_cmd->left_front_joint_pwm = 0;
            motor_cmd->left_back_joint_pwm  = 0;
            goto right_leg;
        }

        /* 4. VMC 目标 = 标称足端 + 控制指令偏移 */
        Foot_position_t target;
        target.x = nominal_foot.x + foot_left_cmd->x;
        target.y = nominal_foot.y + foot_left_cmd->y;

        /* 5. 调用 VMC 核心 (内含正运动学、雅可比、J^T 力控) */
        vmc_calculate(vmc_cfg, &target, &abs_cur,
                      dth_front, dth_back, LEG_LEFT, motor_cmd);
    }

right_leg:
    /* ─────────────── 右腿 ─────────────── */
    {
        Joint_angle_t abs_cur;
        sensor_to_abs_angle(sensor, LEG_RIGHT, &abs_cur);

        float dth_front, dth_back;
        if (first_call) {
            dth_front = 0.0f;
            dth_back  = 0.0f;
            prev_abs[2] = abs_cur.left_motor_angle;
            prev_abs[3] = abs_cur.right_motor_angle;
        } else {
            dth_front = (abs_cur.left_motor_angle  - prev_abs[2]) / ROBOT_CONTROL_DT;
            dth_back  = (abs_cur.right_motor_angle - prev_abs[3]) / ROBOT_CONTROL_DT;
            prev_abs[2] = abs_cur.left_motor_angle;
            prev_abs[3] = abs_cur.right_motor_angle;
        }

        Joint_angle_t abs_nom;
        Foot_position_t nominal_foot;
        get_nominal_abs(LEG_RIGHT, &abs_nom);
        if (five_bar_forward(&abs_nom, &nominal_foot) != 0) {
            motor_cmd->right_front_joint_pwm = 0;
            motor_cmd->right_back_joint_pwm  = 0;
            goto done;
        }

        Foot_position_t target;
        target.x = nominal_foot.x + foot_right_cmd->x;
        target.y = nominal_foot.y + foot_right_cmd->y;

        vmc_calculate(vmc_cfg, &target, &abs_cur,
                      dth_front, dth_back, LEG_RIGHT, motor_cmd);
    }

done:
    first_call = 0;
}
