#include "vmc_calculate.h"
#include <math.h>

/* PWM 输出限幅值 */
#define VMC_PWM_CLAMP_MAX   10000

static inline float clamp(float value, float min, float max){
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* 只清零当前腿的关节电机，另一条腿保持输出 */
static inline void vmc_clear_leg(LegSide_t leg_side, Motor_cmd_duty_t *motor_cmd){
    if (leg_side == LEG_LEFT) {
        motor_cmd->left_front_joint_pwm = 0;
        motor_cmd->left_back_joint_pwm  = 0;
    } else {
        motor_cmd->right_front_joint_pwm = 0;
        motor_cmd->right_back_joint_pwm  = 0;
    }
}

void vmc_calculate(const VMC_Config_t *vmc_cfg,
                                 const Foot_position_t *foot_tar,
                                 const Joint_angle_t *angles_cur,
                                 float joint_front_speed,
                                 float joint_back_speed,
                                 LegSide_t leg_side,
                                 Motor_cmd_duty_t *motor_cmd){
    /* 1. 正运动学与雅可比计算  */
    Foot_position_t foot_cur;
    if (five_bar_forward(angles_cur, &foot_cur) != 0) {
        /* 奇异/不可达：只清零当前腿，另一条腿保持输出 */
        vmc_clear_leg(leg_side, motor_cmd);
        return;
    }

    float J[2][2];
    if (five_bar_jacobian(angles_cur, J) != 0) {
        vmc_clear_leg(leg_side, motor_cmd);
        return;
    }

    /* 2. 速度映射: 关节角速度 -> 足端笛卡尔速度 */
    float v_cur_x = J[0][0] * joint_front_speed + J[0][1] * joint_back_speed;
    float v_cur_y = J[1][0] * joint_front_speed + J[1][1] * joint_back_speed;

    /* 3. 虚拟力计算 (PD控制，增益从参数结构体读取) */
    /* 注意：这里的 err 指的是足端在空间中的偏差 */
    float Fx = vmc_cfg->kp * (foot_tar->x - foot_cur.x) - vmc_cfg->kd * v_cur_x;
    float Fy = vmc_cfg->kp * (foot_tar->y - foot_cur.y) - vmc_cfg->kd * v_cur_y;

    /* 4. 关节力矩：tau = J^T * F */
    float tau_front  = J[0][0] * Fx + J[1][0] * Fy;
    float tau_back = J[0][1] * Fx + J[1][1] * Fy;

    /* 5. 输出限幅 + 四舍五入（按左右腿选择输出通道） */
    tau_front  = clamp(tau_front,  -VMC_PWM_CLAMP_MAX, VMC_PWM_CLAMP_MAX);
    tau_back = clamp(tau_back, -VMC_PWM_CLAMP_MAX, VMC_PWM_CLAMP_MAX);
    int pwm_front = ROUND(tau_front);
    int pwm_back  = ROUND(tau_back);

    if (leg_side == LEG_LEFT) {
        motor_cmd->left_front_joint_pwm = pwm_front;
        motor_cmd->left_back_joint_pwm  = pwm_back;
    } else {
        motor_cmd->right_front_joint_pwm = pwm_front;
        motor_cmd->right_back_joint_pwm  = pwm_back;
    }
}