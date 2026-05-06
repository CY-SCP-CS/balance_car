#ifndef SENSORS_IMU_H
#define SENSORS_IMU_H

#include <stdbool.h>
#include "../../common/types.h"

// ===== 模式选择 =====
// 1: INT2 引脚已连接 → 四元数融合 + EXTI 中断驱动（精度高，延迟低）
// 0: INT2 引脚未连接 → 陀螺仪+加速度计轮询 + 互补滤波（占用 PIT_CH0 @ 1ms）
#define IMU_INT2_ENABLED    1

// ===== 轮询模式参数（IMU_INT2_ENABLED = 0 时生效）=====
// 必须与 PIT_CH0 的初始化周期一致
#define IMU_POLL_DT         0.001f      // 单位：秒，对应 pit_ms_init(PIT_CH0, 1)

// 互补滤波系数：越大越信任陀螺仪积分，越小越信任加速度计
#define IMU_COMP_ALPHA      0.98f

// 初始化 IMU
// INT2 模式：imu660rc_init(480Hz) + EXTI 由库内部配置
// 轮询模式：imu660rc_init(DISABLE) + pit_ms_init(PIT_CH0, 1) 自动初始化
// 返回 true 表示成功
bool imu_init(void);

// 将最新 IMU 数据写入 Feedback_Data_t
// 所有角度单位 rad，角速度单位 rad/s
// 陀螺仪轴映射假设：Y 轴 = 俯仰，Z 轴 = 偏航；如安装方向不同，修改 imu.c 中映射
void imu_update(Feedback_Data_t *fb);

#if !IMU_INT2_ENABLED
// 从 PIT_CH0 的 ISR 中调用，读取陀螺仪+加速度计并更新互补滤波
void imu_poll(void);
#endif

#endif
