#include "imu.h"

#include <math.h>
#include "zf_device_imu660rc.h"
#include "zf_driver_pit.h"

#define DEG_TO_RAD  (3.14159265f / 180.0f)
#define RAD_TO_DEG  (180.0f / 3.14159265f)

// ============================================================
// INT2 中断模式（IMU_INT2_ENABLED = 1）
// ============================================================
#if IMU_INT2_ENABLED

bool imu_init(void)
{
    return imu660rc_init(IMU660RC_QUARTERNION_480HZ) == 0;
    // 库内部已调用 exti_init(IMU660RC_INT2_PIN, EXTI_TRIGGER_RISING)
    // 需在 gpio_6_exti_isr() 中调用 imu660rc_callback()
}

void imu_update(Ctrl_Input_t *fb)
{
    fb->body_pitch      = imu660rc_pitch * DEG_TO_RAD;
    fb->body_roll       = imu660rc_roll  * DEG_TO_RAD;
    fb->body_yaw        = imu660rc_yaw   * DEG_TO_RAD;

    // 陀螺仪轴映射：Y → 俯仰角速度，Z → 偏航角速度
    // 安装方向不同时修改 imu660rc_gyro_x/y/z 的对应关系
    fb->gyro_pitch_rate = imu660rc_gyro_transition(imu660rc_gyro_y) * DEG_TO_RAD;
    fb->gyro_yaw_rate   = imu660rc_gyro_transition(imu660rc_gyro_z) * DEG_TO_RAD;
    fb->gyro_roll_rate  = imu660rc_gyro_transition(imu660rc_gyro_x) * DEG_TO_RAD;
    fb->accel_z         = imu660rc_acc_transition(imu660rc_acc_z);
}

void imu_get_debug_data(IMU_Debug_t *out)
{
    out->pitch_deg  = imu660rc_pitch;
    out->roll_deg   = imu660rc_roll;
    out->yaw_deg    = imu660rc_yaw;
    out->gyro_x_dps = imu660rc_gyro_transition(imu660rc_gyro_x);
    out->gyro_y_dps = imu660rc_gyro_transition(imu660rc_gyro_y);
    out->gyro_z_dps = imu660rc_gyro_transition(imu660rc_gyro_z);
}

// ============================================================
// 轮询 + 互补滤波模式（IMU_INT2_ENABLED = 0）
// ============================================================
#else

static float g_pitch           = 0.0f;
static float g_roll            = 0.0f;
static float g_accel_z         = 0.0f;
static float g_gyro_pitch_rate = 0.0f;
static float g_gyro_yaw_rate   = 0.0f;

bool imu_init(void)
{
    // 不使用片上四元数融合，避免库配置 EXTI（悬空引脚）
    bool ok = (imu660rc_init(IMU660RC_QUARTERNION_DISABLE) == 0);

    // 用 PIT_CH0 定时触发 imu_poll()，周期须与 IMU_POLL_DT 一致
    pit_ms_init(PIT_CH0, 1);    // 1 ms → 1 kHz

    return ok;
}

// 在 pit0_ch0_isr() 中调用，运行时间约 20~30 us（SPI 读取 + 计算）
void imu_poll(void)
{
    imu660rc_get_gyro();
    imu660rc_get_acc();

    // 转换为物理量（单位：g 和 deg/s）
    float ax = imu660rc_acc_transition(imu660rc_acc_x);
    float ay = imu660rc_acc_transition(imu660rc_acc_y);
    float az = imu660rc_acc_transition(imu660rc_acc_z);

    g_accel_z = az;

    // 陀螺仪轴映射：Y → 俯仰，X → 横滚，Z → 偏航
    float gyro_pitch = imu660rc_gyro_transition(imu660rc_gyro_y) * DEG_TO_RAD;
    float gyro_roll  = imu660rc_gyro_transition(imu660rc_gyro_x) * DEG_TO_RAD;
    g_gyro_yaw_rate  = imu660rc_gyro_transition(imu660rc_gyro_z) * DEG_TO_RAD;

    // 由加速度计计算静态倾角（单位：rad）
    // 假设 X 轴朝前、Z 轴朝上；安装不同时修改下方公式
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float acc_roll  = atan2f( ay, az);

    // 互补滤波：高频信任陀螺积分，低频信任加速度计修正漂移
    g_pitch = IMU_COMP_ALPHA * (g_pitch + gyro_pitch * IMU_POLL_DT)
            + (1.0f - IMU_COMP_ALPHA) * acc_pitch;
    g_roll  = IMU_COMP_ALPHA * (g_roll  + gyro_roll  * IMU_POLL_DT)
            + (1.0f - IMU_COMP_ALPHA) * acc_roll;

    g_gyro_pitch_rate = gyro_pitch;
}

void imu_update(Ctrl_Input_t *fb)
{
    fb->body_pitch      = g_pitch;
    fb->body_roll       = g_roll;
    fb->accel_z         = g_accel_z;
    fb->gyro_pitch_rate = g_gyro_pitch_rate;
    fb->gyro_yaw_rate   = g_gyro_yaw_rate;
}

void imu_get_debug_data(IMU_Debug_t *out)
{
    out->pitch_deg  = g_pitch * RAD_TO_DEG;
    out->roll_deg   = g_roll  * RAD_TO_DEG;
    out->yaw_deg    = 0.0f;   // 互补滤波无绝对偏航参考
    out->gyro_x_dps = imu660rc_gyro_transition(imu660rc_gyro_x);
    out->gyro_y_dps = imu660rc_gyro_transition(imu660rc_gyro_y);
    out->gyro_z_dps = imu660rc_gyro_transition(imu660rc_gyro_z);
}

#endif
