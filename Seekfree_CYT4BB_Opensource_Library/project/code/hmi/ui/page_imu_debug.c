#include "page_imu_debug.h"

#include "../../sensors/imu/imu.h"

void page_imu_debug_update(const UI_Frame_t *frame)
{
    (void)frame;

    IMU_Debug_t d;
    imu_get_debug_data(&d);

    ui_scope6_send(d.pitch_deg, d.roll_deg, d.yaw_deg,
                   d.gyro_x_dps, d.gyro_y_dps, d.gyro_z_dps);
}
