#include "page_debug.h"

#include "seekfree_assistant.h"
#include "../../sensors/imu/imu.h"

void debug_display_init(void)
{
    seekfree_assistant_interface_init(DEBUG_DISPLAY_DEVICE);
}

void debug_display_imu(void)
{
    IMU_Debug_t d;
    imu_get_debug_data(&d);

    seekfree_assistant_oscilloscope_data.channel_num = 6;
    seekfree_assistant_oscilloscope_data.data[0] = d.pitch_deg;
    seekfree_assistant_oscilloscope_data.data[1] = d.roll_deg;
    seekfree_assistant_oscilloscope_data.data[2] = d.yaw_deg;
    seekfree_assistant_oscilloscope_data.data[3] = d.gyro_x_dps;
    seekfree_assistant_oscilloscope_data.data[4] = d.gyro_y_dps;
    seekfree_assistant_oscilloscope_data.data[5] = d.gyro_z_dps;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}
