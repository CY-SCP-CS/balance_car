#include "zf_common_headfile.h"
#include "../code/sensors/imu/imu.h"
#include "../code/common/types.h"
#include "../code/app/navigation/nav_engine.h"
#include "../code/hmi/ui/ui_manager.h"

static Ctrl_Input_t g_ctrl;
static Nav_Input_t g_nav_input;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    if(!imu_init())
    {
        zf_log(0, "IMU init failed.");
        while(true);
    }

    nav_init(NULL);

    ui_init(UI_PAGE_DASHBOARD);//UI_PAGE_IMU_DEBUG UI_PAGE_NAV_DEBUG

    interrupt_global_enable(0);

    while(true)
    {
        imu_update(&g_ctrl);
        // g_ctrl.body_pitch / body_roll / gyro_pitch_rate / gyro_yaw_rate (rad, rad/s)

        nav_input_update_from_feedback(&g_nav_input, &g_ctrl);
        Nav_Output_t nav_out = nav_update(&g_nav_input);
        nav_apply_feedback(&g_ctrl, &nav_out);

        ui_update(&g_ctrl, &g_nav_input, &nav_out);
        // Dashboard CH1:pitch CH2:roll CH3:gyro_pitch_rate
        //           CH4:velocity_cmd CH5:steering_cmd CH6:segment_index/safety_stop

    }
}
