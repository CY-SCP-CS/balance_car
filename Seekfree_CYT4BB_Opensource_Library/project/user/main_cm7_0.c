#include "zf_common_headfile.h"
#include "../code/sensors/imu/imu.h"
#include "../code/common/types.h"
#include "../code/app/navigation/nav_engine.h"
#include "../code/hmi/ui/page_debug.h"
#include "../code/hmi/ui/page_dashboard.h"

static Ctrl_Input_t g_fb;
static Nav_Input_t g_nav_input;

#define NAV_DEMO_ENABLED       0
#define NAV_LOOP_DT_S          0.001f
#define NAV_LOOP_DT_MS         1u

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

    debug_display_init();

    interrupt_global_enable(0);

    while(true)
    {
        imu_update(&g_fb);
        // g_fb.body_pitch / body_roll / gyro_pitch_rate / gyro_yaw_rate (rad, rad/s)

        nav_input_update_from_feedback(&g_nav_input,
                                       &g_fb,
                                       NAV_LOOP_DT_S,
                                       NAV_LOOP_DT_MS,
                                       (NAV_DEMO_ENABLED != 0));
        Nav_Output_t nav_out = nav_update(&g_nav_input);
        nav_apply_feedback(&g_fb, &nav_out);

        dashboard_update(&g_fb, &g_nav_input, &nav_out);
        // Dashboard CH1:pitch CH2:roll CH3:gyro_pitch_rate
        //           CH4:velocity_cmd CH5:steering_cmd CH6:segment_index/safety_stop

    }
}
