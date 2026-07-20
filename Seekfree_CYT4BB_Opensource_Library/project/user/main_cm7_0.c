#include "zf_common_headfile.h"
#include "../code/sensors/imu/imu.h"
#include "../code/common/types.h"
#include "../code/app/navigation/nav_engine.h"
#include "../code/app/navigation/nav_adapter.h"
#include "../code/app/navigation/nav_route_record.h"
#include "../code/app/navigation/route_remote.h"
#include "../code/app/vision/vision_pipeline.h"
#include "../code/app/robot_control/robot_control.h"
#include "../code/app/robot_control/small_driver_uart_control.h"
#include "../code/app/remote/remote_comm.h"
#include "../code/app/remote/remote_debug.h"
#include "../code/app/robot_control/jump.h"
#include "../code/app/robot_control/track_elements.h"
#include "../code/control/leg/angle_offset.h"
#include "../code/hmi/indicator/led_buzzer.h"

/* 手动测试开关: 标定完成后直接激活颠簸路段, 不依赖导航 */
#define BUMPY_MANUAL_TEST 0

#define CM7_0_READY_MAGIC 0x43373031u

#pragma location = 0x28006C00
__no_init volatile uint32 g_cm7_0_ready;

static void cm7_0_set_ready(uint32 value)
{
    g_cm7_0_ready = value;
    SCB_CleanDCache_by_Addr((uint32 *)&g_cm7_0_ready, 32u);
}

Ctrl_Input_t   g_ctrl;
static Nav_Input_t    g_nav_input;
static Vision_Result_t g_vision;

Motor_cmd_duty_t g_motor_cmd;

Sensor_data_t g_sensor_data;

Move_cmd_t g_move_cmd;

int main(void)
{
    g_cm7_0_ready = 0u;
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    cm7_0_set_ready(0u);
    zf_log(0, "CM7_1 debugger-managed.");
    remote_debug_init();

    if(!imu_init())
    {
        zf_log(0, "IMU init failed.");
        while(true);
    }

    nav_init(NULL);
    vision_init();
    vision_set_mode(VISION_MODE_MINEFIELD);
    cm7_0_set_ready(CM7_0_READY_MAGIC);

    // UI 运行在 CM7_1，CM7_0 只保留控制/感知/驱动逻辑。
    small_driver_uart_init();
    led_buzzer_init();
    remote_comm_init();
    robot_control_init();

    angle_offset_start(NULL);               // 标定全部四个关节 (左前/左后/右前/右后)

    pit_ms_init(PIT_CH1, 1);

    interrupt_global_enable(0);

    while (!angle_offset_is_done()) {
        imu_update(&g_ctrl);
        sensor_cmd_update(&g_ctrl, &g_sensor_data, &g_move_cmd);
        led_buzzer_tick(1u);
        system_delay_ms(1);

        if (angle_offset_has_fault()) {
            zf_log(0, "Angle calibration FAILED (timeout). Halting.");
            while (true);
        }
    }
    zf_log(0, "Angle calibration OK.");

#if BUMPY_MANUAL_TEST
    track_bumpy_activate();
    track_bumpy_apply_compliance();
    zf_log(0, "Bumpy manual test started (Mode A).");
#else
    //track_rotate720_start();   // 测试旋转720
#endif
    jump_start(-0.0f, 1);//三级台阶
    //jump_start(-1.20f, 1);//直接跳过颠簸

    while(true)
    {

        imu_update(&g_ctrl);// IMU update: for testing, can be moved to timer ISR
        // g_ctrl.body_pitch / body_roll / gyro_pitch_rate / gyro_yaw_rate (rad, rad/s)

        remote_comm_update(&g_ctrl);

        vision_update(&g_vision);

        nav_input_update_from_ctrl(&g_nav_input, &g_ctrl);
        vision_feed_nav_input(&g_nav_input, &g_vision);
        route_remote_update(&g_nav_input);

        {
            Nav_Route_Record_State_t route_state = nav_route_record_get_state();

            if (route_state.mode == NAV_ROUTE_REPLAYING) {
                Nav_Output_t nav_out = nav_route_replay_update(&g_nav_input);
                nav_apply_ctrl(&g_ctrl, &nav_out);

                /* ── 赛道元素区域响应 ── */
                if (nav_out.region_entered) {
                    switch (nav_out.region) {
                    case NAV_REGION_ROTATE:
                        track_rotate720_start();
                        break;
                    case NAV_REGION_JUMP:
                        jump_start(-0.15f, 3);
                        break;
                    case NAV_REGION_SINGLE_BRIDGE:
                    case NAV_REGION_UPHILL:
                        track_bridge_climb_activate();
                        break;
                    case NAV_REGION_SPEED_BUMP:
                        track_bumpy_activate();
                        track_bumpy_apply_compliance();
                        break;
                    default:
                        break;
                    }
                }
                if (nav_out.region_exited) {
                    switch (nav_out.region) {
                    case NAV_REGION_SINGLE_BRIDGE:
                    case NAV_REGION_UPHILL:
                        track_bridge_climb_deactivate();
                        break;
                    case NAV_REGION_SPEED_BUMP:
                        track_bumpy_restore_stiffness();
                        track_bumpy_deactivate();
                        break;
                    case NAV_REGION_ROTATE:
                        track_rotate720_reset();
                        break;
                    default:
                        break;
                    }
                }
                if (nav_out.waypoint_entered &&
                    nav_out.waypoint_action ==
                    NAV_ROUTE_POINT_ACTION_ROTATE720) {
                    track_rotate720_reset();
                    track_rotate720_start();
                }
            }
        }

        if (track_rotate720_is_active()) {
            g_ctrl.velocity_cmd = 0.0f;
        }
        if (track_rotate720_is_done()) {
            track_rotate720_reset();
        }

        small_driver_get_angle(&small_driver_value);
        sensor_cmd_update(&g_ctrl, &g_sensor_data, &g_move_cmd);
        led_buzzer_tick(1u);
        system_delay_ms(1);

    }
}
