#include "zf_common_headfile.h"
#include "../code/sensors/imu/imu.h"
#include "../code/common/types.h"
#include "../code/app/navigation/nav_engine.h"
#include "../code/app/vision/vision_pipeline.h"
#include "../code/app/robot_control/robot_control.h"
#include "../code/app/robot_control/small_driver_uart_control.h"
#include "../code/app/remote/remote_comm.h"
#include "../code/app/remote/remote_debug.h"
#include "../code/app/robot_control/jump.h"
#include "../code/control/leg/angle_offset.h"

Ctrl_Input_t   g_ctrl;
static Nav_Input_t    g_nav_input;
static Vision_Result_t g_vision;

Motor_cmd_duty_t g_motor_cmd;

Sensor_data_t g_sensor_data;

Move_cmd_t g_move_cmd;

/* Route keys: key0 start/finish record, key1 add point,
 * key2 replay recorded route, key3 stop/cancel. */
static bool remote_key_rising(const Remote_State_t *remote, uint8 index)
{
    static uint8 prev_key[4] = {0u, 0u, 0u, 0u};

    if (remote == NULL || index >= 4u) {
        return false;
    }

    uint8 now = remote->key[index] != 0u ? 1u : 0u;
    bool rising = (now != 0u && prev_key[index] == 0u);
    prev_key[index] = now;

    return rising;
}

static void route_remote_update(const Nav_Input_t *input)
{
    const Remote_State_t *remote = remote_comm_get_state();

    if (input == NULL || remote == NULL || !remote->connected) {
        return;
    }

    if (remote_key_rising(remote, 3u)) {
        Nav_Route_Record_State_t state = nav_route_record_get_state();

        if (state.mode == NAV_ROUTE_RECORDING) {
            nav_route_record_reset();
        } else {
            nav_route_replay_stop();
        }
        return;
    }

    if (remote_key_rising(remote, 0u)) {
        Nav_Route_Record_State_t state = nav_route_record_get_state();

        if (state.mode == NAV_ROUTE_RECORDING) {
            (void)nav_route_record_keypoint(input);
            (void)nav_route_record_finish();
        } else {
            (void)nav_route_record_start(input);
        }
    }

    if (remote_key_rising(remote, 1u)) {
        (void)nav_route_record_keypoint(input);
    }

    if (remote_key_rising(remote, 2u)) {
        Nav_Route_Record_State_t state = nav_route_record_get_state();

        if (state.mode != NAV_ROUTE_RECORDING) {
            (void)nav_route_replay_start(input);
        }
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    remote_debug_init();

    if(!imu_init())
    {
        zf_log(0, "IMU init failed.");
        while(true);
    }

    nav_init(NULL);
    vision_init();

    // UI 运行在 CM7_1，CM7_0 只保留控制/感知/驱动逻辑。
    small_driver_uart_init();
    remote_comm_init();
    robot_control_init();

    angle_offset_start(NULL);               // 标定全部四个关节 (左前/左后/右前/右后)

    pit_ms_init(PIT_CH1, 1);

    interrupt_global_enable(0);

    while (!angle_offset_is_done()) {
        imu_update(&g_ctrl);
        sensor_cmd_update(&g_ctrl, &g_sensor_data, &g_move_cmd);
        system_delay_ms(1);

        if (angle_offset_has_fault()) {
            zf_log(0, "Angle calibration FAILED (timeout). Halting.");
            while (true);
        }
    }
    zf_log(0, "Angle calibration OK.");

    ///jump_start();   // 测试跳跃

    while(true)
    {

        imu_update(&g_ctrl);// IMU update: for testing, can be moved to timer ISR
        // g_ctrl.body_pitch / body_roll / gyro_pitch_rate / gyro_yaw_rate (rad, rad/s)

        remote_comm_update(&g_ctrl);

        vision_update(&g_vision);

        nav_input_update_from_ctrl(&g_nav_input, &g_ctrl);
        route_remote_update(&g_nav_input);

        {
            Nav_Route_Record_State_t route_state = nav_route_record_get_state();

            if (route_state.mode == NAV_ROUTE_REPLAYING) {
                Nav_Output_t nav_out = nav_update(&g_nav_input);
                nav_apply_ctrl(&g_ctrl, &nav_out);
            }
        }

        small_driver_get_angle(&small_driver_value);
        sensor_cmd_update(&g_ctrl, &g_sensor_data, &g_move_cmd);
        system_delay_ms(1);

    }
}
