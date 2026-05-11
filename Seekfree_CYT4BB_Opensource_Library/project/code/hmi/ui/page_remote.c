#include "page_remote.h"

#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_mt9v03x.h"

#define RAD_TO_DEG  (180.0f / 3.14159265f)

/* Parameter channel mapping (upper computer -> nav config)
 * Ch 0 : yaw_kp
 * Ch 1 : turn_kp
 * Ch 2 : landmark_kp
 * Ch 3 : steering_limit
 */
#define REMOTE_PARAM_YAW_KP         0u
#define REMOTE_PARAM_TURN_KP        1u
#define REMOTE_PARAM_LANDMARK_KP    2u
#define REMOTE_PARAM_STEERING_LIMIT 3u
#define REMOTE_PARAM_COUNT          4u

void remote_page_init(void)
{
    wifi_spi_init(REMOTE_WIFI_SSID, REMOTE_WIFI_PASSWORD);
    wifi_spi_socket_connect("TCP", REMOTE_TARGET_IP, REMOTE_TARGET_PORT, REMOTE_LOCAL_PORT);

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        mt9v03x_image,
        MT9V03X_W,
        MT9V03X_H);
}

static void apply_remote_params(void)
{
    seekfree_assistant_data_analysis();

    uint8 dirty = 0u;
    Nav_Config_t cfg = nav_get_config();

    for (uint8 i = 0u; i < REMOTE_PARAM_COUNT; i++) {
        if (!seekfree_assistant_parameter_update_flag[i]) {
            continue;
        }
        seekfree_assistant_parameter_update_flag[i] = 0u;
        dirty = 1u;

        float val = seekfree_assistant_parameter[i];

        switch (i) {
        case REMOTE_PARAM_YAW_KP:         cfg.yaw_kp         = val; break;
        case REMOTE_PARAM_TURN_KP:        cfg.turn_kp        = val; break;
        case REMOTE_PARAM_LANDMARK_KP:    cfg.landmark_kp    = val; break;
        case REMOTE_PARAM_STEERING_LIMIT: cfg.steering_limit = val; break;
        default: break;
        }
    }

    if (dirty) {
        nav_set_config(&cfg);
    }
}

void remote_page_update(const Ctrl_Input_t    *fb,
                        const Nav_Input_t     *nav_input,
                        const Nav_Output_t    *nav_output,
                        const Vision_Result_t *vision)
{
    apply_remote_params();

    seekfree_assistant_camera_send();

    Nav_State_t state = nav_get_state();

    seekfree_assistant_oscilloscope_data.channel_num = 6;
    seekfree_assistant_oscilloscope_data.data[0] =
        fb != NULL ? fb->body_pitch * RAD_TO_DEG : 0.0f;
    seekfree_assistant_oscilloscope_data.data[1] =
        fb != NULL ? fb->body_roll * RAD_TO_DEG : 0.0f;
    seekfree_assistant_oscilloscope_data.data[2] =
        nav_output != NULL ? nav_output->steering_cmd : 0.0f;
    seekfree_assistant_oscilloscope_data.data[3] =
        nav_output != NULL ? nav_output->velocity_cmd : 0.0f;
    seekfree_assistant_oscilloscope_data.data[4] =
        vision != NULL ? vision->line_offset : 0.0f;
    seekfree_assistant_oscilloscope_data.data[5] =
        (nav_output != NULL && nav_output->safety_stop) ? -1.0f : (float)state.segment_index;

    (void)nav_input;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}
