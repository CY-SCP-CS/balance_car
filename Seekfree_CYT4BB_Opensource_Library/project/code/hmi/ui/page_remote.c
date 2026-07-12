#include "page_remote.h"

#include <string.h>

#include "zf_common_headfile.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_mt9v03x.h"
#include "../../app/vision/vision_annotate.h"

#define REMOTE_CAMERA_SEND_INTERVAL_TICKS  5u

typedef enum {
    REMOTE_LINK_INIT = 0,
    REMOTE_LINK_WIFI_FAILED,
    REMOTE_LINK_SOCKET_FAILED,
    REMOTE_LINK_READY,
} Remote_Link_State_t;

/* Annotated image buffer — same size as camera frame */
static uint8 g_annotated_image[MT9V03X_H][MT9V03X_W];
static volatile Remote_Link_State_t g_remote_link_state = REMOTE_LINK_INIT;
static uint8 g_camera_send_tick;

static void page_remote_prepare_image(const UI_Frame_t *frame)
{
    SCB_InvalidateDCache_by_Addr(mt9v03x_image_temp[0], MT9V03X_IMAGE_SIZE);

    if (frame->vision != NULL) {
        vision_annotate(g_annotated_image, mt9v03x_image_temp,
                        frame->vision, frame->vision_mode);
    } else {
        (void)memcpy(g_annotated_image[0], mt9v03x_image_temp[0], MT9V03X_IMAGE_SIZE);
    }
}

void page_remote_init(void)
{
    if (wifi_spi_init(REMOTE_WIFI_SSID, REMOTE_WIFI_PASSWORD)) {
        g_remote_link_state = REMOTE_LINK_WIFI_FAILED;
        zf_log(0, "Remote WiFi init failed.");
    } else if (wifi_spi_socket_connect("TCP", REMOTE_TARGET_IP, REMOTE_TARGET_PORT, REMOTE_LOCAL_PORT)) {
        g_remote_link_state = REMOTE_LINK_SOCKET_FAILED;
        zf_log(0, "Remote TCP connect failed.");
    } else {
        g_remote_link_state = REMOTE_LINK_READY;
        zf_log(0, "Remote link ready.");
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        g_annotated_image,
        MT9V03X_W,
        MT9V03X_H);
}

static void apply_remote_params(void)
{
    seekfree_assistant_data_analysis();

    for (uint8 i = 0u; i < REMOTE_PARAM_CHANNELS; i++) {
        if (!seekfree_assistant_parameter_update_flag[i]) {
            continue;
        }
        seekfree_assistant_parameter_update_flag[i] = 0u;
        remote_debug_send_param(i, seekfree_assistant_parameter[i]);
    }
}

void page_remote_update(const UI_Frame_t *frame)
{
    if (g_remote_link_state != REMOTE_LINK_READY) {
        return;
    }

    apply_remote_params();

    g_camera_send_tick++;
    if (g_camera_send_tick >= REMOTE_CAMERA_SEND_INTERVAL_TICKS) {
        g_camera_send_tick = 0u;
        page_remote_prepare_image(frame);
        seekfree_assistant_camera_send();
    }

    const Ctrl_Input_t    *fb         = frame->fb;
    const Nav_Output_t    *nav_output = frame->nav_output;
    const Vision_Result_t *vision     = frame->vision;
    const Nav_State_t     *state      = frame->nav_state;

    ui_scope6_send(
        fb != NULL ? fb->body_pitch * UI_RAD_TO_DEG : 0.0f,
        fb != NULL ? fb->body_roll * UI_RAD_TO_DEG : 0.0f,
        fb != NULL ? fb->gyro_pitch_rate : 0.0f,
        nav_output != NULL ? nav_output->velocity_cmd : 0.0f,
        vision != NULL ? vision->line_offset : 0.0f,
        ui_segment_or_stop(nav_output, state != NULL ? state->segment_index : 0u));
}
