#include "page_remote.h"

#include <string.h>

#include "zf_common_headfile.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_mt9v03x.h"
#include "../../app/vision/minefield_detect.h"
#include "../../app/vision/vision_annotate.h"

#define REMOTE_CAMERA_SEND_INTERVAL_TICKS  5u
#define REMOTE_RECONNECT_INTERVAL_TICKS    200u
#define REMOTE_ELEMENT_BLACK_MAX           60u
#define REMOTE_ELEMENT_WHITE_MIN           200u
#define REMOTE_ELEMENT_MIN_AREA            18u
#define REMOTE_ELEMENT_SQUARE_MIN_SIDE     8u
#define REMOTE_ELEMENT_SQUARE_MAX_SIDE     90u
#define REMOTE_ELEMENT_BAR_MIN_LENGTH      18u
#define REMOTE_ELEMENT_BAR_MAX_THICKNESS   24u
#define REMOTE_ELEMENT_VISITED_BYTES       ((MT9V03X_IMAGE_SIZE + 7u) / 8u)

typedef enum {
    REMOTE_LINK_INIT = 0,
    REMOTE_LINK_WIFI_FAILED,
    REMOTE_LINK_SOCKET_FAILED,
    REMOTE_LINK_READY,
} Remote_Link_State_t;

typedef enum {
    REMOTE_ELEMENT_TONE_BLACK = 0,
    REMOTE_ELEMENT_TONE_WHITE,
} Remote_Element_Tone_t;

typedef struct {
    bool   valid;
    uint16 r0;
    uint16 c0;
    uint16 r1;
    uint16 c1;
    uint16 width;
    uint16 height;
    uint32 area;
    bool   touches_border;
} Remote_Component_t;

typedef struct {
    Remote_Component_t black_square;
    Remote_Component_t black_bar;
    Remote_Component_t white_bar;
} Remote_Element_Result_t;

/* Annotated image buffer — same size as camera frame */
static uint8 g_annotated_image[MT9V03X_H][MT9V03X_W];
static uint8 g_element_visited[REMOTE_ELEMENT_VISITED_BYTES];
static uint16 g_element_stack[MT9V03X_IMAGE_SIZE];
static volatile Remote_Link_State_t g_remote_link_state = REMOTE_LINK_INIT;
static uint8 g_camera_send_tick;
static uint16 g_remote_reconnect_tick;

static bool remote_element_pixel_match(uint8 px, Remote_Element_Tone_t tone)
{
    if (tone == REMOTE_ELEMENT_TONE_BLACK) {
        return px <= REMOTE_ELEMENT_BLACK_MAX;
    }
    return px >= REMOTE_ELEMENT_WHITE_MIN;
}

static uint16 remote_element_pixel_index(uint16 row, uint16 col)
{
    return (uint16)(row * MT9V03X_W + col);
}

static bool remote_element_visited_get(uint16 row, uint16 col)
{
    uint16 index = remote_element_pixel_index(row, col);
    uint8 mask = (uint8)(1u << (index & 7u));
    return (g_element_visited[index >> 3] & mask) != 0u;
}

static void remote_element_visited_set(uint16 row, uint16 col)
{
    uint16 index = remote_element_pixel_index(row, col);
    uint8 mask = (uint8)(1u << (index & 7u));
    g_element_visited[index >> 3] |= mask;
}

static void remote_element_try_push(const uint8 src[MT9V03X_H][MT9V03X_W],
                                    Remote_Element_Tone_t tone,
                                    uint16 row,
                                    uint16 col,
                                    uint16 *stack_size)
{
    if (remote_element_visited_get(row, col)) {
        return;
    }
    if (!remote_element_pixel_match(src[row][col], tone)) {
        return;
    }
    if (*stack_size >= (uint16)MT9V03X_IMAGE_SIZE) {
        return;
    }

    remote_element_visited_set(row, col);
    g_element_stack[*stack_size] = remote_element_pixel_index(row, col);
    (*stack_size)++;
}

static Remote_Component_t remote_element_trace_component(
    const uint8 src[MT9V03X_H][MT9V03X_W],
    Remote_Element_Tone_t tone,
    uint16 start_row,
    uint16 start_col)
{
    Remote_Component_t comp;
    uint16 stack_size = 0u;

    comp.valid = true;
    comp.r0 = start_row;
    comp.c0 = start_col;
    comp.r1 = start_row;
    comp.c1 = start_col;
    comp.width = 1u;
    comp.height = 1u;
    comp.area = 0u;
    comp.touches_border = false;

    remote_element_try_push(src, tone, start_row, start_col, &stack_size);

    while (stack_size > 0u) {
        uint16 index;
        uint16 row;
        uint16 col;

        stack_size--;
        index = g_element_stack[stack_size];
        row = (uint16)(index / MT9V03X_W);
        col = (uint16)(index - row * MT9V03X_W);

        comp.area++;
        if (row < comp.r0) { comp.r0 = row; }
        if (row > comp.r1) { comp.r1 = row; }
        if (col < comp.c0) { comp.c0 = col; }
        if (col > comp.c1) { comp.c1 = col; }
        if (row == 0u || row == (uint16)(MT9V03X_H - 1u) ||
            col == 0u || col == (uint16)(MT9V03X_W - 1u)) {
            comp.touches_border = true;
        }

        if (row > 0u) {
            remote_element_try_push(src, tone, (uint16)(row - 1u), col, &stack_size);
        }
        if (row < (uint16)(MT9V03X_H - 1u)) {
            remote_element_try_push(src, tone, (uint16)(row + 1u), col, &stack_size);
        }
        if (col > 0u) {
            remote_element_try_push(src, tone, row, (uint16)(col - 1u), &stack_size);
        }
        if (col < (uint16)(MT9V03X_W - 1u)) {
            remote_element_try_push(src, tone, row, (uint16)(col + 1u), &stack_size);
        }
    }

    comp.width = (uint16)(comp.c1 - comp.c0 + 1u);
    comp.height = (uint16)(comp.r1 - comp.r0 + 1u);
    return comp;
}

static bool remote_element_component_usable(const Remote_Component_t *comp)
{
    if (!comp->valid || comp->touches_border) {
        return false;
    }
    if (comp->area < REMOTE_ELEMENT_MIN_AREA) {
        return false;
    }
    if (comp->width < 2u || comp->height < 2u) {
        return false;
    }
    return true;
}

static bool remote_element_is_square(const Remote_Component_t *comp)
{
    uint16 short_side;
    uint16 long_side;
    uint32 box_area;

    if (!remote_element_component_usable(comp)) {
        return false;
    }

    short_side = comp->width < comp->height ? comp->width : comp->height;
    long_side = comp->width > comp->height ? comp->width : comp->height;

    if (short_side < REMOTE_ELEMENT_SQUARE_MIN_SIDE ||
        long_side > REMOTE_ELEMENT_SQUARE_MAX_SIDE) {
        return false;
    }
    if ((uint32)long_side * 100u > (uint32)short_side * 170u) {
        return false;
    }

    box_area = (uint32)comp->width * (uint32)comp->height;
    return comp->area * 100u >= box_area * 4u;
}

static bool remote_element_is_bar(const Remote_Component_t *comp)
{
    uint16 short_side;
    uint16 long_side;
    uint32 box_area;

    if (!remote_element_component_usable(comp)) {
        return false;
    }

    short_side = comp->width < comp->height ? comp->width : comp->height;
    long_side = comp->width > comp->height ? comp->width : comp->height;
    if (short_side == 0u) {
        return false;
    }

    if (long_side < REMOTE_ELEMENT_BAR_MIN_LENGTH ||
        short_side > REMOTE_ELEMENT_BAR_MAX_THICKNESS) {
        return false;
    }
    if ((uint32)long_side < (uint32)short_side * 3u) {
        return false;
    }

    box_area = (uint32)comp->width * (uint32)comp->height;
    return comp->area * 100u >= box_area * 25u;
}

static uint32 remote_element_square_score(const Remote_Component_t *comp)
{
    return (uint32)comp->width * (uint32)comp->height;
}

static uint32 remote_element_bar_score(const Remote_Component_t *comp)
{
    uint16 long_side = comp->width > comp->height ? comp->width : comp->height;
    return (uint32)long_side * 100u + comp->area;
}

static void remote_element_update_best(Remote_Component_t *best,
                                       const Remote_Component_t *candidate,
                                       uint32 candidate_score,
                                       bool *best_valid,
                                       uint32 *best_score)
{
    if (!*best_valid || candidate_score > *best_score) {
        *best = *candidate;
        *best_valid = true;
        *best_score = candidate_score;
    }
}

static void remote_element_scan_tone(const uint8 src[MT9V03X_H][MT9V03X_W],
                                     Remote_Element_Tone_t tone,
                                     Remote_Element_Result_t *result)
{
    bool best_black_square_valid = result->black_square.valid;
    bool best_black_bar_valid = result->black_bar.valid;
    bool best_white_bar_valid = result->white_bar.valid;
    uint32 best_black_square_score = 0u;
    uint32 best_black_bar_score = 0u;
    uint32 best_white_bar_score = 0u;

    (void)memset(g_element_visited, 0, sizeof(g_element_visited));

    for (uint16 row = 0u; row < MT9V03X_H; row++) {
        for (uint16 col = 0u; col < MT9V03X_W; col++) {
            Remote_Component_t comp;

            if (remote_element_visited_get(row, col)) {
                continue;
            }
            if (!remote_element_pixel_match(src[row][col], tone)) {
                continue;
            }

            comp = remote_element_trace_component(src, tone, row, col);

            if (tone == REMOTE_ELEMENT_TONE_BLACK) {
                if (remote_element_is_square(&comp)) {
                    remote_element_update_best(&result->black_square, &comp,
                                               remote_element_square_score(&comp),
                                               &best_black_square_valid,
                                               &best_black_square_score);
                }
                if (remote_element_is_bar(&comp)) {
                    remote_element_update_best(&result->black_bar, &comp,
                                               remote_element_bar_score(&comp),
                                               &best_black_bar_valid,
                                               &best_black_bar_score);
                }
            } else if (remote_element_is_bar(&comp)) {
                remote_element_update_best(&result->white_bar, &comp,
                                           remote_element_bar_score(&comp),
                                           &best_white_bar_valid,
                                           &best_white_bar_score);
            }
        }
    }
}

static void remote_element_detect(const uint8 src[MT9V03X_H][MT9V03X_W],
                                  Remote_Element_Result_t *result)
{
    (void)memset(result, 0, sizeof(*result));
    remote_element_scan_tone(src, REMOTE_ELEMENT_TONE_BLACK, result);
    remote_element_scan_tone(src, REMOTE_ELEMENT_TONE_WHITE, result);
}

static void remote_element_set_px(uint8 img[MT9V03X_H][MT9V03X_W],
                                  int16 row,
                                  int16 col,
                                  uint8 value)
{
    if (row < 0 || row >= (int16)MT9V03X_H ||
        col < 0 || col >= (int16)MT9V03X_W) {
        return;
    }
    img[row][col] = value;
}

static void remote_element_draw_hline(uint8 img[MT9V03X_H][MT9V03X_W],
                                      int16 row,
                                      int16 c0,
                                      int16 c1,
                                      uint8 value)
{
    if (c0 > c1) {
        int16 tmp = c0;
        c0 = c1;
        c1 = tmp;
    }
    for (int16 col = c0; col <= c1; col++) {
        remote_element_set_px(img, row, col, value);
    }
}

static void remote_element_draw_vline(uint8 img[MT9V03X_H][MT9V03X_W],
                                      int16 col,
                                      int16 r0,
                                      int16 r1,
                                      uint8 value)
{
    if (r0 > r1) {
        int16 tmp = r0;
        r0 = r1;
        r1 = tmp;
    }
    for (int16 row = r0; row <= r1; row++) {
        remote_element_set_px(img, row, col, value);
    }
}

static void remote_element_draw_rect(uint8 img[MT9V03X_H][MT9V03X_W],
                                     const Remote_Component_t *comp,
                                     uint8 value)
{
    for (int16 inset = 0; inset < 2; inset++) {
        int16 r0 = (int16)comp->r0 + inset;
        int16 c0 = (int16)comp->c0 + inset;
        int16 r1 = (int16)comp->r1 - inset;
        int16 c1 = (int16)comp->c1 - inset;

        if (r0 > r1 || c0 > c1) {
            break;
        }

        remote_element_draw_hline(img, r0, c0, c1, value);
        remote_element_draw_hline(img, r1, c0, c1, value);
        remote_element_draw_vline(img, c0, r0, r1, value);
        remote_element_draw_vline(img, c1, r0, r1, value);
    }
}

static void remote_element_draw_result(uint8 img[MT9V03X_H][MT9V03X_W],
                                       const Remote_Element_Result_t *result)
{
    if (result->black_square.valid) {
        remote_element_draw_rect(img, &result->black_square, 255u);
    }
    if (result->black_bar.valid) {
        remote_element_draw_rect(img, &result->black_bar, 255u);
    }
    if (result->white_bar.valid) {
        remote_element_draw_rect(img, &result->white_bar, 0u);
    }
}

static void remote_element_detect_and_draw(uint8 dst[MT9V03X_H][MT9V03X_W],
                                           const uint8 src[MT9V03X_H][MT9V03X_W])
{
    Remote_Element_Result_t result;

    remote_element_detect(src, &result);
    remote_element_draw_result(dst, &result);
}

static void page_remote_prepare_image(const UI_Frame_t *frame)
{
    Vision_Result_t annotate_result;

    SCB_InvalidateDCache_by_Addr(mt9v03x_image_temp[0], MT9V03X_IMAGE_SIZE);

    if (frame->vision != NULL) {
        annotate_result = *frame->vision;
        if (frame->vision_mode == VISION_MODE_MINEFIELD) {
            minefield_detect(mt9v03x_image_temp, &annotate_result.minefield);
        }
        vision_annotate(g_annotated_image, mt9v03x_image_temp,
                        &annotate_result, frame->vision_mode);
    } else {
        (void)memcpy(g_annotated_image[0], mt9v03x_image_temp[0], MT9V03X_IMAGE_SIZE);
    }

    remote_element_detect_and_draw(g_annotated_image, mt9v03x_image_temp);
}

static void remote_link_try_connect(bool init_wifi)
{
    if (init_wifi && wifi_spi_init(REMOTE_WIFI_SSID, REMOTE_WIFI_PASSWORD)) {
        g_remote_link_state = REMOTE_LINK_WIFI_FAILED;
        zf_log(0, "Remote WiFi init failed.");
        return;
    }

    if (wifi_spi_socket_connect("TCP", REMOTE_TARGET_IP, REMOTE_TARGET_PORT, REMOTE_LOCAL_PORT)) {
        g_remote_link_state = REMOTE_LINK_SOCKET_FAILED;
        zf_log(0, "Remote TCP connect failed.");
        return;
    }

    g_remote_link_state = REMOTE_LINK_READY;
    g_remote_reconnect_tick = 0u;
    zf_log(0, "Remote link ready.");
}

void page_remote_init(void)
{
    remote_link_try_connect(true);

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
        g_remote_reconnect_tick++;
        if (g_remote_reconnect_tick >= REMOTE_RECONNECT_INTERVAL_TICKS) {
            g_remote_reconnect_tick = 0u;
            remote_link_try_connect(g_remote_link_state != REMOTE_LINK_SOCKET_FAILED);
        }
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
