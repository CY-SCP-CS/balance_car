#include "vision_pipeline.h"

#include "../../../../libraries/zf_device/zf_device_mt9v03x.h"
#include "line_detect.h"
#include "obstacle_detect.h"
#include "minefield_detect.h"
#include "stair_detect.h"
#include "bridge_detect.h"

static Vision_Result_t g_last_result;
static Vision_Mode_t   g_mode = VISION_MODE_LINE;

/* target wedge index managed externally and passed on each update */
static uint8 g_bridge_target_wedge = 0u;

void vision_init(void)
{
    mt9v03x_init();

    g_last_result.line_offset          = 0.0f;
    g_last_result.line_confidence      = 0u;
    g_last_result.landmark             = NAV_LANDMARK_NONE;
    g_last_result.obstacle_close       = false;

    g_last_result.minefield.detected   = false;
    g_last_result.stair.detected       = false;
    g_last_result.bridge.wedge_count   = 0u;

    g_mode = VISION_MODE_LINE;
    g_bridge_target_wedge = 0u;
}

void vision_set_mode(Vision_Mode_t mode)
{
    g_mode = mode;
    if (mode == VISION_MODE_BRIDGE) {
        g_bridge_target_wedge = 0u;
    }
}

Vision_Mode_t vision_get_mode(void)
{
    return g_mode;
}

bool vision_update(Vision_Result_t *result)
{
    if (!mt9v03x_finish_flag) {
        if (result != NULL) {
            *result = g_last_result;
        }
        return false;
    }

    mt9v03x_finish_flag = 0u;

    Line_Result_t     line;
    Obstacle_Result_t obs;

    line_detect(mt9v03x_image, &line);
    obstacle_detect(mt9v03x_image, &obs);

    g_last_result.line_offset         = line.center_offset;
    g_last_result.line_confidence     = line.confidence;
    g_last_result.landmark            = obs.landmark;
    g_last_result.obstacle_close      = obs.obstacle_close;

    switch (g_mode) {
    case VISION_MODE_MINEFIELD:
        minefield_detect(mt9v03x_image, &g_last_result.minefield);
        break;
    case VISION_MODE_BRIDGE:
        bridge_detect(mt9v03x_image, g_bridge_target_wedge, &g_last_result.bridge);
        break;
    case VISION_MODE_STAIR:
        stair_detect(mt9v03x_image, &g_last_result.stair);
        break;
    default:
        break;
    }

    if (result != NULL) {
        *result = g_last_result;
    }

    return true;
}

void vision_feed_nav_input(Nav_Input_t *input, const Vision_Result_t *v)
{
    if (input == NULL || v == NULL) {
        return;
    }

    input->landmark            = v->landmark;
    input->obstacle_close      = v->obstacle_close;
}
