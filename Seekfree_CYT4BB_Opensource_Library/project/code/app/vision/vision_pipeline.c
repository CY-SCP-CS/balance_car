#include "vision_pipeline.h"

#include <stddef.h>

#include "../../../../libraries/zf_device/zf_device_mt9v03x.h"
#include "line_detect.h"
#include "obstacle_detect.h"

static Vision_Result_t g_last_result;

void vision_init(void)
{
    mt9v03x_init();

    g_last_result.line_offset          = 0.0f;
    g_last_result.line_confidence      = 0u;
    g_last_result.landmark             = NAV_LANDMARK_NONE;
    g_last_result.landmark_offset      = 0.0f;
    g_last_result.landmark_confidence  = 0u;
    g_last_result.obstacle_close       = false;
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
    g_last_result.landmark_offset     = obs.landmark_offset;
    g_last_result.landmark_confidence = obs.landmark_confidence;
    g_last_result.obstacle_close      = obs.obstacle_close;

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
    input->landmark_offset     = v->landmark_offset;
    input->landmark_confidence = v->landmark_confidence;
    input->obstacle_close      = v->obstacle_close;
}
