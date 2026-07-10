#include "obstacle_detect.h"

#include "../../config/vision_defaults.h"

static bool count_pixels_in_band(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    uint16 row_start, uint16 row_end,
    uint16 col_start, uint16 col_end,
    uint8  lo_thresh,  uint8 hi_thresh,
    uint16 min_pixels)
{
    uint16 count = 0u;

    for (uint16 r = row_start; r <= row_end; r++) {
        for (uint16 c = col_start; c <= col_end; c++) {
            uint8 px = image[r][c];
            if (px >= lo_thresh && px <= hi_thresh) {
                count++;
            }
        }
    }

    return count >= min_pixels;
}

static bool check_obstacle_close(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint32 dark_count = 0u;
    uint32 total      = 0u;

    for (uint16 r = VISION_OBS_ROW_START; r <= VISION_OBS_ROW_END; r++) {
        for (uint16 c = VISION_OBS_COL_START; c <= VISION_OBS_COL_END; c++) {
            if (image[r][c] < VISION_OBS_DARK_THRESH) {
                dark_count++;
            }
            total++;
        }
    }

    return dark_count * VISION_OBS_DARK_RATIO_DEN >= total * VISION_OBS_DARK_RATIO_NUM;
}

void obstacle_detect(const uint8 image[MT9V03X_H][MT9V03X_W], Obstacle_Result_t *result)
{
    if (result == NULL) {
        return;
    }

    result->obstacle_close      = false;
    result->landmark            = NAV_LANDMARK_NONE;

    if (image == NULL) {
        return;
    }

    result->obstacle_close = check_obstacle_close(image);

    /* --- White circle --- */
    if (count_pixels_in_band(image,
                             VISION_CIRCLE_ROW_START, VISION_CIRCLE_ROW_END,
                             VISION_CIRCLE_COL_START,  VISION_CIRCLE_COL_END,
                             VISION_CIRCLE_BRIGHT_THRESH, 255u,
                             VISION_CIRCLE_MIN_PIXELS)) {
        result->landmark            = NAV_LANDMARK_WHITE_CIRCLE;
        return;
    }

    /* --- Cone --- */
    if (count_pixels_in_band(image,
                             VISION_CONE_ROW_START, VISION_CONE_ROW_END,
                             VISION_CONE_COL_START,  VISION_CONE_COL_END,
                             VISION_CONE_LO_THRESH,  VISION_CONE_HI_THRESH,
                             VISION_CONE_MIN_PIXELS)) {
        result->landmark            = NAV_LANDMARK_CONE;
        return;
    }

    /* --- Step: sharp horizontal brightness difference between two rows --- */
    {
        uint32 sum_a = 0u;
        uint32 sum_b = 0u;

        for (uint16 c = 0u; c < MT9V03X_W; c++) {
            sum_a += image[VISION_STEP_ROW_A][c];
            sum_b += image[VISION_STEP_ROW_B][c];
        }

        uint16 mean_a = (uint16)(sum_a / MT9V03X_W);
        uint16 mean_b = (uint16)(sum_b / MT9V03X_W);
        uint16 diff   = (mean_a > mean_b) ? (mean_a - mean_b) : (mean_b - mean_a);

        if (diff >= VISION_STEP_EDGE_THRESH) {
            result->landmark            = NAV_LANDMARK_STEP;
        }
    }
}
