#include "obstacle_detect.h"

#include <stddef.h>

#include "../../config/vision_defaults.h"

#define IMG_CENTER_COL  (MT9V03X_W / 2u)   /* 94 */

/* Returns true and sets *out_confidence if the count-based ratio exceeds the
   expected minimum, and writes the column-centroid into *out_col_sum / *out_col_cnt. */
static bool count_pixels_in_band(
    const uint8_t image[MT9V03X_H][MT9V03X_W],
    uint16_t row_start, uint16_t row_end,
    uint16_t col_start, uint16_t col_end,
    uint8_t  lo_thresh,  uint8_t hi_thresh,
    uint16_t min_pixels, uint16_t conf_scale,
    uint32_t *out_col_sum, uint16_t *out_pixel_cnt)
{
    uint32_t col_sum = 0u;
    uint16_t count   = 0u;

    for (uint16_t r = row_start; r <= row_end; r++) {
        for (uint16_t c = col_start; c <= col_end; c++) {
            uint8_t px = image[r][c];
            if (px >= lo_thresh && px <= hi_thresh) {
                col_sum += c;
                count++;
            }
        }
    }

    *out_col_sum    = col_sum;
    *out_pixel_cnt  = count;

    (void)conf_scale;

    return count >= min_pixels;
}

static bool check_obstacle_close(const uint8_t image[MT9V03X_H][MT9V03X_W])
{
    uint32_t dark_count = 0u;
    uint32_t total      = 0u;

    for (uint16_t r = VISION_OBS_ROW_START; r <= VISION_OBS_ROW_END; r++) {
        for (uint16_t c = VISION_OBS_COL_START; c <= VISION_OBS_COL_END; c++) {
            if (image[r][c] < VISION_OBS_DARK_THRESH) {
                dark_count++;
            }
            total++;
        }
    }

    /* obstacle_close when dark fraction >= 6/10 */
    return dark_count * VISION_OBS_DARK_RATIO_DEN >= total * VISION_OBS_DARK_RATIO_NUM;
}

static uint8_t pixel_count_to_confidence(uint16_t count, uint16_t scale)
{
    uint32_t conf = (uint32_t)count * 10u / (uint32_t)scale;
    return conf > 10u ? 10u : (uint8_t)conf;
}

void obstacle_detect(const uint8_t image[MT9V03X_H][MT9V03X_W], Obstacle_Result_t *result)
{
    if (result == NULL) {
        return;
    }

    result->obstacle_close      = false;
    result->landmark            = NAV_LANDMARK_NONE;
    result->landmark_offset     = 0.0f;
    result->landmark_confidence = 0u;

    if (image == NULL) {
        return;
    }

    result->obstacle_close = check_obstacle_close(image);

    uint32_t col_sum  = 0u;
    uint16_t px_count = 0u;
    float    offset   = 0.0f;

    /* --- White circle --- */
    if (count_pixels_in_band(image,
                             VISION_CIRCLE_ROW_START, VISION_CIRCLE_ROW_END,
                             VISION_CIRCLE_COL_START,  VISION_CIRCLE_COL_END,
                             VISION_CIRCLE_BRIGHT_THRESH, 255u,
                             VISION_CIRCLE_MIN_PIXELS, VISION_CIRCLE_CONF_SCALE,
                             &col_sum, &px_count)) {
        float centroid = (float)col_sum / (float)px_count;
        offset = (centroid - (float)IMG_CENTER_COL) / (float)IMG_CENTER_COL;
        if (offset >  1.0f) { offset =  1.0f; }
        if (offset < -1.0f) { offset = -1.0f; }

        result->landmark            = NAV_LANDMARK_WHITE_CIRCLE;
        result->landmark_offset     = offset;
        result->landmark_confidence = pixel_count_to_confidence(px_count, VISION_CIRCLE_CONF_SCALE);
        return;
    }

    /* --- Cone --- */
    if (count_pixels_in_band(image,
                             VISION_CONE_ROW_START, VISION_CONE_ROW_END,
                             VISION_CONE_COL_START,  VISION_CONE_COL_END,
                             VISION_CONE_LO_THRESH,  VISION_CONE_HI_THRESH,
                             VISION_CONE_MIN_PIXELS, VISION_CONE_CONF_SCALE,
                             &col_sum, &px_count)) {
        float centroid = (float)col_sum / (float)px_count;
        offset = (centroid - (float)IMG_CENTER_COL) / (float)IMG_CENTER_COL;
        if (offset >  1.0f) { offset =  1.0f; }
        if (offset < -1.0f) { offset = -1.0f; }

        result->landmark            = NAV_LANDMARK_CONE;
        result->landmark_offset     = offset;
        result->landmark_confidence = pixel_count_to_confidence(px_count, VISION_CONE_CONF_SCALE);
        return;
    }

    /* --- Step / ramp: sharp horizontal brightness drop between two rows --- */
    {
        uint32_t sum_a = 0u;
        uint32_t sum_b = 0u;

        for (uint16_t c = 0u; c < MT9V03X_W; c++) {
            sum_a += image[VISION_STEP_ROW_A][c];
            sum_b += image[VISION_STEP_ROW_B][c];
        }

        uint16_t mean_a = (uint16_t)(sum_a / MT9V03X_W);
        uint16_t mean_b = (uint16_t)(sum_b / MT9V03X_W);
        uint16_t diff   = (mean_a > mean_b) ? (mean_a - mean_b) : (mean_b - mean_a);

        if (diff >= VISION_STEP_EDGE_THRESH) {
            result->landmark            = NAV_LANDMARK_STEP;
            result->landmark_offset     = 0.0f;
            result->landmark_confidence = (uint8_t)((diff - VISION_STEP_EDGE_THRESH) / 5u);
            if (result->landmark_confidence > 10u) { result->landmark_confidence = 10u; }
        }
    }
}
