#include "line_detect.h"

#include <stddef.h>

#include "../../config/vision_defaults.h"

#define IMG_CENTER_COL  (MT9V03X_W / 2u)   /* 94 */
#define SCAN_ROWS       (VISION_LINE_ROW_END - VISION_LINE_ROW_START + 1u)

static uint8_t row_threshold(const uint8_t *row)
{
    uint8_t mn = 255u;
    uint8_t mx = 0u;

    for (uint16_t c = 0; c < MT9V03X_W; c++) {
        if (row[c] < mn) { mn = row[c]; }
        if (row[c] > mx) { mx = row[c]; }
    }

    uint8_t mid = (uint8_t)(((uint16_t)mn + mx) / 2u);

    if (mid < VISION_LINE_THRESH_LOW)  { mid = VISION_LINE_THRESH_LOW;  }
    if (mid > VISION_LINE_THRESH_HIGH) { mid = VISION_LINE_THRESH_HIGH; }

    return mid;
}

void line_detect(const uint8_t image[MT9V03X_H][MT9V03X_W], Line_Result_t *result)
{
    if (result == NULL) {
        return;
    }

    result->center_offset = 0.0f;
    result->confidence    = 0u;

    if (image == NULL) {
        return;
    }

    int32_t  center_sum   = 0;
    uint8_t  valid_rows   = 0u;

    for (uint16_t r = VISION_LINE_ROW_START; r <= VISION_LINE_ROW_END; r++) {
        const uint8_t *row  = image[r];
        uint8_t        thr  = row_threshold(row);

        /* find leftmost and rightmost white pixel */
        int16_t left  = -1;
        int16_t right = -1;

        for (uint16_t c = 0u; c < MT9V03X_W; c++) {
            if (row[c] >= thr) {
                if (left < 0) { left = (int16_t)c; }
                right = (int16_t)c;
            }
        }

        if (left < 0) { continue; }

        uint16_t width = (uint16_t)(right - left);

        if (width < VISION_LINE_MIN_WIDTH_PX || width > VISION_LINE_MAX_WIDTH_PX) {
            continue;
        }

        center_sum += (left + right) / 2;
        valid_rows++;
    }

    if (valid_rows == 0u) {
        return;
    }

    float avg_center = (float)center_sum / (float)valid_rows;
    result->center_offset = (avg_center - (float)IMG_CENTER_COL) / (float)IMG_CENTER_COL;

    /* clamp to [-1, +1] */
    if (result->center_offset >  1.0f) { result->center_offset =  1.0f; }
    if (result->center_offset < -1.0f) { result->center_offset = -1.0f; }

    /* confidence: fraction of valid rows mapped to [0, 10] */
    result->confidence = (uint8_t)((uint32_t)valid_rows * 10u / SCAN_ROWS);
    if (result->confidence > 10u) { result->confidence = 10u; }
}
