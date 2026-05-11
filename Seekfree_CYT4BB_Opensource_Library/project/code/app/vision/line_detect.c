#include "line_detect.h"

#include "../../config/vision_defaults.h"

#define IMG_CENTER_COL  (MT9V03X_W / 2u)
#define SCAN_ROWS       (VISION_LINE_ROW_END - VISION_LINE_ROW_START + 1u)

static uint8 row_threshold(const uint8 *row)
{
    uint8 mn = 255u;
    uint8 mx = 0u;

    for (uint16 c = 0; c < MT9V03X_W; c++) {
        if (row[c] < mn) { mn = row[c]; }
        if (row[c] > mx) { mx = row[c]; }
    }

    uint8 mid = (uint8)(((uint16)mn + mx) / 2u);

    if (mid < VISION_LINE_THRESH_LOW)  { mid = VISION_LINE_THRESH_LOW;  }
    if (mid > VISION_LINE_THRESH_HIGH) { mid = VISION_LINE_THRESH_HIGH; }

    return mid;
}

void line_detect(const uint8 image[MT9V03X_H][MT9V03X_W], Line_Result_t *result)
{
    if (result == NULL) {
        return;
    }

    result->center_offset = 0.0f;
    result->confidence    = 0u;

    if (image == NULL) {
        return;
    }

    int32 center_sum = 0;
    uint8 valid_rows = 0u;

    for (uint16 r = VISION_LINE_ROW_START; r <= VISION_LINE_ROW_END; r++) {
        const uint8 *row = image[r];
        uint8        thr = row_threshold(row);

        int16 left  = -1;
        int16 right = -1;

        for (uint16 c = 0u; c < MT9V03X_W; c++) {
            if (row[c] >= thr) {
                if (left < 0) { left = (int16)c; }
                right = (int16)c;
            }
        }

        if (left < 0) { continue; }

        uint16 width = (uint16)(right - left);

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

    if (result->center_offset >  1.0f) { result->center_offset =  1.0f; }
    if (result->center_offset < -1.0f) { result->center_offset = -1.0f; }

    result->confidence = (uint8)((uint32)valid_rows * 10u / SCAN_ROWS);
    if (result->confidence > 10u) { result->confidence = 10u; }
}
