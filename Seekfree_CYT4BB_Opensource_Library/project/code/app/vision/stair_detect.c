#include "stair_detect.h"

#include "../../config/vision_defaults.h"

/*
 * Blue step detection strategy (grayscale):
 *   The blue middle step maps to an intermediate brightness band
 *   [STAIR_LO_THRESH, STAIR_HI_THRESH] on the MT9V034.  The exact values
 *   depend on lighting; tune STAIR_LO/HI in vision_defaults.h on site.
 *
 * Distance formula:
 *   distance_cm = STAIR_HEIGHT_CM * STAIR_FOCAL_CONST / pixel_span
 *   Calibrate STAIR_FOCAL_CONST by placing a blue board of known height at
 *   known distances (30, 50, 80 cm) and recording pixel_span each time.
 */

static uint16_t count_stair_pixels_in_row(
    const uint8 image[MT9V03X_H][MT9V03X_W], uint16_t row)
{
    uint16_t count = 0u;
    for (uint16_t c = 0u; c < MT9V03X_W; c++) {
        uint8 px = image[row][c];
        if (px >= VISION_STAIR_LO_THRESH && px <= VISION_STAIR_HI_THRESH) {
            count++;
        }
    }
    return count;
}

void stair_detect(const uint8 image[MT9V03X_H][MT9V03X_W], Stair_Result_t *result)
{
    if (result == NULL) {
        return;
    }

    result->detected     = false;
    result->top_row      = -1;
    result->bottom_row   = -1;
    result->distance_cm  = 0.0f;
    result->confidence   = 0u;

    if (image == NULL) {
        return;
    }

    int16_t first_row = -1;
    int16_t last_row  = -1;
    uint16_t stair_row_count = 0u;

    for (uint16_t r = VISION_STAIR_ROI_TOP; r <= VISION_STAIR_ROI_BOT; r++) {
        if (count_stair_pixels_in_row(image, r) >= VISION_STAIR_MIN_COLS) {
            if (first_row == -1) {
                first_row = (int16_t)r;
            }
            last_row = (int16_t)r;
            stair_row_count++;
        }
    }

    if (first_row == -1 || last_row <= first_row) {
        return;
    }

    uint16_t pixel_span = (uint16_t)(last_row - first_row);

    result->detected    = true;
    result->top_row     = first_row;
    result->bottom_row  = last_row;

    /* guard against division by zero; pixel_span already > 0 here */
    result->distance_cm = VISION_STAIR_HEIGHT_CM * VISION_STAIR_FOCAL_CONST
                          / (float)pixel_span;

    uint32_t roi_height = VISION_STAIR_ROI_BOT - VISION_STAIR_ROI_TOP + 1u;
    uint32_t conf = (uint32_t)stair_row_count * 10u / roi_height;
    result->confidence = (uint8)(conf > 10u ? 10u : conf);
}
