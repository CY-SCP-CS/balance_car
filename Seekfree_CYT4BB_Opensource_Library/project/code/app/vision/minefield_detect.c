#include "minefield_detect.h"

#include "../../config/vision_defaults.h"

/*
 * Scans inward from each frame edge to find the first col/row with enough
 * white pixels to be considered a border line.  The car rotates inside the
 * minefield box, so at most two edges are visible at once; we report each
 * independently and flag "near" when an edge is close to the frame boundary.
 */

static int16_t find_left_border(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    for (uint16_t c = 0u; c < MT9V03X_W; c++) {
        uint16_t count = 0u;
        for (uint16_t r = 0u; r < MT9V03X_H; r++) {
            if (image[r][c] >= VISION_MF_WHITE_THRESH) {
                count++;
            }
        }
        if (count >= VISION_MF_MIN_WHITE_PX) {
            return (int16_t)c;
        }
    }
    return -1;
}

static int16_t find_right_border(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    for (uint16_t c = MT9V03X_W; c-- > 0u; ) {
        uint16_t count = 0u;
        for (uint16_t r = 0u; r < MT9V03X_H; r++) {
            if (image[r][c] >= VISION_MF_WHITE_THRESH) {
                count++;
            }
        }
        if (count >= VISION_MF_MIN_WHITE_PX) {
            return (int16_t)c;
        }
    }
    return -1;
}

static int16_t find_top_border(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    for (uint16_t r = 0u; r < MT9V03X_H; r++) {
        uint16_t count = 0u;
        for (uint16_t c = 0u; c < MT9V03X_W; c++) {
            if (image[r][c] >= VISION_MF_WHITE_THRESH) {
                count++;
            }
        }
        if (count >= VISION_MF_MIN_WHITE_PX) {
            return (int16_t)r;
        }
    }
    return -1;
}

static int16_t find_bottom_border(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    for (uint16_t r = MT9V03X_H; r-- > 0u; ) {
        uint16_t count = 0u;
        for (uint16_t c = 0u; c < MT9V03X_W; c++) {
            if (image[r][c] >= VISION_MF_WHITE_THRESH) {
                count++;
            }
        }
        if (count >= VISION_MF_MIN_WHITE_PX) {
            return (int16_t)r;
        }
    }
    return -1;
}

void minefield_detect(const uint8 image[MT9V03X_H][MT9V03X_W], Minefield_Result_t *result)
{
    if (result == NULL) {
        return;
    }

    result->left_col    = -1;
    result->right_col   = -1;
    result->top_row     = -1;
    result->bottom_row  = -1;
    result->left_near   = false;
    result->right_near  = false;
    result->top_near    = false;
    result->bottom_near = false;
    result->detected    = false;

    if (image == NULL) {
        return;
    }

    result->left_col   = find_left_border(image);
    result->right_col  = find_right_border(image);
    result->top_row    = find_top_border(image);
    result->bottom_row = find_bottom_border(image);

    result->detected = (result->left_col  != -1 ||
                        result->right_col != -1 ||
                        result->top_row   != -1 ||
                        result->bottom_row != -1);

    if (result->left_col != -1) {
        result->left_near = result->left_col < (int16_t)VISION_MF_DANGER_MARGIN;
    }
    if (result->right_col != -1) {
        result->right_near = result->right_col > (int16_t)(MT9V03X_W - 1u - VISION_MF_DANGER_MARGIN);
    }
    if (result->top_row != -1) {
        result->top_near = result->top_row < (int16_t)VISION_MF_DANGER_MARGIN;
    }
    if (result->bottom_row != -1) {
        result->bottom_near = result->bottom_row > (int16_t)(MT9V03X_H - 1u - VISION_MF_DANGER_MARGIN);
    }
}
