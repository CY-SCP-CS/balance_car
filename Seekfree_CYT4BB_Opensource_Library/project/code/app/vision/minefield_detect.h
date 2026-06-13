#ifndef APP_VISION_MINEFIELD_DETECT_H
#define APP_VISION_MINEFIELD_DETECT_H

#include "../../../../libraries/zf_device/zf_device_mt9v03x.h"

typedef struct {
    bool    detected;       /* at least one border edge visible */
    int16_t left_col;       /* column of left border; -1 if not found */
    int16_t right_col;      /* column of right border; -1 if not found */
    int16_t top_row;        /* row of top border; -1 if not found */
    int16_t bottom_row;     /* row of bottom border; -1 if not found */
    bool    left_near;      /* left edge within DANGER_MARGIN of frame */
    bool    right_near;
    bool    top_near;
    bool    bottom_near;
} Minefield_Result_t;

void minefield_detect(const uint8 image[MT9V03X_H][MT9V03X_W], Minefield_Result_t *result);

#endif
