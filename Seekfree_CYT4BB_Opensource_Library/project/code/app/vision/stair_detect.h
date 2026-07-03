#ifndef APP_VISION_STAIR_DETECT_H
#define APP_VISION_STAIR_DETECT_H

#include "../../../../libraries/zf_device/zf_device_mt9v03x.h"

typedef struct {
    bool    detected;       /* blue step band found */
    int16_t top_row;        /* upper boundary row of blue step */
    int16_t bottom_row;     /* lower boundary row of blue step */
    float   distance_cm;    /* estimated distance; 0 if not detected */
    uint8   confidence;     /* 0-10 */
} Stair_Result_t;

void stair_detect(const uint8 image[MT9V03X_H][MT9V03X_W], Stair_Result_t *result);

#endif
