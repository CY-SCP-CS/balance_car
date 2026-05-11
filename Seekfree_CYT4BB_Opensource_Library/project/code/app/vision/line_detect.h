#ifndef APP_VISION_LINE_DETECT_H
#define APP_VISION_LINE_DETECT_H

#include "../../../../libraries/zf_device/zf_device_mt9v03x.h"

typedef struct {
    float center_offset;  /* normalised: -1.0 = far left, +1.0 = far right */
    uint8 confidence;     /* 0-10 */
} Line_Result_t;

void line_detect(const uint8 image[MT9V03X_H][MT9V03X_W], Line_Result_t *result);

#endif
