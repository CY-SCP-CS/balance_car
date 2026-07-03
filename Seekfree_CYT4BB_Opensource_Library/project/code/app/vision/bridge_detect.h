#ifndef APP_VISION_BRIDGE_DETECT_H
#define APP_VISION_BRIDGE_DETECT_H

#include "../../../../libraries/zf_device/zf_device_mt9v03x.h"

#define BRIDGE_MAX_WEDGES 12u

typedef struct {
    uint8   wedge_count;                    /* total black wedges found in ROI */
    int16_t wedge_col[BRIDGE_MAX_WEDGES];   /* column center of each wedge */
    float   lateral_offset;                 /* (target_wedge_col - frame_center) / frame_center */
    bool    aligned;                        /* |lateral_offset| < ALIGN_THRESH */
    uint8   confidence;                     /* 0-10, based on white-pixel context */
} Bridge_Result_t;

/*
 * target_wedge_idx: which wedge the car is currently targeting (0-based).
 * The caller (navigation layer) increments this as each wedge is cleared.
 */
void bridge_detect(const uint8 image[MT9V03X_H][MT9V03X_W],
                   uint8 target_wedge_idx,
                   Bridge_Result_t *result);

#endif
