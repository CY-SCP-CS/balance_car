#ifndef APP_VISION_VISION_ANNOTATE_H
#define APP_VISION_VISION_ANNOTATE_H

#include "../../../../libraries/zf_device/zf_device_mt9v03x.h"
#include "vision_pipeline.h"

/*
 * Copy src into dst and draw detection results on top.
 * Markings use inverted-pixel style (dark on bright, bright on dark)
 * so they are visible regardless of background intensity.
 *
 * Annotation legend by mode:
 *   VISION_MODE_MINEFIELD — solid lines where edges found; dashed if near-warning
 *   VISION_MODE_BRIDGE    — cross at each wedge center; filled bar for target wedge
 *   VISION_MODE_STAIR     — horizontal lines at top/bottom stair rows
 *   all modes             — center crosshair reference mark
 */
void vision_annotate(uint8 dst[MT9V03X_H][MT9V03X_W],
                     const uint8 src[MT9V03X_H][MT9V03X_W],
                     const Vision_Result_t *result,
                     Vision_Mode_t mode);

#endif
