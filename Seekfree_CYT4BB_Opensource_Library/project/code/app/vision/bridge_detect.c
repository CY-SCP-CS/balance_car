#include "bridge_detect.h"

#include "../../config/vision_defaults.h"

#define IMG_CENTER_COL  (MT9V03X_W / 2u)

/*
 * Find the row in [ROI_TOP, ROI_BOT] with the most black pixels.
 * This is the "canonical" scan row where wedges are most visible.
 */
static uint16_t find_best_row(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint16_t best_row   = VISION_BRIDGE_ROI_TOP;
    uint16_t best_count = 0u;

    for (uint16_t r = VISION_BRIDGE_ROI_TOP; r <= VISION_BRIDGE_ROI_BOT; r++) {
        uint16_t count = 0u;
        for (uint16_t c = 0u; c < MT9V03X_W; c++) {
            if (image[r][c] < VISION_BRIDGE_DARK_THRESH) {
                count++;
            }
        }
        if (count > best_count) {
            best_count = count;
            best_row   = r;
        }
    }

    return best_row;
}

/*
 * Scan best_row for dark runs flanked by lighter pixels.
 * Each qualifying run is one wedge.
 */
static uint8 scan_wedges(const uint8 image[MT9V03X_H][MT9V03X_W],
                         uint16_t scan_row,
                         int16_t out_cols[],
                         uint8 max_wedges)
{
    uint8    count      = 0u;
    bool     in_run     = false;
    uint16_t run_start  = 0u;

    for (uint16_t c = 0u; c < MT9V03X_W && count < max_wedges; c++) {
        bool dark = image[scan_row][c] < VISION_BRIDGE_DARK_THRESH;

        if (dark && !in_run) {
            in_run    = true;
            run_start = c;
        } else if (!dark && in_run) {
            in_run = false;
            uint16_t width = c - run_start;
            if (width >= VISION_BRIDGE_WEDGE_MIN_W && width <= VISION_BRIDGE_WEDGE_MAX_W) {
                out_cols[count] = (int16_t)((run_start + c - 1u) / 2u);
                count++;
            }
        }
    }

    /* close an open run at the right edge */
    if (in_run && count < max_wedges) {
        uint16_t width = MT9V03X_W - run_start;
        if (width >= VISION_BRIDGE_WEDGE_MIN_W && width <= VISION_BRIDGE_WEDGE_MAX_W) {
            out_cols[count] = (int16_t)((run_start + MT9V03X_W - 1u) / 2u);
            count++;
        }
    }

    return count;
}

/*
 * Confidence: fraction of the ROI row that is white (bright PVC),
 * indicating the camera is actually looking at the bridge surface.
 */
static uint8 compute_confidence(const uint8 image[MT9V03X_H][MT9V03X_W],
                                uint16_t scan_row)
{
    uint16_t white_count = 0u;
    for (uint16_t c = 0u; c < MT9V03X_W; c++) {
        if (image[scan_row][c] >= 140u) {
            white_count++;
        }
    }
    uint32_t conf = (uint32_t)white_count * 10u / MT9V03X_W;
    return (uint8)(conf > 10u ? 10u : conf);
}

void bridge_detect(const uint8 image[MT9V03X_H][MT9V03X_W],
                   uint8 target_wedge_idx,
                   Bridge_Result_t *result)
{
    if (result == NULL) {
        return;
    }

    result->wedge_count    = 0u;
    result->lateral_offset = 0.0f;
    result->aligned        = false;
    result->confidence     = 0u;

    if (image == NULL) {
        return;
    }

    uint16_t scan_row = find_best_row(image);

    result->wedge_count = scan_wedges(image, scan_row,
                                      result->wedge_col,
                                      BRIDGE_MAX_WEDGES);
    result->confidence  = compute_confidence(image, scan_row);

    if (result->wedge_count == 0u) {
        return;
    }

    uint8 target = (target_wedge_idx < result->wedge_count)
                   ? target_wedge_idx
                   : (uint8)(result->wedge_count - 1u);

    float offset = ((float)result->wedge_col[target] - (float)IMG_CENTER_COL)
                   / (float)IMG_CENTER_COL;
    if (offset >  1.0f) { offset =  1.0f; }
    if (offset < -1.0f) { offset = -1.0f; }

    result->lateral_offset = offset;
    result->aligned = (offset > -VISION_BRIDGE_ALIGN_THRESH &&
                       offset <  VISION_BRIDGE_ALIGN_THRESH);
}
