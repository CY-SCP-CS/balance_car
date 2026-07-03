#include "vision_annotate.h"

#include <string.h>

/* --- Drawing primitives ----------------------------------------- */

/* Invert a single pixel: bright→dark, dark→bright. Always visible. */
static inline void invert_px(uint8 img[MT9V03X_H][MT9V03X_W],
                              int16_t row, int16_t col)
{
    if (row < 0 || row >= (int16_t)MT9V03X_H ||
        col < 0 || col >= (int16_t)MT9V03X_W) {
        return;
    }
    img[row][col] = (img[row][col] >= 128u) ? 0u : 255u;
}

static void draw_hline(uint8 img[MT9V03X_H][MT9V03X_W],
                       int16_t row, int16_t c0, int16_t c1)
{
    if (c0 > c1) { int16_t tmp = c0; c0 = c1; c1 = tmp; }
    for (int16_t c = c0; c <= c1; c++) {
        invert_px(img, row, c);
    }
}

static void draw_vline(uint8 img[MT9V03X_H][MT9V03X_W],
                       int16_t col, int16_t r0, int16_t r1)
{
    if (r0 > r1) { int16_t tmp = r0; r0 = r1; r1 = tmp; }
    for (int16_t r = r0; r <= r1; r++) {
        invert_px(img, r, col);
    }
}

/* Dashed horizontal line — skips every other 4-px segment */
static void draw_hline_dashed(uint8 img[MT9V03X_H][MT9V03X_W],
                               int16_t row, int16_t c0, int16_t c1)
{
    if (c0 > c1) { int16_t tmp = c0; c0 = c1; c1 = tmp; }
    for (int16_t c = c0; c <= c1; c++) {
        if (((c - c0) & 4u) == 0u) {   /* on for 4, off for 4 */
            invert_px(img, row, c);
        }
    }
}

/* Dashed vertical line */
static void draw_vline_dashed(uint8 img[MT9V03X_H][MT9V03X_W],
                               int16_t col, int16_t r0, int16_t r1)
{
    if (r0 > r1) { int16_t tmp = r0; r0 = r1; r1 = tmp; }
    for (int16_t r = r0; r <= r1; r++) {
        if (((r - r0) & 4u) == 0u) {
            invert_px(img, r, col);
        }
    }
}

/* Small cross marker (arm length = arm_len pixels each side) */
static void draw_cross(uint8 img[MT9V03X_H][MT9V03X_W],
                        int16_t row, int16_t col, int16_t arm_len)
{
    draw_hline(img, row, (int16_t)(col - arm_len), (int16_t)(col + arm_len));
    draw_vline(img, col, (int16_t)(row - arm_len), (int16_t)(row + arm_len));
}

/* Rectangle outline */
static void draw_rect(uint8 img[MT9V03X_H][MT9V03X_W],
                       int16_t r0, int16_t c0, int16_t r1, int16_t c1)
{
    draw_hline(img, r0, c0, c1);
    draw_hline(img, r1, c0, c1);
    draw_vline(img, c0, r0, r1);
    draw_vline(img, c1, r0, r1);
}

/* --- Mode-specific annotation ----------------------------------- */

static void annotate_center_ref(uint8 img[MT9V03X_H][MT9V03X_W])
{
    /* Small cross at frame center — always drawn as orientation reference */
    int16_t cr = (int16_t)(MT9V03X_H / 2u);
    int16_t cc = (int16_t)(MT9V03X_W / 2u);
    draw_cross(img, cr, cc, 6);
}

static void annotate_minefield(uint8 img[MT9V03X_H][MT9V03X_W],
                                const Minefield_Result_t *mf)
{
    if (!mf->detected) {
        return;
    }

    /* Left border */
    if (mf->left_col >= 0) {
        if (mf->left_near) {
            draw_vline_dashed(img, mf->left_col, 0, (int16_t)(MT9V03X_H - 1u));
        } else {
            draw_vline(img, mf->left_col, 0, (int16_t)(MT9V03X_H - 1u));
        }
    }

    /* Right border */
    if (mf->right_col >= 0) {
        if (mf->right_near) {
            draw_vline_dashed(img, mf->right_col, 0, (int16_t)(MT9V03X_H - 1u));
        } else {
            draw_vline(img, mf->right_col, 0, (int16_t)(MT9V03X_H - 1u));
        }
    }

    /* Top border */
    if (mf->top_row >= 0) {
        if (mf->top_near) {
            draw_hline_dashed(img, mf->top_row, 0, (int16_t)(MT9V03X_W - 1u));
        } else {
            draw_hline(img, mf->top_row, 0, (int16_t)(MT9V03X_W - 1u));
        }
    }

    /* Bottom border */
    if (mf->bottom_row >= 0) {
        if (mf->bottom_near) {
            draw_hline_dashed(img, mf->bottom_row, 0, (int16_t)(MT9V03X_W - 1u));
        } else {
            draw_hline(img, mf->bottom_row, 0, (int16_t)(MT9V03X_W - 1u));
        }
    }

    /* Corner tick where two found borders intersect (debugging aid) */
    if (mf->left_col >= 0 && mf->top_row >= 0) {
        draw_cross(img, mf->top_row, mf->left_col, 4);
    }
    if (mf->right_col >= 0 && mf->top_row >= 0) {
        draw_cross(img, mf->top_row, mf->right_col, 4);
    }
    if (mf->left_col >= 0 && mf->bottom_row >= 0) {
        draw_cross(img, mf->bottom_row, mf->left_col, 4);
    }
    if (mf->right_col >= 0 && mf->bottom_row >= 0) {
        draw_cross(img, mf->bottom_row, mf->right_col, 4);
    }
}

static void annotate_bridge(uint8 img[MT9V03X_H][MT9V03X_W],
                              const Bridge_Result_t *br)
{
    if (br->wedge_count == 0u) {
        return;
    }

    /* ROI band boundary lines */
    draw_hline(img, (int16_t)(MT9V03X_H * 60u / 120u),
               0, (int16_t)(MT9V03X_W - 1u));
    draw_hline(img, (int16_t)(MT9V03X_H * 100u / 120u),
               0, (int16_t)(MT9V03X_W - 1u));

    /* Mid-ROI row for wedge markers */
    int16_t mark_row = (int16_t)((60u + 100u) / 2u);

    for (uint8 i = 0u; i < br->wedge_count; i++) {
        int16_t col = br->wedge_col[i];

        /* Vertical bar across full ROI for every wedge */
        draw_vline(img, col,
                   (int16_t)(MT9V03X_H * 60u / 120u),
                   (int16_t)(MT9V03X_H * 100u / 120u));

        /* Small cross at mid-row; larger cross for the target wedge */
        if (br->lateral_offset >= -1.0f) {
            /* find target wedge by checking whose column gives the stored offset */
            float expected_off = ((float)col - (float)(MT9V03X_W / 2u))
                                 / (float)(MT9V03X_W / 2u);
            bool is_target = (expected_off > br->lateral_offset - 0.05f &&
                              expected_off < br->lateral_offset + 0.05f);
            draw_cross(img, mark_row, col, is_target ? 8 : 4);
        } else {
            draw_cross(img, mark_row, col, 4);
        }
    }

    /* Alignment indicator: vertical line at frame center bottom  */
    if (br->aligned) {
        draw_vline(img, (int16_t)(MT9V03X_W / 2u),
                   (int16_t)(MT9V03X_H - 10u),
                   (int16_t)(MT9V03X_H - 1u));
    }
}

static void annotate_stair(uint8 img[MT9V03X_H][MT9V03X_W],
                             const Stair_Result_t *st)
{
    if (!st->detected) {
        return;
    }

    /* Horizontal lines at detected step edges */
    draw_hline(img, st->top_row,    0, (int16_t)(MT9V03X_W - 1u));
    draw_hline(img, st->bottom_row, 0, (int16_t)(MT9V03X_W - 1u));

    /* Distance indicator: short vertical tick marks on left edge,
     * number of ticks = clamp(distance_cm / 10, 1, 8) */
    uint8 ticks = (uint8)(st->distance_cm / 10.0f);
    if (ticks < 1u) { ticks = 1u; }
    if (ticks > 8u) { ticks = 8u; }

    for (uint8 t = 0u; t < ticks; t++) {
        draw_hline(img,
                   (int16_t)(st->top_row - 2 - (int16_t)(t * 4u)),
                   0, 8);
    }
}

/* --- Public API -------------------------------------------------- */

void vision_annotate(uint8 dst[MT9V03X_H][MT9V03X_W],
                     const uint8 src[MT9V03X_H][MT9V03X_W],
                     const Vision_Result_t *result,
                     Vision_Mode_t mode)
{
    if (dst == NULL || src == NULL || result == NULL) {
        return;
    }

    /* Copy raw frame */
    (void)memcpy(dst, src, (uint32_t)MT9V03X_H * (uint32_t)MT9V03X_W);

    annotate_center_ref(dst);

    switch (mode) {
    case VISION_MODE_MINEFIELD:
        annotate_minefield(dst, &result->minefield);
        break;
    case VISION_MODE_BRIDGE:
        annotate_bridge(dst, &result->bridge);
        break;
    case VISION_MODE_STAIR:
        annotate_stair(dst, &result->stair);
        break;
    default:
        break;
    }
}
