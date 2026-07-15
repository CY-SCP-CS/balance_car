#include "minefield_detect.h"

#include "../../config/vision_defaults.h"

/*
 * Detects the 100 cm x 100 cm box as a perspective trapezoid in the grayscale
 * camera image.  The white frame is not actually high-white in this camera
 * setup; field measurements put it in [95, 155].  The blue center patch maps
 * to [45, 75] and is used as a strong confirmation signal.
 *
 * Public output still uses the legacy Minefield_Result_t bounding fields, so
 * other modules do not need to change.
 */

#define MF_BOX_LO_THRESH             95u
#define MF_BOX_HI_THRESH             155u
#define MF_BLUE_LO_THRESH            45u
#define MF_BLUE_HI_THRESH            75u

#define MF_ROI_TOP                   18u
#define MF_ROI_BOT                   (MT9V03X_H * 3u / 4u)
#define MF_MAX_ROW_GAP               2u
#define MF_ROW_MIN_BORDER_PIXELS     6u
#define MF_ROW_MIN_SPAN              18u
#define MF_MIN_VALID_ROWS            12u
#define MF_MIN_FRAME_HEIGHT          18u
#define MF_MIN_FRAME_WIDTH           24u
#define MF_MIN_FRAME_PIXELS          80u
#define MF_MIN_AREA_DEN              5u
#define MF_EDGE_AVG_ROWS             5u
#define MF_EDGE_BAND_MIN_ROWS        3u
#define MF_MIN_WIDTH_RATIO_PERCENT   25u
#define MF_TOP_WIDTH_MAX_PERCENT     110u
#define MF_SIDE_SLOPE_TOL            10

#define MF_BLUE_MIN_PIXELS           35u
#define MF_BLUE_MIN_AREA_PERCENT     2u
#define MF_BLUE_CENTER_TOL_PERCENT   35u

typedef struct {
    bool     valid;
    int16_t  top_row;
    int16_t  bottom_row;
    int16_t  left_top;
    int16_t  right_top;
    int16_t  left_bottom;
    int16_t  right_bottom;
    uint16_t valid_rows;
    uint32_t frame_pixels;
} Minefield_Box_t;

static bool is_box_pixel(uint8 px)
{
    return px >= MF_BOX_LO_THRESH && px <= MF_BOX_HI_THRESH;
}

static bool is_blue_pixel(uint8 px)
{
    return px >= MF_BLUE_LO_THRESH && px <= MF_BLUE_HI_THRESH;
}

static bool is_supported_box_pixel(const uint8 image[MT9V03X_H][MT9V03X_W],
                                   uint16_t row,
                                   uint16_t col)
{
    if (!is_box_pixel(image[row][col])) {
        return false;
    }

    if (row > 0u && is_box_pixel(image[row - 1u][col])) {
        return true;
    }
    if (row < (uint16_t)(MT9V03X_H - 1u) &&
        is_box_pixel(image[row + 1u][col])) {
        return true;
    }
    if (col > 0u && is_box_pixel(image[row][col - 1u])) {
        return true;
    }
    if (col < (uint16_t)(MT9V03X_W - 1u) &&
        is_box_pixel(image[row][col + 1u])) {
        return true;
    }

    return false;
}

static int16_t clamp_col(int16_t col)
{
    if (col < 0) {
        return 0;
    }
    if (col >= (int16_t)MT9V03X_W) {
        return (int16_t)(MT9V03X_W - 1u);
    }
    return col;
}

static int16_t edge_lerp(int16_t top_value,
                         int16_t bottom_value,
                         int16_t top_row,
                         int16_t bottom_row,
                         int16_t row)
{
    int16_t height = (int16_t)(bottom_row - top_row);
    if (height <= 0) {
        return top_value;
    }

    int32_t delta = (int32_t)(bottom_value - top_value) *
                    (int32_t)(row - top_row);
    return (int16_t)(top_value + delta / height);
}

static void result_reset(Minefield_Result_t *result)
{
    result->left_col    = -1;
    result->right_col   = -1;
    result->top_row     = -1;
    result->bottom_row  = -1;
    result->top_left_col = -1;
    result->top_right_col = -1;
    result->bottom_left_col = -1;
    result->bottom_right_col = -1;
    result->left_near   = false;
    result->right_near  = false;
    result->top_near    = false;
    result->bottom_near = false;
    result->detected    = false;
}

static void scan_row_edges(const uint8 image[MT9V03X_H][MT9V03X_W],
                           int16_t left_edge[MT9V03X_H],
                           int16_t right_edge[MT9V03X_H],
                           Minefield_Box_t *box)
{
    uint16_t row_pixels[MT9V03X_H];
    int16_t  current_start = -1;
    int16_t  current_last = -1;
    uint16_t current_rows = 0u;
    uint32_t current_pixels = 0u;
    uint8    current_gap = 0u;
    uint32_t best_pixels = 0u;

    box->valid = false;
    box->top_row = -1;
    box->bottom_row = -1;
    box->valid_rows = 0u;
    box->frame_pixels = 0u;

    for (uint16_t r = 0u; r < MT9V03X_H; r++) {
        left_edge[r] = -1;
        right_edge[r] = -1;
        row_pixels[r] = 0u;
    }

    for (uint16_t r = MF_ROI_TOP; r <= MF_ROI_BOT && r < MT9V03X_H; r++) {
        int16_t left = -1;
        int16_t right = -1;
        uint16_t count = 0u;

        for (uint16_t c = 0u; c < MT9V03X_W; c++) {
            if (!is_supported_box_pixel(image, r, c)) {
                continue;
            }

            if (left < 0) {
                left = (int16_t)c;
            }
            right = (int16_t)c;
            count++;
        }

        if (left < 0 || right <= left) {
            continue;
        }

        uint16_t span = (uint16_t)(right - left + 1);
        if (count < MF_ROW_MIN_BORDER_PIXELS || span < MF_ROW_MIN_SPAN) {
            continue;
        }

        left_edge[r] = left;
        right_edge[r] = right;
        row_pixels[r] = count;
    }

    for (uint16_t r = MF_ROI_TOP; r <= MF_ROI_BOT && r < MT9V03X_H; r++) {
        bool row_valid = left_edge[r] >= 0 && right_edge[r] > left_edge[r];

        if (row_valid) {
            if (current_start < 0) {
                current_start = (int16_t)r;
                current_rows = 0u;
                current_pixels = 0u;
            }
            current_last = (int16_t)r;
            current_rows++;
            current_pixels += row_pixels[r];
            current_gap = 0u;
            continue;
        }

        if (current_start >= 0 && current_gap < MF_MAX_ROW_GAP) {
            current_gap++;
            continue;
        }

        if (current_start >= 0 && current_pixels > best_pixels) {
            best_pixels = current_pixels;
            box->top_row = current_start;
            box->bottom_row = current_last;
            box->valid_rows = current_rows;
            box->frame_pixels = current_pixels;
        }

        current_start = -1;
        current_last = -1;
        current_rows = 0u;
        current_pixels = 0u;
        current_gap = 0u;
    }

    if (current_start >= 0 && current_pixels > best_pixels) {
        box->top_row = current_start;
        box->bottom_row = current_last;
        box->valid_rows = current_rows;
        box->frame_pixels = current_pixels;
    }
}

static bool average_edge_band(const int16_t left_edge[MT9V03X_H],
                              const int16_t right_edge[MT9V03X_H],
                              int16_t start_row,
                              int16_t end_row,
                              int16_t *left_avg,
                              int16_t *right_avg,
                              uint16_t *valid_count)
{
    int32_t left_sum = 0;
    int32_t right_sum = 0;
    uint16_t count = 0u;

    if (start_row < 0) {
        start_row = 0;
    }
    if (end_row >= (int16_t)MT9V03X_H) {
        end_row = (int16_t)(MT9V03X_H - 1u);
    }

    for (int16_t r = start_row; r <= end_row; r++) {
        if (left_edge[r] < 0 || right_edge[r] < 0) {
            continue;
        }
        left_sum += left_edge[r];
        right_sum += right_edge[r];
        count++;
    }

    if (count == 0u) {
        return false;
    }

    *left_avg = (int16_t)(left_sum / (int32_t)count);
    *right_avg = (int16_t)(right_sum / (int32_t)count);
    if (valid_count != NULL) {
        *valid_count = count;
    }
    return true;
}

static bool estimate_box_edges(const int16_t left_edge[MT9V03X_H],
                               const int16_t right_edge[MT9V03X_H],
                               Minefield_Box_t *box)
{
    if (box->valid_rows < MF_MIN_VALID_ROWS ||
        box->frame_pixels < MF_MIN_FRAME_PIXELS ||
        box->top_row < 0 ||
        box->bottom_row <= box->top_row) {
        return false;
    }

    int16_t height = (int16_t)(box->bottom_row - box->top_row);
    if (height < (int16_t)MF_MIN_FRAME_HEIGHT) {
        return false;
    }

    int16_t band = (int16_t)MF_EDGE_AVG_ROWS;
    if (height / 4 < band) {
        band = (int16_t)(height / 4);
    }
    if (band < 1) {
        band = 1;
    }

    uint16_t top_band_rows = 0u;
    uint16_t bottom_band_rows = 0u;

    if (!average_edge_band(left_edge, right_edge,
                           box->top_row,
                           (int16_t)(box->top_row + band),
                           &box->left_top,
                           &box->right_top,
                           &top_band_rows)) {
        return false;
    }

    if (!average_edge_band(left_edge, right_edge,
                           (int16_t)(box->bottom_row - band),
                           box->bottom_row,
                           &box->left_bottom,
                           &box->right_bottom,
                           &bottom_band_rows)) {
        return false;
    }

    if (top_band_rows < MF_EDGE_BAND_MIN_ROWS ||
        bottom_band_rows < MF_EDGE_BAND_MIN_ROWS) {
        return false;
    }

    int16_t top_width = (int16_t)(box->right_top - box->left_top);
    int16_t bottom_width = (int16_t)(box->right_bottom - box->left_bottom);

    if (top_width < (int16_t)MF_MIN_FRAME_WIDTH ||
        bottom_width < (int16_t)MF_MIN_FRAME_WIDTH) {
        return false;
    }

    if ((int32_t)top_width * 100 >
        (int32_t)bottom_width * (int32_t)MF_TOP_WIDTH_MAX_PERCENT) {
        return false;
    }

    if (box->left_bottom > (int16_t)(box->left_top + MF_SIDE_SLOPE_TOL) ||
        box->right_bottom < (int16_t)(box->right_top - MF_SIDE_SLOPE_TOL)) {
        return false;
    }

    uint32_t trapezoid_area = ((uint32_t)top_width + (uint32_t)bottom_width) *
                              (uint32_t)height / 2u;
    if (trapezoid_area * MF_MIN_AREA_DEN < (uint32_t)MT9V03X_IMAGE_SIZE) {
        return false;
    }

    /*
     * Perspective can make either edge shorter depending on mounting and
     * approach angle.  Reject only extreme ratios, then let the blue center
     * validation decide.
     */
    if ((int32_t)top_width * 100 <
            (int32_t)bottom_width * (int32_t)MF_MIN_WIDTH_RATIO_PERCENT ||
        (int32_t)bottom_width * 100 <
            (int32_t)top_width * (int32_t)MF_MIN_WIDTH_RATIO_PERCENT) {
        return false;
    }

    box->valid = true;
    return true;
}

static bool validate_blue_center(const uint8 image[MT9V03X_H][MT9V03X_W],
                                 const Minefield_Box_t *box)
{
    uint32_t blue_count = 0u;
    uint32_t inside_area = 0u;
    uint32_t blue_row_sum = 0u;
    uint32_t blue_col_sum = 0u;

    if (!box->valid) {
        return false;
    }

    for (int16_t r = box->top_row; r <= box->bottom_row; r++) {
        int16_t left = edge_lerp(box->left_top, box->left_bottom,
                                 box->top_row, box->bottom_row, r);
        int16_t right = edge_lerp(box->right_top, box->right_bottom,
                                  box->top_row, box->bottom_row, r);

        left = clamp_col((int16_t)(left + 2));
        right = clamp_col((int16_t)(right - 2));

        if (right <= left) {
            continue;
        }

        inside_area += (uint32_t)(right - left + 1);
        for (int16_t c = left; c <= right; c++) {
            if (is_blue_pixel(image[r][c])) {
                blue_count++;
                blue_row_sum += (uint32_t)r;
                blue_col_sum += (uint32_t)c;
            }
        }
    }

    if (inside_area == 0u || blue_count < MF_BLUE_MIN_PIXELS) {
        return false;
    }

    if (blue_count * 100u < inside_area * MF_BLUE_MIN_AREA_PERCENT) {
        return false;
    }

    int16_t blue_center_row = (int16_t)(blue_row_sum / blue_count);
    int16_t blue_center_col = (int16_t)(blue_col_sum / blue_count);

    int16_t mid_row = (int16_t)((box->top_row + box->bottom_row) / 2);
    int16_t mid_left = edge_lerp(box->left_top, box->left_bottom,
                                 box->top_row, box->bottom_row, mid_row);
    int16_t mid_right = edge_lerp(box->right_top, box->right_bottom,
                                  box->top_row, box->bottom_row, mid_row);
    if (mid_right <= mid_left) {
        return false;
    }

    int16_t mid_col = (int16_t)((mid_left + mid_right) / 2);
    int16_t half_width = (int16_t)((mid_right - mid_left) / 2);
    int16_t half_height = (int16_t)((box->bottom_row - box->top_row) / 2);

    int16_t col_tol = (int16_t)((half_width * MF_BLUE_CENTER_TOL_PERCENT) / 100);
    int16_t row_tol = (int16_t)((half_height * MF_BLUE_CENTER_TOL_PERCENT) / 100);

    if (col_tol < 6) {
        col_tol = 6;
    }
    if (row_tol < 5) {
        row_tol = 5;
    }

    if (blue_center_col < (int16_t)(mid_col - col_tol) ||
        blue_center_col > (int16_t)(mid_col + col_tol)) {
        return false;
    }

    if (blue_center_row < (int16_t)(mid_row - row_tol) ||
        blue_center_row > (int16_t)(mid_row + row_tol)) {
        return false;
    }

    return true;
}

static int16_t min_i16(int16_t a, int16_t b)
{
    return a < b ? a : b;
}

static int16_t max_i16(int16_t a, int16_t b)
{
    return a > b ? a : b;
}

static void publish_box_result(const Minefield_Box_t *box,
                               Minefield_Result_t *result)
{
    int16_t left_col = min_i16(box->left_top, box->left_bottom);
    int16_t right_col = max_i16(box->right_top, box->right_bottom);

    result->left_col = left_col;
    result->right_col = right_col;
    result->top_row = box->top_row;
    result->bottom_row = box->bottom_row;
    result->top_left_col = box->left_top;
    result->top_right_col = box->right_top;
    result->bottom_left_col = box->left_bottom;
    result->bottom_right_col = box->right_bottom;
    result->detected = true;

    result->left_near = left_col < (int16_t)VISION_MF_DANGER_MARGIN;
    result->right_near = right_col >
                         (int16_t)(MT9V03X_W - 1u - VISION_MF_DANGER_MARGIN);
    result->top_near = box->top_row < (int16_t)VISION_MF_DANGER_MARGIN;
    result->bottom_near = box->bottom_row >
                          (int16_t)(MT9V03X_H - 1u - VISION_MF_DANGER_MARGIN);
}

void minefield_detect(const uint8 image[MT9V03X_H][MT9V03X_W],
                      Minefield_Result_t *result)
{
    int16_t left_edge[MT9V03X_H];
    int16_t right_edge[MT9V03X_H];
    Minefield_Box_t box;

    if (result == NULL) {
        return;
    }

    result_reset(result);

    if (image == NULL) {
        return;
    }

    scan_row_edges(image, left_edge, right_edge, &box);
    if (!estimate_box_edges(left_edge, right_edge, &box)) {
        return;
    }

    if (!validate_blue_center(image, &box)) {
        return;
    }

    publish_box_result(&box, result);
}
