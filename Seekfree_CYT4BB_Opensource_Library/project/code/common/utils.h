#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

static inline float clamp(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

#endif
