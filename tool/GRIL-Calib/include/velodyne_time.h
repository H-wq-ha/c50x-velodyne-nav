#pragma once

#include <cmath>

// Relative point time may start at zero. A positive final time distinguishes
// measured offsets from an absent/all-zero time field.
inline bool valid_velodyne_endpoint_times(float first, float last) {
    return std::isfinite(first) && std::isfinite(last) && first >= 0.0f && last > 0.0f;
}
