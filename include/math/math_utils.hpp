#pragma once

#include <cmath>

namespace math {

inline constexpr float flt_tolerance    = 1e-6f;
inline constexpr float eps              = 1e-6f;
inline constexpr float inner_area_width = 100.0;

template <typename T>
inline bool is_equal(T a, T b, T eps = static_cast<T>(flt_tolerance)) {
    if (std::isnan(a) || std::isnan(b)) return false;
    if (std::isinf(a) || std::isinf(b)) return a == b;

    T diff = std::fabs(a - b);
    T norm = std::max(std::fabs(a), std::fabs(b));
    return diff <= eps * (1 + norm);
}

inline bool is_zero(float x, float tol = flt_tolerance) {
    return is_equal(x, 0.0f, tol);
}
} // namespace math