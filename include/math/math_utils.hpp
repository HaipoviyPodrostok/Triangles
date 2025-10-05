#pragma once

#include <cmath>

inline constexpr float flt_tolerance    = 1e-6f;
inline constexpr float inner_area_width = 100.0;

namespace math {

inline constexpr bool is_zero(float x, float tol = flt_tolerance) {
    return std::fabs(x) < tol;
}

template <typename T>
bool is_equal(T a, T b, T eps = static_cast<T>(1e-6)) {
    return std::fabs(a - b) < eps;
}
} // namespace math