#pragma once

#include <cmath>

namespace math {

inline constexpr double dbl_tolerance    = 1e-2;
inline constexpr double eps              = 1e-2;
inline constexpr double inner_area_width = 100.0;

template <typename T>
inline bool is_equal(T a, T b, T eps = static_cast<T>(dbl_tolerance)) {
    if (std::isnan(a) || std::isnan(b)) return false;
    if (std::isinf(a) || std::isinf(b)) return a == b;

    T diff = std::fabs(a - b);
    T norm = std::max(std::fabs(a), std::fabs(b));
    return diff <= eps * (1 + norm);
}

inline bool is_zero(double x, double tol = dbl_tolerance) {
    return is_equal(x, 0.0, tol);
}

template <typename T>
constexpr T sqr(T x) { return x * x;}

enum class Axis {
    X = 0,
    Y = 1,
    Z = 2,
};
} // namespace math