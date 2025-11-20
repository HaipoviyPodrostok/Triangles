#pragma once

#include <cmath>

namespace math {

inline constexpr double eps = 1e-6;
inline constexpr double inner_area_width = 100.0;

template <typename T>
inline bool is_equal(T a, T b,
                     T abs_tol = static_cast<T>(1e-6),
                     T rel_tol = static_cast<T>(1e-6)) {
    if (std::isnan(a) || std::isnan(b)) { return false; }
    if (std::isinf(a) || std::isinf(b)) { return a == b; }

    T diff = std::fabs(a - b);
    if (diff <= abs_tol) {
        return true;
    }
    T scale = std::max(std::fabs(a), std::fabs(b));
    return diff <= rel_tol * scale;
}

inline bool is_zero(double x, double tol = 1e-6) {
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