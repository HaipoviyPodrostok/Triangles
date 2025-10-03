#pragma once

#include <cmath>

namespace math {

template <typename T>
bool is_equal(T a, T b, T eps = static_cast<T>(1e-6)) {
    return std::fabs(a - b) < eps;
}

} // namespace math