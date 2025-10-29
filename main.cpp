#include "geometry/geometry.hpp"
#include "math/math_utils.hpp"

using namespace geometry;
constexpr float eps = math::flt_tolerance;

int main() {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 1, 0));
    Section s2(Vector3D(0, 1, 0), Vector3D(1, 0, 0));

    s1.is_intersect(s2);
    return 0;
}
