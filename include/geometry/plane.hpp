#pragma  once

#include "geometry/line.hpp"
#include "vector_3d.hpp"

namespace geometry {

// -- plane -- ((r, n) = D) 
class Plane {
public:
    Vector3D r;
    Vector3D normal;
    float D;

    Plane(const Vector3D& point, const Vector3D& n);

    bool is_valid() const;
    float get_distance(const Plane& other) const;
    bool is_match(const Plane& other) const;
    bool is_parallel(const Plane& other) const;
    bool is_contains(const Vector3D& p) const;
    bool is_contains(const Line& line) const;
    bool is_intersected(const Line& line) const;
    Vector3D get_intersect_point(const Line& line) const;
};
} // namespace geometry