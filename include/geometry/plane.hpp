#pragma  once

#include "vector_3d.hpp"

namespace geometry {

// -- plane -- ((r, n) = D) 
class Plane {
public:
    Vector3D r_;
    Vector3D normal_;
    float D_;

    Plane(const Vector3D& point, const Vector3D& n);

    bool is_valid() const;
    Vector3D normal() const;
    float get_distance(const Plane& other) const;
    bool is_match(const Plane& other) const;
    bool is_parallel(const Plane& other) const;
    bool is_contains(const Vector3D& p) const;
};
} // namespace geometry