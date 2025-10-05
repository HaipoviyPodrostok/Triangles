#pragma  once

#include "geometry/vector_3d.hpp"

namespace geometry {

// -- plane -- ((r, n) + D = 0) 
class Plane {
public:
    Plane(const Vector3D& point, const Vector3D& n);

    bool match(const Plane& other) const;
    bool parallel(const Plane& other) const;
    bool is_valid() const;

private:
    Vector3D r_;
    Vector3D normal_;
    float D_;
};
} // namespace geometry