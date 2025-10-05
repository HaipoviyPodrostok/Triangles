#include "geometry/plane.hpp"
#include "math/math_utils.hpp"

namespace geometry {

Plane::Plane(const Vector3D& point, const Vector3D& n)
: r_(point), normal_(n) {
    
    D_ = -((n.x() * point.x()) + (n.y() * point.y()) + (n.z() * point.z()));
}

bool Plane::match(const Plane& other) const {
    float scalar = r_.scalar(normal_);
    
    return (normal_.is_collinear(other.normal_) &&
            math::is_equal(scalar, 0.0f)); 
}

bool Plane::parallel(const Plane& other) const {
    float scalar = r_.scalar(normal_);
    
    return (normal_.is_collinear(other.normal_) &&
            !math::is_equal(scalar, 0.0f)); 
}

bool Plane::is_valid() const {
    return (normal_.is_zero());
}
} // namespace geometry