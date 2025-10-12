#include "geometry/plane.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"

namespace geometry {

Plane::Plane(const Vector3D& point, const Vector3D& n)
: r_(point), normal_(n) {
    
    D_ = n.x() * point.x() + n.y() * point.y() + (n.z() * point.z());
}

Vector3D Plane::normal() const { return normal_; }

float Plane::D() const { return D_; }

bool Plane::is_match(const Plane& other) const {
    if (!normal_.is_collinear(other.normal_))
        return false;

    float sign = (normal_.is_codirected(other.normal_)) ? 1.0f : -1.0f;
    float distance = std::fabs(D_ - sign * other.D_) / normal_.length();
    
    return distance < flt_tolerance;
}

bool Plane::is_parallel(const Plane& other) const {
    if (!normal_.is_collinear(other.normal_))
        return false;

    float sign = (normal_.is_codirected(other.normal_)) ? 1.0f : -1.0f;
    float distance = std::fabs(D_ - sign * other.D_) / normal_.length();
    
    return distance > flt_tolerance;
}

bool Plane::is_valid() const {
    return (!normal_.is_zero());
}
} // namespace geometry