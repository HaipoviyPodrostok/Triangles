#include "geometry/plane.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <cassert>

namespace geometry {
    
Plane::Plane(const Vector3D& point, const Vector3D& n)
: r_(point), normal_(n), D_(n.scalar(point)) { }

bool Plane::is_valid() const {
    return (r_.is_valid()      &&
            normal_.is_valid() &&
            !normal_.is_zero());
}

Vector3D Plane::normal() const { return normal_; }

float Plane::D() const { return D_; }

bool Plane::is_match(const Plane& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    
    if (!normal_.is_collinear(other.normal_)) {
        return false;
    }

    float sign = (normal_.is_codirected(other.normal_)) ? 1.0f : -1.0f;
    float distance = std::fabs(D_ - sign * other.D_) / normal_.length();
    
    return distance <= math::eps;
}

bool Plane::is_parallel(const Plane& other) const {
    assert(this->is_valid());
    assert(other.is_valid());

    if (!normal_.is_collinear(other.normal_)) {
        return false;
    }

    float sign = (normal_.is_codirected(other.normal_)) ? 1.0f : -1.0f;
    float distance = std::fabs(D_ - sign * other.D_) / normal_.length();
    
    return distance >= math::eps;
}

bool Plane::is_contains(const Vector3D& p) const {
    assert(this->is_valid());
    assert(p.is_valid());
    return math::is_zero(p.scalar(normal_) - D_);
}
} // namespace geometry