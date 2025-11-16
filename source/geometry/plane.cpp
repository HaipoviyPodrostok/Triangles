#include "geometry/plane.hpp"
#include "geometry/line.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <cassert>
#include <stdexcept>

namespace geometry {
    
Plane::Plane(const Vector3D& point, const Vector3D& n)
    : r(point) {
        assert(point.is_valid() && n.is_valid() && !n.is_zero());
        normal = n / n.length();
        D = normal.scalar(point);
}

bool Plane::is_valid() const {
    return (r.is_valid()      &&
            normal.is_valid() &&
            !normal.is_zero());
}

float Plane::get_distance(const Plane& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    float sign = (normal.is_codirected(other.normal)) ? 1.0f : -1.0f;
    return std::fabs(D - sign * other.D);
}

bool Plane::is_match(const Plane& other) const {
    assert(this->is_valid());
    assert(other.is_valid());

    if (!normal.is_collinear(other.normal)) {
        return false;
    }

    return math::is_zero(get_distance(other));
}

bool Plane::is_parallel(const Plane& other) const {
    assert(this->is_valid());
    assert(other.is_valid());

    if (!normal.is_collinear(other.normal)) {
        return false;
    }

    return !math::is_zero(get_distance(other));
}

bool Plane::is_contains(const Vector3D& p) const {
    assert(this->is_valid());
    assert(p.is_valid());
    return math::is_zero(p.scalar(normal) - D);
}

bool Plane::is_intersected(const Line& l) const {
    assert(this->is_valid());
    assert(l.is_valid());

    if (!math::is_zero(normal.scalar(l.dir))) { return true; }
    return this->is_contains(l.origin);
}

Vector3D Plane::get_intersect_point(const Line& l) const {
    assert(this->is_valid());
    assert(l.is_valid());
    if (!is_intersected(l)) {
        throw std::logic_error("Plane and line are not intersect");
    }

    float denom = normal.scalar(l.dir);
    float num   = normal.scalar(l.origin) + D;

    assert(!(math::is_zero(denom) && !math::is_zero(num)));
    if (math::is_zero(denom)) {
        return l.origin;
    }

    float t = - num / denom;
    return l.origin + l.dir * t;
}
} // namespace geometry