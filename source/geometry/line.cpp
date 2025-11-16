#include <cassert>
#include <iostream>
#include <stdexcept>

#include "geometry/line.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"

namespace geometry {

Line::Line(const Vector3D& origin, const Vector3D& dir)
    : origin(origin), dir(dir) {
    
    if (!origin.is_valid()) {
        throw std::invalid_argument("Origin vector is invalid");
    }

    if (!dir.is_valid()) {
        throw std::invalid_argument("Direction vector is invalid");
    }

    if (dir.is_zero()) {
        throw std::invalid_argument("Direction vector can not be (0, 0, 0)");
    }
}

bool Line::is_valid() const {
    return origin.is_valid() && dir.is_valid() && !(dir.is_zero());
}

bool Line::is_match(const Line& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    return dir.is_collinear(other.dir) &&
           (origin - other.origin).is_collinear(dir);
}

bool Line::is_parallel(const Line& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    return dir.is_collinear(other.dir) &&
           !(origin - other.origin).is_collinear(dir);
}

bool Line::is_contains(const Vector3D& point) const {
    assert(this->is_valid());
    assert(point.is_valid());
    return ((point - origin).is_collinear(dir));
}

bool Line::is_intersect(const Line& other) const {
    assert(is_valid());
    assert(other.is_valid());

    if (is_match(other))    { return true;  }
    if (is_parallel(other)) { return false; }

    const Vector3D v = origin - other.origin;
    const Vector3D n = dir.cross(other.dir);

    return math::is_zero(v.scalar(n));
}

// -- line this:  p1 + t * d1 --
// -- line other: p2 + s * d2 --
Vector3D Line::intersect_point(const Line& other) const {
    assert(is_valid());
    assert((other.is_valid()));
    assert(this->is_intersect(other));
    assert(!this->is_match(other));

    const Vector3D& p1 = origin;
    const Vector3D& d1 = dir;
    const Vector3D& p2 = other.origin;
    const Vector3D& d2 = other.dir;

    const Vector3D n = d1.cross(d2);
    assert(!n.is_zero());
    const float n_len_squared = n.length() * n.length();

    const float t = ( (p2 - p1).cross(d2) ).scalar(n) / n_len_squared;
    const float s = ( (p2 - p1).cross(d1) ).scalar(n) / n_len_squared;

    const Vector3D a = p1 + d1 * t;
    const Vector3D b = p2 + d2 * s;
    
    if (math::is_zero( (a - b).length() )) {
        return a;
    }

    return Vector3D::invalid();
}

void Line::print() const {
    std::cout << "origin = "; origin.print();
    std::cout << ", dir = ";  dir.print();
}
}// namespace geometry