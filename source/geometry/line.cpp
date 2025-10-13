#include <cassert>
#include <cmath>
#include <iostream>

#include "geometry/line.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"

namespace geometry {

Line::Line(const Vector3D& origin, const Vector3D& dir)
    : origin_(origin), dir_(dir) {
    
    if (dir.is_zero()) {
        throw std::invalid_argument("Direction vector can not be (0, 0, 0)");
    }
}

Vector3D Line::origin() const { return origin_; }
Vector3D Line::dir()    const { return dir_; }

bool Line::is_valid() const {
    return origin_.is_valid() && dir_.is_valid() && !(dir_.is_zero());
}

bool Line::is_match(const Line& other) const {
    return dir_.is_collinear(other.dir_) &&
           (origin_ - other.origin_).is_collinear(dir_);
}

bool Line::is_parallel(const Line& other) const {
    return dir_.is_collinear(other.dir_) &&
           !(origin_ - other.origin_).is_collinear(dir_);
}

bool Line::is_contains(const Vector3D& point) const {
    return ((point - origin_).is_collinear(dir_));
}

bool Line::is_intersect(const Line& other) const {
    assert(is_valid());
    assert(other.is_valid());

    if (is_match(other))    { return true;  }
    if (is_parallel(other)) { return false; }

    const Vector3D& p1 = origin_;
    const Vector3D& d1 = dir_;
    const Vector3D& p2 = other.origin_;
    const Vector3D& d2 = other.dir_;

    const Vector3D v = p2 - p1;
    const Vector3D n = d1.cross(d2);

    if (n.is_zero()) { return false; }
    if (math::is_zero(v.scalar(n))) { return true; }

    return false;
}

// -- line this:  p1 + t * d1 --
// -- line other: p2 + s * d2 --
Vector3D Line::intersect_point(const Line& other) const {
    assert(is_valid());
    assert((other.is_valid()));
    if (!this->is_intersect(other) || this->is_match(other)) {
        return {NAN, NAN, NAN};
    }

    const Vector3D& p1 = origin_;
    const Vector3D& d1 = dir_;
    const Vector3D& p2 = other.origin_;
    const Vector3D& d2 = other.dir_;

    const Vector3D n = d1.cross(d2);
    if (n.is_zero()) { return {NAN, NAN, NAN}; }
    const float n_len_squared = n.length() * n.length();

    const float t = ( (p2 - p1).cross(d2) ).scalar(n) / n_len_squared;
    const float s = ( (p2 - p1).cross(d1) ).scalar(n) / n_len_squared;

    const Vector3D a = p1 + d1 * t;
    const Vector3D b = p2 + d2 * s;
    
    if (math::is_zero( (a - b).length() )) {
        return a;
    }

    return Vector3D{NAN, NAN, NAN}; //NOTE or nullopt
}

void Line::print() const {
    std::cout << "origin = "; origin_.print();
    std::cout << ", dir = ";  dir_.print();
}
}// namespace geometry