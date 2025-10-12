#include "geometry/triangle.hpp"
#include "geometry/section.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <cassert>
#include <iostream>

namespace geometry {

Triangle::Triangle(const Vector3D& a, const Vector3D& b, const Vector3D& c)
    : a_(a), b_(b), c_(c), 
        ab_{a, b}, bc_{b, c}, ac_{a, c} {

    normal_ = (b_ - a_).cross(c_ - a_);
}

bool Triangle::is_valid() const {
    return a_.is_valid()  && b_.is_valid()  && c_.is_valid()  &&
           ab_.is_valid() && ac_.is_valid() && bc_.is_valid() &&
           (ab_.length() + bc_.length() > ac_.length())       &&
           (ab_.length() + ac_.length() > bc_.length())       &&
           (bc_.length() + ac_.length() > ab_.length());
}

void Triangle::print() const {
    std::cout << "A = ";   a_.print();
    std::cout << ", B = "; b_.print();
    std::cout << ", C = "; c_.print();
}

bool Triangle::intersection(const Triangle& other) const {
    const Plane first_pl  = this->find_plane();
    const Plane second_pl = other.find_plane();
    
    if (first_pl.is_match(second_pl)) {
        return is_intersect_2d(other);
    }

    if (first_pl.is_parallel(second_pl)) { return false; }

    return false;
}

Plane Triangle::find_plane() const {
    const Vector3D ab = {b_.x() - a_.x(), b_.y() - a_.y(), b_.z() - a_.z()};
    const Vector3D ac = {c_.x() - a_.x(), c_.y() - a_.y(), c_.z() - a_.z()};

    const Vector3D normal = ab.cross(ac);

    return Plane{a_, normal};
}

Side Triangle::get_side(const Vector3D& side_normal) const {
    float scalar = side_normal.scalar(normal_);
    if      (scalar > 0)              { return LEFT_SIDE; }
    else if (math::is_zero(scalar)) { return INTER_SIDE; }

    return RIGHT_SIDE;
}

bool Triangle::is_inside(const Vector3D& p) const {
    // Vector3D p = other.a_;

    const Vector3D n1 = (b_ - a_).cross(p - a_);
    const Vector3D n2 = (c_ - b_).cross(p - b_);
    const Vector3D n3 = (a_ - c_).cross(p - c_);

    const Side side_ab = get_side(n1);
    const Side side_bc = get_side(n2);
    const Side side_ca = get_side(n3);
    
    if (side_ab == side_bc && side_bc == side_ca) {
        return true;
    }

    return false;
}

bool Triangle::is_intersect_2d(const Triangle& other) const {
    assert(this->is_valid());
    assert(other.is_valid());

    if (ab_.is_intersect(other.ab_) ||
        ab_.is_intersect(other.bc_) ||
        ab_.is_intersect(other.ac_) ||
        bc_.is_intersect(other.ab_) ||
        bc_.is_intersect(other.bc_) ||
        bc_.is_intersect(other.ac_) ||
        ac_.is_intersect(other.ab_) ||
        ac_.is_intersect(other.bc_) ||
        ac_.is_intersect(other.ac_)) {
            
        return true;
    }

    if (is_inside(other.a_)) { return true; }
    if (is_inside(other.b_)) { return true; }
    if (is_inside(other.c_)) { return true; }

    if (other.is_inside(a_)) { return true; }
    if (other.is_inside(b_)) { return true; }
    if (other.is_inside(c_)) { return true; }

    return false;
}
} // namespace geometry