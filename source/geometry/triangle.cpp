#include "geometry/triangle.hpp"
#include <cassert>

namespace geometry {

Triangle::Triangle(const Vector3D& a, const Vector3D& b, const Vector3D& c)
    : a_(a), b_(b), c_(c), 
        ab_{a, b}, bc_{b, c}, ac_{a, c} { }

bool Triangle::is_valid() const {
    return a_.is_valid()  && b_.is_valid()  && c_.is_valid()  &&
           ab_.is_valid() && ac_.is_valid() && bc_.is_valid() &&
           (ab_.length() + bc_.length() > ac_.length())       &&
           (ab_.length() + ac_.length() > bc_.length())       &&
           (bc_.length() + ac_.length() > ab_.length());
}

bool Triangle::intersection(const Triangle& other) const {
    Plane first_pl  = this->find_plane();
    Plane second_pl = other.find_plane();

    if (first_pl.parallel(second_pl)) { return false; }

    if (first_pl.match(second_pl)) {
        //TODO return intersect_2d(other);
    }

    else {

    }

    return false;
}

Plane Triangle::find_plane() const {
    Vector3D ab = {b_.x() - a_.x(), b_.y() - a_.y(), b_.z() - a_.z()};
    Vector3D ac = {c_.x() - a_.x(), c_.y() - a_.y(), c_.z() - a_.z()};

    Vector3D normal = ab.cross(ac);

    return Plane{a_, normal};
}

Triangle Triangle::change_basis(const Plane& plane) const {
    assert(this->is_valid());
    assert(plane.is_valid());

    
}


bool Triangle::is_intersect_2d(const Triangle& other) const {
    assert(this->is_valid());
    assert(other.is_valid());

    Triangle this_new_basis = change_basis()

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

    if ()

}

} // namespace geometry