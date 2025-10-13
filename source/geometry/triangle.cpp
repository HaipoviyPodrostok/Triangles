#include "geometry/triangle.hpp"
#include "geometry/plane.hpp"
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
    assert(this->is_valid());
    assert(other.is_valid());
    
    const Plane first_pl  = this->get_plane();
    const Plane second_pl = other.get_plane();
    
    if (first_pl.is_match(second_pl)) {
        return is_intersect_2d(other);
    }

    if (first_pl.is_parallel(second_pl)) { return false; }

    return is_intersect_3d(other);
}

Plane Triangle::get_plane() const {
    const Vector3D ab = b_ - a_;
    const Vector3D ac = c_ - a_;

    const Vector3D normal = ab.cross(ac);

    return Plane{a_, normal};
}

bool Triangle::is_inside(const Vector3D& p) const {
    assert(p.is_valid());
    if ( !(this->get_plane()).is_contains(p) ) { return false; }

    const Vector3D ab = b_ - a_;
    const Vector3D bc = c_ - b_;
    const Vector3D ca = a_ - c_;
    const Vector3D ap = p - a_;
    const Vector3D bp = p - b_;
    const Vector3D cp = p - c_;

    const float side_ab = ( normal_.cross(ab) ).scalar(ap);
    const float side_bc = ( normal_.cross(bc) ).scalar(bp);
    const float side_ca = ( normal_.cross(ca) ).scalar(cp);

    return (side_ab >= - math::eps && side_bc >= - math::eps && side_ca >= - math::eps) ||
           (side_ab <=   math::eps && side_bc <=   math::eps && side_ca <=   math::eps);
}

bool Triangle::is_intersect_2d(const Triangle& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    if ( !(this->get_plane().is_match(other.get_plane())) ) { return false;}

    const Section* sides1[3] = {&ab_, &bc_, &ac_};
    const Section* sides2[3] = {&other.ab_, &other.bc_, &other.ac_};
    
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            sides1[i]->is_intersect(*sides2[j]);
        }
    }

    if (is_inside(other.a_) || is_inside(other.b_) || is_inside(other.c_)) {
        return true;
    }

    if (other.is_inside(a_) || other.is_inside(b_) || other.is_inside(c_)) {
        return true;
    }

    return false;
}

bool Triangle::is_intersect_3d(const Triangle& other) const {
    assert(this->is_valid());
    assert(other.is_valid());

    const Line intersect_line = get_intersect_line(other);

    const Line this_line_ab = ab_.get_line();
    const Line this_line_bc = bc_.get_line(); 
    const Line this_line_ac = ac_.get_line(); 

    const Line other_line_ab = other.ab_.get_line();
    const Line other_line_bc = other.bc_.get_line();
    const Line other_line_ac = other.ac_.get_line();

    Vector3D intersect_sec_a = {NAN, NAN, NAN};
    Vector3D intersect_sec_b = {NAN, NAN, NAN};

    if (ab_.is_intersect(intersect_line)) {

    }


}

Line Triangle::get_intersect_line(const Triangle& other) const {
        const Plane pl1 = get_plane();
    const Plane pl2 = other.get_plane();
    
    const float D1 = pl1.D();
    const float D2 = pl2.D();
    
    const Vector3D n1 = pl1.normal();
    const Vector3D n2 = pl2.normal(); 

    const Vector3D dir = normal_.cross(other.normal_);

    const Vector3D pl_intersect_p = (n2 * D1 - n1 * D2).cross(n1.cross(n2)) /
                                        math::sqr(( n1.cross(n2) ).length());

    return {pl_intersect_p, dir};
}
} // namespace geometry