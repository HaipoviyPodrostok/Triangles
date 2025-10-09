#include "geometry/section.hpp"
#include "geometry/line.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <cassert>

namespace geometry {

Section::Section(const Vector3D& a, const Vector3D& b)
    : a_(a), b_(b) { }

bool Section::is_valid() const {
    return a_.is_valid() &&
           b_.is_valid() &&
           !((a_ - b_).is_zero());
}

Line Section::get_line() const {
    return Line{a_, b_ - a_};
}

bool Section::is_intersect(const Section& other) const {  // TODO failed test
    assert(this->is_valid());
    assert(other.is_valid());
    
    Line first_line  = get_line();
    Line second_line = other.get_line();

    if (first_line.is_intersect(second_line)) {
        Vector3D p = first_line.intersect_point(second_line);
        if ( this->is_contains(p) && other.is_contains(p) ) {
            return true;
        }
    }
    
    return false;
}

float Section::length() const {
    return (a_ - b_).length();
}

Side Section::get_side(const Vector3D& p) const {
    Vector3D ab = b_ - a_;
    Vector3D ap = p  - a_;

    Vector3D n = ab.cross(ap);
    float n_length = n.length();

    if (n_length > 0) { return LEFT_SIDE; }
    if (n_length < 0) { return  RIGHT_SIDE; }

    return INTER_SIDE;
}

bool Section::is_contains(const Vector3D& p) const { // TODO failed test
    Vector3D ap = p - a_;
    Vector3D ab = b_ - a_;
    Vector3D bp = p - b_;

    if ((ap).is_collinear(ab)) {
        float scalar_ab_bp = ap.scalar(bp);
        if (scalar_ab_bp < 0 || math::is_zero(scalar_ab_bp)) {
            return true;
        }
    }

    return false;
}

void Section::print() const {
    std::cout << "p1 = ";   a_.print();
    std::cout << ", p2 = "; b_.print();
    std::cout << std::endl;
}
} // namespace geometry 