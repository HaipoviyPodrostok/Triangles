#include "geometry/section.hpp"
#include "geometry/line.hpp"
#include "geometry/vector_3d.hpp"

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

// bool Section::is_intersect(const Section& other) const {
//     Line first_line  = get_line();
//     Line second_line = other.get_line();

// }

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
} // namespace geometry 

