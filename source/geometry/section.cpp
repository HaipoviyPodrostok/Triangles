#include "geometry/section.hpp"
#include "geometry/line.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>

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

bool Section::is_intersect(const Section& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    
    const Line l1 = this->get_line();
    const Line l2 = other.get_line();

    if (l1.is_intersect(l2)) {
        const Vector3D p = l1.intersect_point(l2);
        if (!p.is_valid()) { return false; }
        if ( this->is_contains(p) && other.is_contains(p) ) {
            return true;
        }
    }

    if (!l1.is_contains(other.a_)) {
        return false;
    }

    const Vector3D d = b_ - a_;
    
    const float abs_x = fabs(d.x_);
    const float abs_y = fabs(d.y_);
    const float abs_z = fabs(d.z_);

    Axis axis = (abs_x >= abs_y && abs_x >= abs_z) ? Axis::X :
                (abs_y >= abs_z ? Axis::Y : Axis::Z);
        
    auto get_1d_coord = [&](const Vector3D& v) {
        switch (axis) {
            case Axis::X:
                return v.x_;
            case Axis::Y:
                return v.y_;
            case Axis::Z:
                return v.z_;
            default:
                return NAN;
        }
    };
    
    float a1 = get_1d_coord(a_);       float b1 = get_1d_coord(b_);
    float a2 = get_1d_coord(other.a_); float_t b2 = get_1d_coord(other.b_);

    if (a1 > b1) { std::swap(a1, b1); }
    if (a2 > b2) { std::swap(a2, b2); }

    const float left_max  = std::max(a1, a2);
    const float right_min = std::min(b1, b2);

    return right_min + math::eps >= left_max;
}

bool Section::is_intersect(const Line& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    
    const Line l1 = this->get_line();

    if (l1.is_intersect(other)) {
        const Vector3D p = l1.intersect_point(other);
        if (p.is_valid()) { return false; }
        return this->is_contains(p);
    }

    return false;
}

Vector3D Section::intersect_point(const Line& other) const {
    assert(other.is_valid());

    const Line this_line = this->get_line();
    if (this_line.is_match(other) || this_line.is_parallel(other)) {
        return {NAN, NAN, NAN};
    }

    return this_line.intersect_point(other);
}

float Section::length() const {
    return (a_ - b_).length();
}

bool Section::is_contains(const Vector3D& p) const {
    if (p.is_valid()) { return false; }
    
    const Vector3D ap = p - a_;
    const Vector3D ab = b_ - a_;
    const Vector3D bp = p - b_;

    if ((ap).is_collinear(ab)) {
        float scalar_ap_bp = ap.scalar(bp);
        return (scalar_ap_bp < 0 || math::is_zero(scalar_ap_bp));
    }

    return false;
}

void Section::print() const {
    std::cout << "p1 = ";   a_.print();
    std::cout << ", p2 = "; b_.print();
    std::cout << std::endl;
}
} // namespace geometry 