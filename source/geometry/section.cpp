#include "geometry/section.hpp"
#include "geometry/line.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>

namespace geometry {

Section::Section(const Vector3D& a, const Vector3D& b)
    : a(a), b(b) { }

bool Section::is_valid() const {
    return a.is_valid() &&
           b.is_valid() &&
           !((a - b).is_zero());
}

Line Section::get_line() const {
    this->is_valid();
    return Line{a, b - a};
}

bool Section::is_intersect(const Section& other) const {    
    assert(this->is_valid());
    assert(other.is_valid());
    
    const Line l1 = this->get_line();
    const Line l2 = other.get_line();
    
    if (l1.is_match(l2)) {
        const Vector3D d = b - a;
        
        const double abs_x = std::fabs(d.x);
        const double abs_y = std::fabs(d.y);
        const double abs_z = std::fabs(d.z);

        math::Axis axis = (abs_x >= abs_y && abs_x >= abs_z) ? math::Axis::X :
                    (abs_y >= abs_z ? math::Axis::Y : math::Axis::Z);
            
        auto get_1d_coord = [&](const Vector3D& v) {
            switch (axis) {
                case math::Axis::X:
                    return v.x;
                case math::Axis::Y:
                    return v.y;
                case math::Axis::Z:
                    return v.z;
                default:
                    throw std::logic_error("Invalid axis in get_1d_coord()");
            }
        };
        
        double a1 = get_1d_coord(a);       double b1 = get_1d_coord(b);
        double a2 = get_1d_coord(other.a); double b2 = get_1d_coord(other.b);

        if (a1 > b1) { std::swap(a1, b1); }
        if (a2 > b2) { std::swap(a2, b2); }

        const double left_max  = std::max(a1, a2);
        const double right_min = std::min(b1, b2);

        return right_min + math::eps >= left_max;
    }

    if (l1.is_intersect(l2)) {
        const Vector3D p = l1.intersect_point(l2);
        assert(p.is_valid());
        return ( this->is_contains(p) && other.is_contains(p) );
    }    

    return false;
}

bool Section::is_intersect(const Line& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    
    const Line l1 = this->get_line();

    if (l1.is_match(other)) {
        return true;
    }

    if (!l1.is_intersect(other)) {
        return false;
    }

    const Vector3D p = l1.intersect_point(other);
    if (!p.is_valid()) {
        return false;
    }

    return this->is_contains(p);
}

bool Section::is_belong(const Line& line) const {
    assert(is_valid());
    assert(line.is_valid());
    return line.is_contains(a) && line.is_contains(b);
}

Vector3D Section::intersect_point(const Line& line) const {
    assert(this->is_valid());
    assert(line.is_valid());
    assert(this->is_intersect(line));

    const Line this_line = this->get_line();

    if (this_line.is_match(line)) {
        return a;
    }

    return this_line.intersect_point(line);
}

double Section::length() const {
    return (a - b).length();
}

bool Section::is_contains(const Vector3D& p) const {
    this->is_valid();
    p.is_valid();

    if (!p.is_valid()) { return false; }
    
    const Vector3D ap = p - a;
    const Vector3D ab = b - a;
    const Vector3D bp = p - b;

    if ((ap).is_collinear(ab)) {
        double scalar_ap_bp = ap.scalar(bp);
        return (scalar_ap_bp < 0 || math::is_zero(scalar_ap_bp));
    }

    return false;
}

void Section::print() const {
    std::cout << "p1 = ";   a.print();
    std::cout << ", p2 = "; b.print();
    std::cout << std::endl;
}
} // namespace geometry 
