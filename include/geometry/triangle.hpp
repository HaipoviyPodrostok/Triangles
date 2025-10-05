#pragma once

#include <cassert>
#include <math.h>
#include <gtest/gtest.h>

#include "geometry/vector_3d.hpp"
#include "geometry/section.hpp"
#include "geometry/plane.hpp"

namespace geometry {

class Triangle {
public:
    Triangle(const Vector3D& a, const Vector3D& b, const Vector3D& c);
    bool is_valid() const;
    bool intersection(const Triangle& other) const;

    Triangle change_basis(const Plane& plane) const;

private:
    Vector3D a_;
    Vector3D b_;
    Vector3D c_;

    Section ab_;
    Section bc_;
    Section ac_;
    
    Plane find_plane() const;
    bool is_intersect_2d(const Triangle& other) const;
};
} // namespace geometry


    // bool is_intersect(const Line& first, const Line& second)    
    // }
        
    // Line(const Vector3D& point, const Vector3D& normal)
    //     : 
    
    
    
    
    // float a = -1.0f;
    // float b = -1.0f;
    // float c =  0.0f;
    
    // void print() const { std::cout << a << "x + " << b << "y + " << c << " = 0"; }
    
    // line_t(const point_t &p1, const point_t &p2) {
    //     float angle = std::atan((p2.y - p1.y) / (p2.x - p1.x));
    //     float sin_angle = std::sin(angle);
    //     float cos_angle = std::sqrt(1.0 - sin_angle * sin_angle);
    //     point_t normal_vector{-sin_angle, cos_angle};
    //     a = normal_vector.x;
    //     b = normal_vector.y;
    //     c = -(p1.x * normal_vector.x + p1.y * normal_vector.y);
    // }
    
    // enum area_t get_side_area(const point_t &point) const {
    //     float side_offset = a * point.x + b * point.y + c;
    //     if (side_offset > 0.0 + float_tolerance * inner_area_width) {
    //         return LEFT_SIDE;
    //     }
    
    //     if (side_offset > 0.0 - float_tolerance * inner_area_width) {
    //         return INNER_SIDE;
    //     }
    
    //     return RIGHT_SIDE;
    // }
    
    // bool separates(const point_t &point1, const point_t &point2) const {
    //     area_t side1 = get_side_area(point1);
    //     area_t side2 = get_side_area(point2);
    //     if (side1 == INNER_SIDE || side2 == INNER_SIDE) {
    //         return false;
    //     }
    //     return !(side1 == side2);
    // }
    
    // bool valid() const { return !(a != a || b != b || c != c); }


// struct Point {   // TODO maybe via vector
//     float x = NAN;
//     float y = NAN;
//     float z = NAN;

//     void print() const { std::cout << "(" << x << "; " << y << "; " << z << ")"; }
    
//     bool valid() const { return !(x != x || y != y || z != z); }
    
//     bool is_equal(const Point& rhs) const {
//         assert(valid() && rhs.valid());
//         return (math::is_equal(x, rhs.x)) &&
//                (math::is_equal(y, rhs.y)) &&
//                (math::is_equal(z, rhs.z)); 
//     }
// };
