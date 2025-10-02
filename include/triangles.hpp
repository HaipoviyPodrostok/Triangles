#pragma once

#include <cassert>
#include <cmath>
#include <math.h>
#include <gtest/gtest.h>
#include <stdexcept>

constexpr float flt_tolerance    = 1e-6f;
constexpr float inner_area_width = 100.0;

enum Area { LEFT_SIDE, INNER_SIDE, RIGHT_SIDE };

struct Point {   // TODO maybe via vector
    float x = NAN;
    float y = NAN;
    float z = NAN;

    void print() const { std::cout << "(" << x << "; " << y << "; " << z << ")"; }
    
    bool valid() const { return !(x != x || y != y || z != z); }
    
    bool is_equal(const Point& rhs) const {
        assert(valid() && rhs.valid());
        return (std::abs(x - rhs.x) < flt_tolerance) &&
               (std::abs(y - rhs.y) < flt_tolerance) &&
               (std::abs(z - rhs.y) < flt_tolerance); 
    }
};

struct Vector3D {
    float x = NAN;
    float y = NAN;
    float z = NAN;
    
    bool valid() const { return !(x != x || y != y || z != z); }
    
    bool operator==(const Vector3D& rhs) const {
        assert(valid() && rhs.valid());
        return (std::abs(x - rhs.x) < flt_tolerance) &&
               (std::abs(y - rhs.y) < flt_tolerance) &&
               (std::abs(z - rhs.z) < flt_tolerance); 
    }
    
    Vector3D operator+ (const Vector3D other) {
        return Vector3D{x + other.x, y + other.y, z + other.z};
    }

    float length() const {
        return (std::sqrtf(x * x + y * y + z * z));
    }
    
    Vector3D cross(const Vector3D other) const {
        if (other.length() < flt_tolerance ||
                  length() < flt_tolerance) {
            throw std::invalid_argument("Crossing of zero vectors");
        }
        
        float i = (y * other.z) - (z * other.y);
        float j = (z * other.x) - (x * other.z);
        float k = (x * other.y) - (y * other.x);

        return Vector3D{i, j, k};
    }

    bool collinear(const Vector3D other) const {
        if (this->cross(other) == Vector3D{0, 0, 0}) {
            return true;
        }
        return false;
    }
};

// -- line -- (r = origin + t * dir)
struct Line {
    Vector3D origin;
    Vector3D dir;

    Line(const Vector3D& origin_, const Vector3D& dir_)
        : origin(origin_), dir(dir_) {
        
        if (dir.x == 0 && dir.y == 0 && dir.z == 0) {
            throw std::invalid_argument("Direction vector can not be (0, 0, 0)");
        }
    }
    
    bool parallel(const Line& other) const {        
        if (dir.collinear(other.dir)) {
            return true;
        }
        return false;
    }
};
    // bool is_intersect(const Line& first, const Line& second) {

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



// -- plane -- ((r, n) + D = 0) 
struct Plane {
    Vector3D normal;
    float D;

    Plane(const Vector3D& point, const Vector3D& n)
        : normal(n) {
            
        D = -((n.x * point.x) + (n.y * point.y) + (n.z * point.z));
    }

    bool parallel(const Plane& other) {
        
        if (normal.collinear(other.normal)) {
            return true;
        }
    }

    bool match(const Plane& other) {

    }
};