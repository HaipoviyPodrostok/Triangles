#pragma once

#include <cassert>
#include <math.h>
#include <gtest/gtest.h>

constexpr float flt_tolerance    = 1e-6f;
constexpr float inter_area_width = 100.0;

enum area_t { LEFT_SIDE, INNER_SIDE, RIGHT_SIDE };

struct point_t {
    float x = NAN;
    float y = NAN;
    
    void print() const { std::cout << "(" << x << " ; " << y << ")"; }
    
    bool valid() const { return !(x != x || y != y); }
    
    bool is_equal(const point_t &rhs) const {
        assert(valid() && rhs.valid());
        return (std::abs(x - rhs.x) < flt_tolerance) &&
               (std::abs(y - rhs.y) < flt_tolerance); 
    }
};

// ax + by + c = 0
struct line_t {
    float a = -1.0f;
    float b = -1.0f;
    float c =  0.0f;

    void print() const { std::cout << a << "x + " << b << "y + " << c << " = 0"; }
    
    line_t(const point_t &p1, const point_t &p2) {
        float angle = std::atan((p2.y - p1.y) / (p2.x - p1.x));
        float sin_angle = std::sin(angle);
        float cos_angle = std::sqrt(1.0 - sin_angle * sin_angle);
        point_t normal_vector{-sin_angle, cos_angle};
        a = normal_vector.x;
        b = normal_vector.y;
        c = -(p1.x * normal_vector.x + p1.y * normal_vector.y);
    }
    
    enum area_t get_side_area(const point_t &point) const {
        float side_offset = a * point.x + b * point.y + c;
        if (side_offset > 0.0 + flt_tolerance * inter_area_width) {
            return LEFT_SIDE;
        }

        if (side_offset > 0.0 - flt_tolerance * inter_area_width) {
            return INNER_SIDE;
        }

        return RIGHT_SIDE;
    }

    bool separates(const point_t &point1, const point_t &point2) const {
        area_t side1 = get_side_area(point1);
        area_t side2 = get_side_area(point2);
        if (side1 == INNER_SIDE || side2 == INNER_SIDE) {
            return false;
        }
        return !(side1 == side2);
    }

    bool valid() const { return !(a != a || b != b || c != c); }
};
