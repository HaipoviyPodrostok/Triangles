#pragma once

#include <math.h>
#include <gtest/gtest.h>

struct point_t {
    float x = NAN;
    float y = NAN;
    
    void print() const;
    bool valid() const;
    bool equal(const point_t &rhs) const;   //TODO std::abs(x - rhs.x) < fit_tolerance
};

// ax + by + c = 0
struct line_t {
    float a = -1.0f;
    float b = -1.0f;
    float c =  0.0f;

    void print() const;
    bool valid() const;
    line_t(const point_t &p1, const point_t &p2);
};