#pragma once

#include <cassert>
#include <math.h>
#include <gtest/gtest.h>

#include "vector_3d.hpp"
#include "section.hpp"
#include "plane.hpp"

namespace geometry {

class Triangle {
public:
    Triangle(const Vector3D& a, const Vector3D& b, const Vector3D& c);
    bool is_valid() const;
    void print() const;
    bool intersection(const Triangle& other) const;
    
    bool is_intersect_2d(const Triangle& other) const;
    bool is_inside(const Vector3D& p) const;
    bool is_intersect_3d(const Triangle& other) const;

private:    
    Vector3D a_;
    Vector3D b_;
    Vector3D c_;
    
    Section ab_;
    Section bc_;
    Section ac_;
    
    Vector3D normal_;
    
    Plane get_plane() const;
    Line get_intersect_line(const Triangle& other) const;
};
} // namespace geometry