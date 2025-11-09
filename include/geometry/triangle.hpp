#pragma once

#include <cassert>
#include <math.h>
#include <gtest/gtest.h>

#include "vector_3d.hpp"
#include "line.hpp"
#include "plane.hpp"

namespace geometry {

struct Triangle {
    Vector3D a;
    Vector3D b;
    Vector3D c;
    
    Vector3D centroid;
    
    Triangle(const Vector3D& a, const Vector3D& b, const Vector3D& c);
    
    Plane get_plane() const;
    Line get_intersect_line(const Triangle& other) const;
    bool is_valid() const;
    void print() const;
    bool intersection(const Triangle& other) const;
    
    bool is_intersect_2d(const Triangle& other) const;
    bool is_inside(const Vector3D& p) const;
    bool is_intersect_3d(const Triangle& other) const;
    Vector3D get_centre() const;
};
} // namespace geometry
