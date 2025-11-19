#pragma once

#include <cassert>
#include <math.h>
#include <gtest/gtest.h>

#include "section.hpp"
#include "vector_3d.hpp"
#include "line.hpp"
#include "plane.hpp"

namespace geometry {

class Triangle {
public:
    Vector3D a;
    Vector3D b;
    Vector3D c;
    
    Vector3D centroid;
    
    Triangle(const Vector3D& a, const Vector3D& b, const Vector3D& c);
    
    Plane get_plane() const;
    Line get_intersect_line(const Triangle& other) const;
    void print() const;
    Vector3D get_centre() const;
    
    bool is_valid() const;
    bool is_intersect(const Section& sec) const; 
    bool is_intersect(const Triangle& other) const;
    bool is_point() const;
    bool is_section() const;
    bool is_inside(const Vector3D& p) const;
   
private:
   bool is_intersect_2d(const Triangle& other) const;
   bool is_intersect_3d(const Triangle& other) const;
};
} // namespace geometry
