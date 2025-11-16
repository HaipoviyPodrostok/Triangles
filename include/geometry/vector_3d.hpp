#pragma once

#include <cassert>
#include <cmath>
#include <cstddef>
#include <math.h>
#include <gtest/gtest.h>

namespace geometry {

// TODO template
struct Vector3D {
    float x;
    float y;
    float z;

    Vector3D(float x_ = NAN, float y_ = NAN, float z_ = NAN);

    static inline Vector3D invalid() { return {NAN, NAN, NAN}; }
    
    bool is_valid     ()                      const;
    bool is_zero      ()                      const;
    bool is_collinear (const Vector3D& other) const;
    bool is_codirected(const Vector3D& other) const;
    bool is_match     (const Vector3D& other) const;

    float length() const;
    float scalar(const Vector3D& other) const;

    Vector3D     operator+  (const Vector3D& other) const;
    Vector3D     operator-  (const Vector3D& other) const;
    Vector3D     operator*  (float scalar)          const;
    Vector3D     operator/  (float scalar)          const;
    float&       operator[] (size_t idx)                 ;
    const float& operator[] (size_t idx)            const; 
          
    Vector3D cross(const Vector3D& other) const;
    
    void print() const;
};
} // namespace geometry