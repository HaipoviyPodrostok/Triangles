#pragma once

#include <cassert>
#include <cmath>
#include <cstddef>
#include <math.h>
#include <gtest/gtest.h>

namespace geometry {

struct Vector3D {
    double x;
    double y;
    double z;

    Vector3D(double x_ = NAN, double y_ = NAN, double z_ = NAN);

    static inline Vector3D invalid() {
        return {NAN, NAN, NAN};
    }
    
    bool is_valid     ()                      const;
    bool is_zero      (double scale = 1.0)    const;
    bool is_collinear (const Vector3D& other) const;
    bool is_codirected(const Vector3D& other) const;
    bool is_match     (const Vector3D& other) const;

    double length() const;
    double scalar(const Vector3D& other) const;

    Vector3D      operator+  (const Vector3D& other) const;
    Vector3D      operator-  (const Vector3D& other) const;
    Vector3D      operator*  (double scalar)         const;
    Vector3D      operator/  (double scalar)         const;
    double&       operator[] (size_t idx)                 ;
    const double& operator[] (size_t idx)            const; 
          
    Vector3D cross(const Vector3D& other) const;
    
    void print() const;
};
} // namespace geometry