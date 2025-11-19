#pragma once

#include <cassert>
#include <cmath>
#include <cstddef>
#include <math.h>
#include <gtest/gtest.h>

namespace geometry {

// TODO template
template<typename T>
struct Vector3DBase {
    T x;
    T y;
    T z;

    Vector3DBase(T x = NAN, T y = NAN, T z = NAN);

    static inline Vector3DBase invalid() { return {NAN, NAN, NAN}; }
    
    bool is_valid     ()                      const;
    bool is_zero      ()                      const;
    bool is_collinear (const Vector3DBase& other) const;
    bool is_codirected(const Vector3DBase& other) const;
    bool is_match     (const Vector3DBase& other) const;

    float length() const;
    float scalar(const Vector3DBase& other) const;

    Vector3DBase     operator+  (const Vector3DBase& other) const;
    Vector3DBase     operator-  (const Vector3DBase& other) const;
    Vector3DBase     operator*  (float scalar)          const;
    Vector3DBase     operator/  (float scalar)          const;
    float&       operator[] (size_t idx)                 ;
    const float& operator[] (size_t idx)            const; 
          
    Vector3DBase cross(const Vector3DBase& other) const;
    
    void print() const;
};
} // namespace geometry