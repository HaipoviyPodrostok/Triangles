#pragma once

#include <cassert>
#include <cmath>
#include <math.h>
#include <gtest/gtest.h>

namespace geometry {

class Vector3D {
public:   
    Vector3D(float x = NAN, float y = NAN, float z = NAN);

    float x() const;
    float y() const;
    float z() const;
    
    bool is_valid() const;
    bool is_zero()  const;
    bool is_collinear(const Vector3D& other) const;
    bool is_codirected(const Vector3D& other) const;

    float length() const;
    float scalar(const Vector3D& other) const;

    Vector3D operator+ (const Vector3D& other)  const;
    Vector3D operator- (const Vector3D& other)  const;
    Vector3D operator* (const float& scalar) const;

    Vector3D cross(const Vector3D& other) const;
    
    void print() const;

private:
    float x_ = NAN;
    float y_ = NAN;
    float z_ = NAN;
};
} // namespace geometry