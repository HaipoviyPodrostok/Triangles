#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <cassert>
#include <cmath>

namespace geometry {

Vector3D::Vector3D(float x_, float y_, float z_)
    : x(x_), y(y_), z(z_) { }

bool Vector3D::is_valid() const {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

Vector3D Vector3D::operator+ (const Vector3D& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    return Vector3D{x + other.x, y + other.y, z + other.z};
}

Vector3D Vector3D::operator- (const Vector3D& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    return Vector3D{x - other.x, y - other.y, z - other.z};
}

Vector3D Vector3D::operator* (const float& scalar) const {
    assert(this->is_valid());
    return Vector3D{x * scalar, y * scalar, z * scalar};
}

Vector3D Vector3D::operator/ (const float& scalar) const {
    assert(this->is_valid());
    assert(!math::is_zero(scalar));
    return *this * (1 / scalar);
}

bool Vector3D::is_collinear(const Vector3D& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    return ( (this->cross(other) ).is_zero() );
}

bool Vector3D::is_codirected(const Vector3D& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    return (is_collinear(other) && this->scalar(other) >= 0.0f);
}

bool Vector3D::is_match(const Vector3D& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    return ((*this - other).is_zero());
}

bool Vector3D::is_zero() const {
    assert(this->is_valid());
    return (math::is_zero(length())) ;
}

float Vector3D::length() const {
    assert(this->is_valid());
    return (std::sqrtf(x * x + y * y + z * z));
}

float Vector3D::scalar(const Vector3D& other) const {
    assert(this->is_valid()); 
    return (x * other.x + y * other.y + z * other.z);
}

Vector3D Vector3D::cross(const Vector3D& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
 
    float i = (y * other.z) - (z * other.y);
    float j = (z * other.x) - (x * other.z);
    float k = (x * other.y) - (y * other.x);
    
    return Vector3D{i, j, k};
}

void Vector3D::print() const {
    std::cout << "(" << x << ", " << y << ", " << z << ")";
}
} // namespace geometry