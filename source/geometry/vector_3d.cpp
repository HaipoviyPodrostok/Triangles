#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <cassert>

namespace geometry {

Vector3D::Vector3D(float x, float y, float z)
    : x_(x), y_(y), z_(z) { }

float Vector3D::x() const { return x_; }
float Vector3D::y() const { return y_; }
float Vector3D::z() const { return z_; }

Vector3D Vector3D::operator+ (const Vector3D& other) const {
    return Vector3D{x_ + other.x_, y_ + other.y_, z_ + other.z_};
}

Vector3D Vector3D::operator- (const Vector3D& other) const {
    return Vector3D{x_ - other.x_, y_ - other.y_, z_ - other.z_};
}

Vector3D Vector3D::operator* (const float& scalar) const {
    return Vector3D{x_ * scalar, y_ * scalar, z_ * scalar};
}
 
bool Vector3D::is_valid() const {
    return std::isfinite(x_) && std::isfinite(y_) && std::isfinite(z_);
}

bool Vector3D::is_collinear(const Vector3D& other) const {
    Vector3D cross_res = this->cross(other);
    return (cross_res.is_zero());        
}
bool Vector3D::is_zero() const {
    return (length() < flt_tolerance) ;
}

float Vector3D::length() const {
    return (std::sqrtf(x_ * x_ + y_ * y_ + z_ * z_));
}

float Vector3D::scalar(const Vector3D& other) const {
    return (x_ * other.x_ + y_ * other.y_ + z_ * other.z_);
}

Vector3D Vector3D::cross(const Vector3D& other) const {
    if (other.is_zero() ||
        this->is_zero()) {
        throw std::invalid_argument("Crossing of zero vectors");
    }
    
    float i = (y_ * other.z_) - (z_ * other.y_);
    float j = (z_ * other.x_) - (x_ * other.z_);
    float k = (x_ * other.y_) - (y_ * other.x_);
    
    return Vector3D{i, j, k};
}

void Vector3D::print() const {
    std::cout << "(" << x_ << ", " << y_ << ", " << z_ << ")";
}
} // namespace geometry