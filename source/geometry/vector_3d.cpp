#include "geometry/vector_3d.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "math/math.hpp"

namespace geometry {

bool Vector3D::is_valid() const {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

Vector3D operator+(const Vector3D& lhs, const Vector3D& rhs) noexcept {
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vector3D operator-(const Vector3D& lhs, const Vector3D& rhs) noexcept {
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vector3D operator*(const Vector3D& v, double scalar) noexcept {
  return {v.x * scalar, v.y * scalar, v.z * scalar};
}

Vector3D operator*(double scalar, const Vector3D& v) noexcept {
  return v * scalar;
}

Vector3D operator/(const Vector3D& v, double scalar) noexcept {
  return {v.x / scalar, v.y / scalar, v.z / scalar};
}

Vector3D operator/(double scalar, const Vector3D& v) noexcept {
  return {scalar / v.x, scalar / v.y, scalar / v.z};
}

double& Vector3D::operator[](size_t idx) {
  assert(this->is_valid());
  if (idx == 0) {
    return x;
  };
  if (idx == 1) {
    return y;
  };
  if (idx == 2) {
    return z;
  };
  throw std::out_of_range("Vector3D index out of range");
  ;
}

bool Vector3D::is_collinear(const Vector3D& other) const {
  assert(this->is_valid());
  assert(other.is_valid());

  const double scale = this->length() * other.length();
  return ((this->cross(other)).is_zero(scale));
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

bool Vector3D::is_zero(double scale) const {
  assert(this->is_valid());
  return (math::is_zero(length(), scale));
}

double Vector3D::length() const {
  assert(this->is_valid());
  return (std::sqrtf(x * x + y * y + z * z));
}

double Vector3D::scalar(const Vector3D& other) const {
  assert(this->is_valid());
  assert(other.is_valid());
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
}  // namespace geometry