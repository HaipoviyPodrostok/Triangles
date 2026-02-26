#pragma once

#include <gtest/gtest.h>

#include <cassert>
#include <cmath>
#include <cstddef>

namespace geometry {

struct Vector3D {
  double x;
  double y;
  double z;

  Vector3D(double x_ = NAN, double y_ = NAN, double z_ = NAN) noexcept
      : x(x_), y(y_), z(z_) {}

  static inline Vector3D invalid() { return {NAN, NAN, NAN}; }

  [[nodiscard]] bool is_valid() const;
  [[nodiscard]] bool is_zero(double scale = 1.0) const;
  [[nodiscard]] bool is_collinear(const Vector3D& other) const;
  [[nodiscard]] bool is_codirected(const Vector3D& other) const;
  [[nodiscard]] bool is_match(const Vector3D& other) const;

  [[nodiscard]] double length() const;
  [[nodiscard]] double scalar(const Vector3D& other) const;

  double& operator[](size_t idx);
  const double& operator[](size_t idx) const;

  [[nodiscard]] Vector3D cross(const Vector3D& other) const;

  void print() const;
};

Vector3D operator+(const Vector3D& lhs, const Vector3D& rhs) noexcept;
Vector3D operator-(const Vector3D& lhs, const Vector3D& rhs) noexcept;
Vector3D operator*(const Vector3D& v, double scalar) noexcept;
Vector3D operator*(double scalar, const Vector3D& v) noexcept;
Vector3D operator/(const Vector3D& v, double scalar) noexcept;
Vector3D operator/(double scalar, const Vector3D& v) noexcept;
}  // namespace geometry