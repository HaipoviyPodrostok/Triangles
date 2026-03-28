#pragma once

#include "line.hpp"
#include "vector_3d.hpp"

namespace geometry {

// -- plane -- ((r, n) = D)
class Plane {
 public:
  Plane(const Vector3D& point, const Vector3D& n);

  [[nodiscard]] const Vector3D& get_r() const { return r; }
  [[nodiscard]] const Vector3D& get_normal() const { return normal; }
  [[nodiscard]] double get_D() const { return D; }

  [[nodiscard]] bool is_valid() const;
  [[nodiscard]] double get_distance(const Plane& other) const;
  [[nodiscard]] bool is_match(const Plane& other) const;
  [[nodiscard]] bool is_parallel(const Plane& other) const;
  [[nodiscard]] bool is_contains(const Vector3D& p) const;
  [[nodiscard]] bool is_contains(const Line& line) const;
  [[nodiscard]] bool is_intersected(const Line& line) const;
  [[nodiscard]] std::optional<Vector3D> get_intersect_point(
      const Line& line) const;

 private:
  Vector3D r;
  Vector3D normal;
  double D;
};
}  // namespace geometry