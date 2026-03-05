#pragma once

#include <cassert>

#include "geometry/vector_3d.hpp"

namespace acceleration {

struct AABB {
  geometry::Vector3D min;
  geometry::Vector3D max;

  AABB() = default;
  AABB(const geometry::Vector3D min_, const geometry::Vector3D max_)
      : min(min_), max(max_) {}

  [[nodiscard]] bool is_valid() const noexcept;
  [[nodiscard]] bool is_intersect(const AABB& other) const noexcept;
  [[nodiscard]] bool is_inside(const AABB& other) const noexcept;

  void expand(const geometry::Vector3D& p) noexcept;
  void merge(const AABB& other) noexcept;
};

[[nodiscard]] AABB merge(const AABB& a, const AABB& b) noexcept;

}  // namespace acceleration