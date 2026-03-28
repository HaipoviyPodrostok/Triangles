#pragma once

#include <cassert>

#include "geometry/geometry.hpp"

namespace acceleration {

struct AABB {
  geometry::Vector3D min;
  geometry::Vector3D max;

  AABB() = default;
  AABB(const geometry::Vector3D min_, const geometry::Vector3D max_)
      : min(min_), max(max_) {}
  AABB(const geometry::Triangle& tri);

  [[nodiscard]] bool is_valid() const;
  [[nodiscard]] bool is_intersect(const AABB& other) const;
  [[nodiscard]] bool is_inside(const AABB& other) const;
  [[nodiscard]] bool is_contains(const AABB& other) const;

  void expand(const geometry::Vector3D& p);
  void merge(const AABB& other);

  void add_tri_to_aabb(const geometry::Triangle& tri, AABB& aabb);
};

[[nodiscard]] AABB merge(const AABB& a, const AABB& b);

}  // namespace acceleration