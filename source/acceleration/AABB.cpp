#include "acceleration/AABB.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>

#include "geometry/geometry.hpp"
#include "math/math.hpp"

namespace acceleration {

bool AABB::is_valid() const noexcept {
  return min.is_valid() && max.is_valid() && min.x < max.x && min.y < max.y &&
         min.z < max.z && !min.is_match(max);
}

bool AABB::is_intersect(const AABB& other) const noexcept {
  assert(this->is_valid());
  assert(other.is_valid());

  const geometry::Vector3D& minA = this->min;
  const geometry::Vector3D& maxA = this->max;
  const geometry::Vector3D& minB = other.min;
  const geometry::Vector3D& maxB = other.max;

  return (minA.x <= maxB.x + math::eps) && (math::eps + maxA.x >= minB.x) &&
         (minA.y <= maxB.y + math::eps) && (math::eps + maxA.y >= minB.y) &&
         (minA.z <= maxB.z + math::eps) && (math::eps + maxA.z >= minB.z);
}

void AABB::expand(const geometry::Vector3D& p) noexcept {
  assert(this->is_valid());
  assert(p.is_valid());

  for (size_t i = 0; i < 3; ++i) { min[i] = std::fmin(min[i], p[i]); }
}

void AABB::merge(const AABB& other) noexcept {
  assert(this->is_valid());
  assert(other.is_valid());

  for (size_t i = 0; i < 3; ++i) {
    min[i] = std::fmin(min[i], other.min[i]);
    max[i] = std::fmax(max[i], other.max[i]);
  }
}

AABB merge(AABB& a, AABB& b) noexcept {
  assert(a.is_valid() && b.is_valid());
  a.merge(b);
  return a;
}

bool AABB::is_inside(const AABB& other) const noexcept {
  assert(this->is_valid());
  assert(other.is_valid());

  return min.x < other.min.x + math::eps && min.y < other.min.y + math::eps &&
         min.z < other.min.z + math::eps && max.x < other.max.x + math::eps &&
         max.y < other.max.y + math::eps && max.z < other.max.z + math::eps;
}

}  // namespace acceleration