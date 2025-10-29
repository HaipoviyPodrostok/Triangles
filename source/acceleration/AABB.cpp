#include <cassert>

#include "geometry/triangle.hpp"
#include "geometry/vector_3d.hpp"
#include "geometry/geometry.hpp"
#include "math/math_utils.hpp"
#include "acceleration/AABB.hpp"


namespace aabb {

AABB::AABB(const geometry::Triangle& tri)
    : min{std::fmin( std::fmin( tri.a.x, tri.b.x ), tri.c.x ),
          std::fmin( std::fmin( tri.a.y, tri.b.y ), tri.c.y ),
          std::fmin( std::fmin( tri.a.z, tri.b.z ), tri.c.z ) }, 
      max{std::fmax( std::fmax( tri.a.x, tri.b.x ), tri.c.x ),
          std::fmax( std::fmax( tri.a.y, tri.b.y ), tri.c.y ),
          std::fmax( std::fmax( tri.a.z, tri.b.z ), tri.c.z ) }
{
    assert(min.is_valid() && max.is_valid()); 
}

bool AABB::is_valid() const {
    return min.is_valid() && max.is_valid() &&
           min.x < max.x  && min.y < max.y  && min.z < max.z &&
           !min.is_match(max);
}

bool AABB::is_intersect(const AABB& other) const {
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

void AABB::expand(const geometry::Triangle& tri) {
    assert(this->is_valid());
    assert(tri.is_valid());

    const geometry::Vector3D tri_points[3] = {tri.a, tri.b, tri.c};

    for (size_t i = 0; i < 3; ++i) {
        min.x = fmin(min.x, tri_points[i].x);
        min.y = fmin(min.y, tri_points[i].y);
        min.z = fmin(min.z, tri_points[i].z);
        min.x = fmax(max.x, tri_points[i].x);
        min.y = fmax(max.y, tri_points[i].y);
        min.z = fmax(max.z, tri_points[i].z);
    }
}

void AABB::merge(const AABB& other) {
    assert(this->is_valid());
    assert(other.is_valid());

    min.x = fmin(min.x, other.min.x);
    min.y = fmin(min.y, other.min.y);
    min.z = fmin(min.z, other.min.z);
    min.x = fmax(max.x, other.max.x);
    min.y = fmax(max.y, other.max.y);
    min.z = fmax(max.z, other.max.z);
}
} // namespace aabb