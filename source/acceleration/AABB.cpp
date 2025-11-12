#include <cassert>
#include <cmath>
#include <cstddef>

#include "geometry/triangle.hpp"
#include "geometry/vector_3d.hpp"
#include "geometry/geometry.hpp"
#include "math/math_utils.hpp"
#include "acceleration/AABB.hpp"


namespace acceleration {

AABB::AABB(const geometry::Vector3D min_, const geometry::Vector3D max_)
    : min(min_), max(max_) { }

AABB::AABB(const geometry::Triangle tri)
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
        for (size_t j = 0; j < 3; ++j) {
            min[j] = fmin(min[j], tri_points[i][j]);
            max[j] = fmax(max[j], tri_points[i][j]);
        }        
    }
}

void AABB::merge(const AABB& other) {
    assert(this->is_valid());
    assert(other.is_valid());

    for (size_t i = 0; i < 3; ++i) {
        min[i] = fmin(min[i], other.min[i]);
        max[i] = fmax(max[i], other.max[i]);
    }
}

AABB merge(AABB& a, AABB& b) {
    assert(a.is_valid() && b.is_valid());
    a.merge(b);
    return a;
}

bool AABB::is_inside(const AABB& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    
    return min.x < other.min.x + math::eps &&
           min.y < other.min.y + math::eps &&
           min.z < other.min.z + math::eps &&
           max.x < other.max.x + math::eps &&
           max.y < other.max.y + math::eps &&    
           max.z < other.max.z + math::eps;
}

} // namespace acceleration