#pragma once

#include "geometry/geometry.hpp"
#include "geometry/triangle.hpp"
#include "geometry/vector_3d.hpp"
#include <cassert>

namespace acceleration {

struct AABB {
    geometry::Vector3D min;
    geometry::Vector3D max;

    AABB(const geometry::Vector3D min_, const geometry::Vector3D max_);
    AABB(const geometry::Triangle tri);
    
    bool is_valid() const;
    bool is_intersect(const AABB& other) const;
    void expand(const geometry::Triangle& tri);
    void merge(const AABB& other);
};

} // namespace acceleration