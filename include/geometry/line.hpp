#pragma once

#include "vector_3d.hpp"

namespace geometry {

// -- line -- (r = origin + t * dir)
class Line {
public:
    Vector3D origin;
    Vector3D dir;

    Line(const Vector3D& origin, const Vector3D& dir);

    bool is_valid() const;
    bool is_match(const Line& other) const;
    bool is_parallel(const Line& other) const;
    bool is_intersect(const Line& other) const;
    bool is_contains(const Vector3D& point) const;

    Vector3D intersect_point(const Line& other) const;
    void print() const;
};
} // namespace geometry