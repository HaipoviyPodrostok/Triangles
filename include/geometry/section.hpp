#pragma once

#include "vector_3d.hpp"
#include "line.hpp"

namespace geometry {

enum Side { LEFT_SIDE, INTER_SIDE, RIGHT_SIDE };

class Section {
public:
    Section(const Vector3D& start, const Vector3D& end);

    bool is_valid() const;
    bool is_intersect(const Section& other) const;
    float length() const;
    Line get_line() const;
    Side get_side(const Vector3D& p) const;
    bool is_contains(const Vector3D& p) const;
    void print() const;

private:
    Vector3D a_;
    Vector3D b_;
};
} // namespace geometry