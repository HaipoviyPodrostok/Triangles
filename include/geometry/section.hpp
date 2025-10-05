#pragma once

#include "geometry/vector_3d.hpp"
#include "geometry/line.hpp"

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

private:
    Vector3D a_;
    Vector3D b_;
};
}