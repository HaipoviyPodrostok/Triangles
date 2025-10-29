#pragma once

#include "vector_3d.hpp"
#include "line.hpp"

namespace geometry {

class Section {
public:
    Vector3D a_;
    Vector3D b_;

    Section(const Vector3D& start, const Vector3D& end);

    bool is_valid() const;
    bool is_intersect(const Section& other) const;
    bool is_intersect(const Line& other) const;
    bool is_belong(const Line& line) const; 
    Vector3D intersect_point(const Line& other) const;
    float length() const;
    Line get_line() const;
    bool is_contains(const Vector3D& p) const;
    void print() const;
};
} // namespace geometry