#pragma once

#include <math.h>

#include <cassert>

#include "line.hpp"
#include "plane.hpp"
#include "section.hpp"
#include "vector_3d.hpp"

namespace geometry {

class Triangle {
 public:
  Vector3D a;
  Vector3D b;
  Vector3D c;

  Triangle(const Vector3D& a, const Vector3D& b, const Vector3D& c);

  [[nodiscard]] Plane get_plane() const;
  [[nodiscard]] Line get_intersect_line(const Triangle& other) const;
  void print() const;
  [[nodiscard]] Vector3D get_centre() const;

  [[nodiscard]] bool is_valid() const;
  [[nodiscard]] bool is_intersect(const Section& sec) const;
  [[nodiscard]] bool is_intersect(const Triangle& other) const;
  [[nodiscard]] bool is_point() const;
  [[nodiscard]] bool is_section() const;
  [[nodiscard]] bool is_inside(const Vector3D& p) const;

 private:
  [[nodiscard]] bool is_intersect_2d(const Triangle& other) const;
  [[nodiscard]] bool is_intersect_3d(const Triangle& other) const;
};

}  // namespace geometry