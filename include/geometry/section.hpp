#pragma once

#include <optional>

#include "line.hpp"
#include "vector_3d.hpp"

namespace geometry {

class Section {
 public:
  Vector3D a;
  Vector3D b;

  Section(const Vector3D& a, const Vector3D& b);

  [[nodiscard]] bool is_valid() const;
  [[nodiscard]] bool is_intersect(const Section& other) const;
  [[nodiscard]] bool is_intersect(const Line& other) const;
  [[nodiscard]] bool is_belong(const Line& line) const;
  [[nodiscard]] std::optional<Vector3D> intersect_point(
      const Line& other) const;
  [[nodiscard]] double length() const;
  [[nodiscard]] Line get_line() const;
  [[nodiscard]] bool is_contains(const Vector3D& p) const;

  void print() const;
};
}  // namespace geometry