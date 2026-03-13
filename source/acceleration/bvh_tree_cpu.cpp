#include <algorithm>
#include <cassert>
#include <cstddef>

#include "acceleration/bvh_tree.hpp"

namespace acceleration {

namespace {

[[nodiscard]] uint32_t expand_bits(uint32_t v) noexcept {
  v &= 0x000003ff;

  v = (v | (v << 16)) & 0x030000ff;
  v = (v | (v << 8)) & 0x0300f00f;
  v = (v | (v << 4)) & 0x030c30c3;
  v = (v | (v << 2)) & 0x09249249;
  return v;
}

[[nodiscard]] std::vector<uint32_t> get_morton_code(
    const std::vector<geometry::Vector3D>& centroids,
    const acceleration::AABB& box) {
  const double x_gap = std::max(box.max.x - box.min.x, 1e-9);
  const double y_gap = std::max(box.max.y - box.min.y, 1e-9);
  const double z_gap = std::max(box.max.z - box.min.z, 1e-9);

  std::vector<uint32_t> morton_codes;

  for (size_t i = 0; i < centroids.size(); ++i) {
    const double norm_x =
        ((centroids[i].x - box.min.x) / x_gap) * grid_resolution;
    const double norm_y =
        ((centroids[i].y - box.min.y) / y_gap) * grid_resolution;
    const double norm_z =
        ((centroids[i].z - box.min.z) / z_gap) * grid_resolution;

    uint32_t ix = static_cast<uint32_t>(
        std::clamp(norm_x, 0.0, static_cast<double>(grid_resolution - 1)));
    uint32_t iy = static_cast<uint32_t>(
        std::clamp(norm_y, 0.0, static_cast<double>(grid_resolution - 1)));
    uint32_t iz = static_cast<uint32_t>(
        std::clamp(norm_z, 0.0, static_cast<double>(grid_resolution - 1)));

    const uint32_t morton_code =
        (expand_bits(ix) << 2) | (expand_bits(iy) << 1) | (expand_bits(iz));
    morton_codes.push_back(morton_code);
  }

  return morton_codes;
}
}  // namespace

}  // namespace acceleration