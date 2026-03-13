#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "AABB.hpp"

namespace acceleration {

inline constexpr size_t max_leaf_capacity = 4;
inline constexpr size_t tree_max_depth = 64;
inline constexpr size_t morton_code_size = 32;  // bits
inline constexpr size_t grid_resolution = 1024;

#ifdef USE_OPENCL
inline const std::string opencl_file = "source/acceleration/kernels/lbvh.cl";
#endif  // USE_OPENCL

#ifdef ENABLE_BVH_DEBUG
inline const std::string default_dump_folder = "./dumps";
inline const std::string counter_file_name = "./dumps/counter_file.txt";
#endif  // ENABLE_BVH_DEBUG

struct BVHNode {
  AABB box;

  int32_t left_idx = -1;
  int32_t right_idx = -1;

  size_t start = 0;
  size_t n_objs = 0;

  BVHNode(const AABB& box_ = AABB(), const size_t first_ = 0,
          const size_t n_objs_ = 0);

  [[nodiscard]] bool is_leaf() const noexcept { return (n_objs > 0); }
  [[nodiscard]] bool is_valid() const noexcept;

  void init_leaf(const AABB& box_, const size_t start_, const size_t n_objs_);
  void init_internal(const AABB& box_, const int left_idx_,
                     const int right_idx_);
};

template <typename PolT>
class BVHTree {
 public:
  explicit BVHTree(std::vector<PolT>& input) : input(input) {
    const std::vector<geometry::Vector3D> centroids = find_centroids();
    morton_codes = get_morton_code(centroids, compute_global_box(centroids));
  }

  void build() noexcept;

  [[nodiscard]] AABB compute_global_box(
      const std::vector<geometry::Vector3D> centroids) const noexcept;

  [[nodiscard]] std::vector<geometry::Vector3D> find_centroids(
      const std::vector<PolT>& input) const noexcept;

  void sort_input() noexcept;

 private:
  std::vector<BVHNode> nodes;
  std::vector<PolT>& input;
  std::vector<uint32_t> morton_codes;

  void build_gpu() noexcept;
};

template <typename PolT>
std::vector<geometry::Vector3D> BVHTree<PolT>::find_centroids(
    const std::vector<PolT>& input) const noexcept {
  std::vector<geometry::Vector3D> centroids;

  for (size_t i = 0; i < input.size(); ++i) {
    assert(input[i].is_valid());
    const geometry::Vector3D centroid = input[i].get_centre();
    centroids.emplace_back(centroid);
  }
  return centroids;
}

template <typename PolT>
[[nodiscard]] AABB BVHTree<PolT>::compute_global_box(
    const std::vector<geometry::Vector3D> centroids) const noexcept {
  if (centroids.empty()) { return AABB(); }

  geometry::Vector3D min = input[0];
  geometry::Vector3D max = input[0];

  for (size_t i = 1; i < input.size(); ++i) {
    for (size_t j = 0; j < 3; ++j) {
      const geometry::Vector3D& centroid = centroids[i];

      if (centroid[j] < min[j]) { min[j] = centroid[j]; }
      if (centroid[j] > max[j]) { max[j] = centroid[j]; }
    }
  }
  return {min, max};
}

template <typename PolT>
void BVHTree<PolT>::sort_input() noexcept {
  const size_t num_obj = input.size();
  if (num_obj == 0) return;

  std::vector<std::pair<uint32_t, uint32_t>> morton_pairs;
  morton_pairs.reserve(num_obj);

  for (size_t i = 0; i < input.size(); ++i) {
    morton_pairs.emplace_back(morton_codes[i], i);
  }

  std::sort(morton_pairs.begin(), morton_pairs.end());

  std::vector<PolT> sorted_input;
  sorted_input.reserve(num_obj);
  std::vector<uint32_t> sorted_codes;
  sorted_codes.reserve(num_obj);

  for (size_t i = 0; i < num_obj; ++i) {
    const uint32_t original_index = morton_pairs[i].second;

    sorted_codes.push_back(morton_pairs[i].first);
    sorted_input.push_back(std::move(input[original_index]));
  }

  morton_codes = std::move(sorted_codes);
  input = std::move(sorted_input);
}
}

// class BVHTree {
// //  public:
// //   size_t max_depth_reached = 0;

//   BVHTree(std::vector<geometry::Triangle>& input);

//   bool is_valid() const;
//   int build_node(const size_t start, const size_t n_objs, size_t depth);
//   AABB calculate_box(const size_t start, const size_t n_objs);

// #ifdef ENABLE_BVH_DEBUG  // TODO << переопределить
//   bool is_dumping = false;

//   // BVHBVHTree(const std::vector<geometry::Triangle>& input,
//   //              )//TODO не дописано
//   //   void dump(const std::vector<geometry::Triangle>& input,
//   //             std::string dump_folder = default_dump_folder);
// #endif  // ENENABLE_BVH_DEBUG

//  private:
//   std::vector<BVHNode> nodes;
//   std::vector<geometry::Triangle>& triangles;

//   int root = -1;

//   int partition_by_median(const size_t start, const size_t n_objs);
//   void sort_triangles_by_centers(const size_t start, const size_t n_objs,
//                                  const math::Axis wildest_axis,
//                                  const size_t mid_idx);

// #ifdef ENABLE_BVH_DEBUG
//   size_t dump_call_cnt_ = 0;
//   std::ofstream dump_filename_;

//   std::ofstream make_dump_file();
//   void dump_recursion(const int node_idx);
// #endif  // ENABLE_BVH_DEBUG
// };
}  // namespace acceleration
