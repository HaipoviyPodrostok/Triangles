#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <numeric>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "AABB.hpp"
#include "geometry/geometry.hpp"
#include "math/math.hpp"
#include "utils/logger.hpp"

namespace acceleration {

inline constexpr size_t max_leaf_cap_for_cpu = 4;
inline constexpr size_t tree_max_depth       = 64;
inline constexpr size_t morton_code_size     = 32;  // bits
inline constexpr size_t grid_resolution      = 1024;

#ifdef USE_OPENCL
inline const std::string opencl_file = "source/acceleration/kernels/lbvh.cl";
#endif  // USE_OPENCL

struct BVHNode {
  AABB box;

  int32_t left_idx  = -1;
  int32_t right_idx = -1;

  size_t start  = 0;
  size_t n_objs = 0;

  BVHNode(const AABB& box_ = AABB(), const size_t first_ = 0,
      const size_t n_objs_ = 0);

  [[nodiscard]] bool is_leaf() const { return (n_objs > 0); }
  [[nodiscard]] bool is_valid() const;

  void init_leaf(const AABB& box_, const size_t start_, const size_t n_objs_);
  void init_internal(
      const AABB& box_, const int left_idx_, const int right_idx_);
};

template <typename ObjT>
class BVHTree {
 public:
  BVHTree() = delete;
  explicit BVHTree(std::vector<ObjT>& input) : input(input) { build(); }

  size_t max_depth_reached;

  void dump_to_dot(const std::string& filename) const;
  void dump_node_dot(std::ostream& out, size_t idx) const;

  [[nodiscard]] std::vector<bool> get_intersections() const;
  [[nodiscard]] bool              validate_tree() const;

 private:
  std::vector<BVHNode>  nodes;
  std::vector<ObjT>&    input;
  std::vector<size_t>   indexes;
  std::vector<uint32_t> morton_codes;

  [[nodiscard]] std::vector<geometry::Vector3D> find_centroids() const;

  [[nodiscard]] AABB compute_box(const size_t start, const size_t n_objs) const;
  [[nodiscard]] size_t partition_by_median(
      const size_t start, const size_t n_objs);
  [[nodiscard]] bool validate_node_rec(const size_t node_idx,
      size_t& total_leaf_objects, std::vector<bool>& visited) const;

  void   build();
  void   build_cpu();
  void   sort_input_cpu(const size_t start, const size_t n_objs,
        const math::Axis wildest_axis, const size_t mid_idx);
  size_t build_node_rec_cpu(
      const size_t start, const size_t n_objs, const size_t depth);
  bool get_intersections_rec(const size_t node_idx, const size_t query,
      std::vector<bool>& ever_intersected, const AABB& query_box) const;

#ifdef USE_OPENCL
  void build_gpu();

  [[nodiscard]] AABB compute_global_box(
      const std::vector<geometry::Vector3D> centroids);

  void sort_input_gpu();
  void compute_boxes(const size_t node_idx, const size_t n_internals);
#endif
};

template <typename ObjT>
void BVHTree<ObjT>::build() {
  nodes.clear();
  indexes.resize(input.size());
  std::iota(indexes.begin(), indexes.end(), 0);

#ifdef USE_OPENCL
  try {
    build_gpu();
    LOG_INFO("GPU build sccessful");
    return;
  } catch (const std::exception& e) {
    LOG_WARN("GPU build failed ({}). Falling back to CPU.", e.what());
    nodes.clear();
    indexes.resize(input.size());
    std::iota(indexes.begin(), indexes.end(), 0);
  }
#endif  // USE_OPENCL

  build_cpu();
}

template <typename ObjT>
[[nodiscard]] std::vector<geometry::Vector3D> BVHTree<ObjT>::find_centroids()
    const {
  std::vector<geometry::Vector3D> centroids;

  for (size_t i = 0; i < input.size(); ++i) {
    assert(input[i].is_valid());
    const geometry::Vector3D centroid = input[i].get_centre();
    centroids.emplace_back(centroid);
  }
  return centroids;
}

#ifdef USE_OPENCL
namespace detail {

struct SplitInfo {
  std::vector<int32_t> splits;
  std::vector<int32_t> starts;
  std::vector<int32_t> n_objs;
};

[[nodiscard]] SplitInfo get_split_info(
    const std::vector<uint32_t>& morton_codes);

void fill_node_idx(std::vector<BVHNode>& nodes, const SplitInfo& split_info);

[[nodiscard]] std::vector<uint32_t> get_morton_code(
    const std::vector<geometry::Vector3D>& centroids,
    const acceleration::AABB&              box) noexcept;

}  // namespace detail

template <typename ObjT>
AABB BVHTree<ObjT>::compute_global_box(
    const std::vector<geometry::Vector3D> centroids) {
  if (centroids.empty()) { return AABB(); }

  geometry::Vector3D min = centroids[0];
  geometry::Vector3D max = centroids[0];

  for (size_t i = 1; i < centroids.size(); ++i) {
    for (size_t j = 0; j < 3; ++j) {
      const geometry::Vector3D& centroid = centroids[i];

      if (centroid[j] < min[j]) { min[j] = centroid[j]; }
      if (centroid[j] > max[j]) { max[j] = centroid[j]; }
    }
  }
  return {min, max};
}

template <typename ObjT>
void BVHTree<ObjT>::sort_input_gpu() {
  const size_t num_obj = input.size();
  if (num_obj == 0) return;

  std::vector<std::pair<uint32_t, size_t>> morton_pairs;
  morton_pairs.reserve(num_obj);

  for (size_t i = 0; i < num_obj; ++i) {
    morton_pairs.emplace_back(morton_codes[i], indexes[i]);
  }

  std::sort(morton_pairs.begin(), morton_pairs.end());

  std::vector<uint32_t> sorted_codes;
  sorted_codes.reserve(num_obj);

  for (size_t i = 0; i < num_obj; ++i) {
    sorted_codes.push_back(morton_pairs[i].first);
    indexes[i] = morton_pairs[i].second;
  }

  morton_codes = std::move(sorted_codes);
}

template <typename ObjT>
void BVHTree<ObjT>::compute_boxes(
    const size_t node_idx, const size_t n_internals) {
  if (node_idx >= n_internals) {
    const size_t obj_idx = nodes[node_idx].start;
    nodes[node_idx].box  = AABB{input[indexes[obj_idx]]};
    return;
  }

  const size_t left_idx  = nodes[node_idx].left_idx;
  const size_t right_idx = nodes[node_idx].right_idx;

  compute_boxes(left_idx, n_internals);
  compute_boxes(right_idx, n_internals);

  nodes[node_idx].box = merge(nodes[left_idx].box, nodes[right_idx].box);
}

template <typename ObjT>
void BVHTree<ObjT>::build_gpu() {
  const std::vector<geometry::Vector3D> centroids = find_centroids();
  const AABB global_box = compute_global_box(centroids);

  this->morton_codes = detail::get_morton_code(centroids, global_box);
  this->sort_input_gpu();

  detail::SplitInfo split_info = detail::get_split_info(this->morton_codes);

  detail::fill_node_idx(nodes, split_info);
  compute_boxes(0, split_info.splits.size());
}

#endif  // USE_OPENCL

template <typename ObjT>
void BVHTree<ObjT>::build_cpu() {
  build_node_rec_cpu(0, input.size(), 1);
}

template <typename ObjT>
AABB BVHTree<ObjT>::compute_box(const size_t start, const size_t n_objs) const {
  if (n_objs == 0) { return AABB(); }

  AABB box{input[indexes[start]]};

  for (size_t i = start + 1; i < start + n_objs; ++i) {
    box.merge(AABB{input[indexes[i]]});
  }
  return box;
}

template <typename ObjT>
size_t BVHTree<ObjT>::partition_by_median(
    const size_t start, const size_t n_objs) {
  const AABB& box = compute_box(start, n_objs);

  const double spread_x = std::fabs(box.max.x - box.min.x);
  const double spread_y = std::fabs(box.max.y - box.min.y);
  const double spread_z = std::fabs(box.max.z - box.min.z);

  math::Axis wildest_axis =
      (spread_x >= spread_y && spread_x >= spread_z)
          ? math::Axis::X
          : (spread_y >= spread_z ? math::Axis::Y : math::Axis::Z);

  const size_t mid_idx = start + n_objs / 2;

  sort_input_cpu(start, n_objs, wildest_axis, mid_idx);

  return mid_idx;
}

template <typename ObjT>
void BVHTree<ObjT>::sort_input_cpu(const size_t start, const size_t n_objs,
    const math::Axis wildest_axis, const size_t mid_idx) {
  using IdxIt = std::vector<size_t>::iterator;

  IdxIt begin = indexes.begin() + start;
  IdxIt mid   = indexes.begin() + mid_idx;
  IdxIt end   = indexes.begin() + start + n_objs;

  switch (wildest_axis) {
    case math::Axis::X: {
      auto centre_cmp = [this](size_t a, size_t b) {
        return input[a].get_centre().x < input[b].get_centre().x;
      };
      std::nth_element(begin, mid, end, centre_cmp);
      break;
    }
    case math::Axis::Y: {
      auto centre_cmp = [this](size_t a, size_t b) {
        return input[a].get_centre().y < input[b].get_centre().y;
      };
      std::nth_element(begin, mid, end, centre_cmp);
      break;
    }
    default: {
      auto centre_cmp = [this](size_t a, size_t b) {
        return input[a].get_centre().z < input[b].get_centre().z;
      };
      std::nth_element(begin, mid, end, centre_cmp);
      break;
    }
  }
}

template <typename ObjT>
size_t BVHTree<ObjT>::build_node_rec_cpu(
    const size_t start, const size_t n_objs, const size_t depth) {
  size_t node_idx = nodes.size();
  nodes.emplace_back();

  if (depth > max_depth_reached) { max_depth_reached = depth; }

  if (n_objs <= max_leaf_cap_for_cpu || depth >= tree_max_depth) {
    nodes[node_idx].init_leaf(compute_box(start, n_objs), start, n_objs);
    return node_idx;
  }

  const int mid_idx = partition_by_median(start, n_objs);

  const size_t left_n_objs  = mid_idx - start;
  const size_t right_n_objs = n_objs - left_n_objs;

  const int left_idx  = build_node_rec_cpu(start, left_n_objs, depth + 1);
  const int right_idx = build_node_rec_cpu(mid_idx, right_n_objs, depth + 1);

  nodes[node_idx].init_internal(
      merge(nodes[left_idx].box, nodes[right_idx].box), left_idx, right_idx);

  return node_idx;
}

template <typename ObjT>
bool BVHTree<ObjT>::validate_tree() const {
  if (nodes.empty()) return input.empty();
  const size_t root_idx           = 0;
  size_t       leaf_objects_count = 0;

  std::vector<bool> visited(nodes.size(), false);

  bool is_valid = validate_node_rec(root_idx, leaf_objects_count, visited);

  if (leaf_objects_count != input.size()) {
    LOG_ERR("Validation failed: Tree holds {}, but input has {}.",
        leaf_objects_count, input.size());
    return false;
  }

  return is_valid;
}

template <typename ObjT>
bool BVHTree<ObjT>::validate_node_rec(const size_t node_idx,
    size_t& total_leaf_objects, std::vector<bool>& visited) const {
  if (node_idx >= nodes.size()) {
    LOG_ERR("Validation failed: Index out of bounds.");
    return false;
  }

  if (visited[node_idx]) {
    LOG_ERR(
        "Validation failed: Cycle detected: Node {} points back to an "
        "already "
        "visited node.",
        node_idx);
    return false;
  }
  visited[node_idx] = true;

  const BVHNode& node = nodes[node_idx];

  if (!node.box.is_valid()) {
    LOG_ERR("Validation failed: Node {} gets invalid AABB.", node_idx);
    return false;
  }
  if (node.is_leaf()) {
    total_leaf_objects += node.n_objs;
    // node.box ==
    // compute_box(node.start, node.n_objs);
    return true;
  }

  if (node.left_idx == node_idx || node.right_idx == node_idx ||
      node.left_idx < 0 || node.right_idx < 0 ||
      node.left_idx >= nodes.size() || node.right_idx >= nodes.size()) {
    LOG_ERR(
        "Validation failed: Node {} has corrupted children indices ({}, {}).",
        node_idx, node.left_idx, node.right_idx);
    return false;
  }

  const BVHNode& left  = nodes[node.left_idx];
  const BVHNode& right = nodes[node.right_idx];

  AABB merged = merge(left.box, right.box);

  if (!node.box.is_contains(merged)) {
    LOG_ERR(
        "Validation failed: Node {}: AABB does not fully contain its "
        "children",
        node_idx);
    return false;
  }

  bool left_valid =
      validate_node_rec(node.left_idx, total_leaf_objects, visited);
  bool right_valid =
      validate_node_rec(node.right_idx, total_leaf_objects, visited);

  return left_valid && right_valid;
}

template <typename ObjT>
std::vector<bool> BVHTree<ObjT>::get_intersections() const {
  std::vector<bool> ever_intersected(input.size(), false);

  if (nodes.empty()) { return ever_intersected; }
  if (!validate_tree()) {
    throw std::runtime_error("Run get_intersections(): tree is invalid");
  }

  for (size_t query_idx = 0; query_idx < input.size(); ++query_idx) {
    if (ever_intersected[query_idx]) { continue; }

    AABB query_box{input[query_idx]};
    get_intersections_rec(0, query_idx, ever_intersected, query_box);
  }

  return ever_intersected;
}

template <typename ObjT>
bool BVHTree<ObjT>::get_intersections_rec(const size_t node_idx,
    const size_t query_idx, std::vector<bool>& ever_intersected,
    const AABB& query_box) const {
  const BVHNode& node     = nodes[node_idx];
  const AABB&    node_box = node.box;

  if (!query_box.is_intersect(node_box)) { return false; }

  if (node.is_leaf()) {
    for (size_t i = node.start; i < node.start + node.n_objs; ++i) {
      size_t real_id = indexes[i];
      if (real_id != query_idx &&
          input[query_idx].is_intersect(input[real_id])) {
        ever_intersected[query_idx] = true;
        ever_intersected[real_id]   = true;
        return true;
      }
    }
    return false;
  }
  if (get_intersections_rec(
          node.left_idx, query_idx, ever_intersected, query_box)) {
    return true;
  }
  if (get_intersections_rec(
          node.right_idx, query_idx, ever_intersected, query_box)) {
    return true;
  }

  return false;
}

template <typename ObjT>
void BVHTree<ObjT>::dump_to_dot(const std::string& filename) const {
  std::ofstream out(filename);
  out << "digraph BVH {\n";
  out << "  node [shape=record];\n";

  if (!nodes.empty()) { dump_node_dot(out, 0); }

  out << "}\n";
}

template <typename ObjT>
void BVHTree<ObjT>::dump_node_dot(std::ostream& out, size_t idx) const {
  const auto& node = nodes[idx];

  out << "  node" << idx << " [label=\"{ID: " << idx;
  if (node.is_leaf()) {
    out << " | LEAF | n_objs: " << node.n_objs << "}\"];\n";
  } else {
    out << " | INTERNAL}\"];\n";

    out << "  node" << idx << " -> node" << node.left_idx << ";\n";
    out << "  node" << idx << " -> node" << node.right_idx << ";\n";

    dump_node_dot(out, node.left_idx);
    dump_node_dot(out, node.right_idx);
  }
}

}  // namespace acceleration

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
// namespace acceleration
