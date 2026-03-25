#include "acceleration/bvh_tree.hpp"

namespace acceleration {

BVHNode::BVHNode(const AABB& box_, const size_t first_, const size_t n_objs_)
    : box(box_), start(first_), n_objs(n_objs_) {
  // double min_x = box.min.x;
  // double min_y = box.min.y;
  // double min_z = box.min.z;

  // double max_x = box.max.x;
  // double max_y = box.max.y;
  // double max_z = box.max.z;

  // spdlog::info("{}, {}, {}, {}, {}, {}", min_x, min_y, min_z, max_x, max_y,
  //              max_z);
  assert(box_.is_valid());
}

bool BVHNode::is_valid() const noexcept {
  if (!box.is_valid()) { return false; }

  if (is_leaf()) {
    return left_idx == -1 && right_idx == -1 && n_objs > 0 &&
           n_objs <= max_leaf_capacity;
  } else {
    return n_objs == 0 && left_idx > -1 && right_idx > -1 &&
           left_idx != right_idx;
  }
}

void BVHNode::init_leaf(const AABB& box_, const size_t start_,
                        const size_t n_objs_) {
  box = box_;
  start = start_;
  n_objs = n_objs_;
  left_idx = -1;
  right_idx = -1;
}

void BVHNode::init_internal(const AABB& box_, const int left_idx_,
                            const int right_idx_) {
  box = box_;
  start = 0;
  n_objs = 0;
  left_idx = left_idx_;
  right_idx = right_idx_;
}

}  // namespace acceleration