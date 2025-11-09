#pragma once
#include "acceleration/AABB.hpp"
#include "geometry/triangle.hpp"
#include "math/math_utils.hpp"
#include <cstddef>

namespace acceleration {

inline constexpr size_t max_leaf_capacity  = 4;
inline constexpr size_t tree_max_depth = 64; 

struct BVHNode {
    AABB box;
   
    int left_idx  = -1;
    int right_idx = -1;
   
    size_t start  = 0;
    size_t n_objs = 0;
   
    bool is_leaf = false;

    void init_leaf(const AABB& box_, const size_t start_, const size_t n_objs_);
    void init_internal(const AABB& box_, const int left_idx_, const int right_idx_);

    BVHNode(const AABB& box_, const size_t first_, const size_t n_objs_);
};
    
struct BVHTree {
    std::vector<BVHNode> nodes;
    std::vector<geometry::Triangle> triangles;

    int root = -1;
    size_t max_depth_reached = 0;

    BVHTree(const std::vector<geometry::Triangle>& input);

    AABB calculate_box(const size_t start, const size_t n_objs);
    void partition_by_median(const int idx);
    void sort_triangles_by_centers(const BVHNode& node, const math::Axis wildest_axis, 
                                     const size_t mid_idx);
    int build_node(const size_t start, const size_t n_objs, size_t depth);

};
} // namespace acceleration
