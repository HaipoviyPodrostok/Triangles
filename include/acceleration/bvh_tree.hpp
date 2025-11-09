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
   
    int left  = -1;
    int right = -1;
   
    size_t first  = 0;
    size_t n_objs = 0;
   
    bool is_leaf = false;

    BVHNode(const AABB& box_, const size_t first_, const size_t n_objs_);
};
    
struct BVHTree {
    std::vector<BVHNode> nodes;
    std::vector<geometry::Triangle> triangles;

    int root = -1;

    BVHTree(const std::vector<geometry::Triangle>& input);

    void partition(const int idx);
    void sort_triangles_by_centroids(const BVHNode& node, const math::Axis wildest_axis, 
                                     const size_t mid_idx);

};
} // namespace acceleration
