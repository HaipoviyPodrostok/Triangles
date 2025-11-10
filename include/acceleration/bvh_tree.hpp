#pragma once
#include "acceleration/AABB.hpp"
#include "geometry/triangle.hpp"
#include "math/math_utils.hpp"
#include <cstddef>
#include <string>

namespace acceleration {

inline constexpr size_t max_leaf_capacity  = 4;
inline constexpr size_t tree_max_depth = 64; 
inline const std::string dump_folder_name = "./dumps";
inline const std::string counter_file_name = "./dumps/counter_file.txt"

struct BVHNode {
    AABB box;
   
    int left_idx  = -1;
    int right_idx = -1;
   
    size_t start  = 0;
    size_t n_objs = 0;
   
    bool is_leaf = false;

    BVHNode(const AABB& box_ = AABB(), const size_t first_ = 0, const size_t n_objs_ = 0);

    bool is_valid() const;
    void init_leaf(const AABB& box_, const size_t start_, const size_t n_objs_);
    void init_internal(const AABB& box_, const int left_idx_, const int right_idx_);

};
    
class BVHTree {
public:
    std::vector<BVHNode> nodes;
    std::vector<geometry::Triangle> triangles;

    int root = -1;
    size_t max_depth_reached = 0;

    BVHTree(const std::vector<geometry::Triangle>& input);

    bool is_valid() const;
    int build_node(const size_t start, const size_t n_objs, size_t depth);
    AABB calculate_box(const size_t start, const size_t n_objs);
    
    void dump();

private:
    size_t dump_call_cnt = 0;
    
    int partition_by_median(const size_t start, const size_t n_objs);
    void sort_triangles_by_centers(const size_t start, const size_t n_objs,
                                   const math::Axis wildest_axis, const size_t mid_idx);

    std::string make_dump_file_name();
};
} // namespace acceleration
