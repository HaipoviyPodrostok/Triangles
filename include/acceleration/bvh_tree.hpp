#pragma once
#include "acceleration.hpp"
#include "geometry/triangle.hpp"
#include "math/math_utils.hpp"
#include <cstddef>
#include <fstream>
#include <string>

namespace acceleration {

inline constexpr size_t max_leaf_capacity  = 4;
inline constexpr size_t tree_max_depth = 64; 

#ifdef ENABLE_BVH_DEBUG
inline const std::string default_dump_folder = "./dumps";
inline const std::string counter_file_name = "./dumps/counter_file.txt";
#endif // ENABLE_BVH_DEBUG

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

    void print_node() const;
};
    
class BVHTree {
public:
    size_t max_depth_reached = 0;

    BVHTree(std::vector<geometry::Triangle>& input);

    bool is_valid() const;
    int build_node(const size_t start, const size_t n_objs, size_t depth);
    AABB calculate_box(const size_t start, const size_t n_objs);
    
#ifdef ENABLE_BVH_DEBUG //TODO << переопределить
    bool is_dumping = false;

    BVHBVHTree(const std::vector<geometry::Triangle>& input,
               )//TODO не дописано
    void dump(const std::vector<geometry::Triangle>& input,
              std::string dump_folder = default_dump_folder);
#endif // ENENABLE_BVH_DEBUG

private:
    std::vector<BVHNode> nodes;
    std::vector<geometry::Triangle>& triangles;

    int root = -1;

    int partition_by_median(const size_t start, const size_t n_objs);
    void sort_triangles_by_centers(const size_t start, const size_t n_objs,
                               const math::Axis wildest_axis, const size_t mid_idx);

#ifdef ENABLE_BVH_DEBUG
    size_t dump_call_cnt_ = 0;
    std::ofstream dump_filename_;

    std::ofstream make_dump_file();
    void dump_recursion(const int node_idx);
#endif // ENABLE_BVH_DEBUG
};
} // namespace acceleration
