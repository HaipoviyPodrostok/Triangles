#include "acceleration/bvh_tree.hpp"
#include "acceleration/acceleration.hpp"
#include "geometry/triangle.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <stdexcept>

namespace acceleration {

BVHNode::BVHNode(const AABB& box_, const size_t first_, const size_t n_objs_)
    : box(box_), start(first_), n_objs(n_objs_) { 
        assert(box_.is_valid());
}

void BVHNode::init_leaf(const AABB& box_, const size_t start_, const size_t n_objs_) {
    box = box_; start = start_; n_objs = n_objs_;
    left_idx = -1; right_idx = -1;
}

void BVHNode::init_internal(const AABB& box_, const int left_idx_, const int right_idx_) {
    box = box_; start = 0; n_objs = 0;
    left_idx  = left_idx_;
    right_idx = right_idx_;
}

BVHTree::BVHTree(const std::vector<geometry::Triangle>& input) 
    : triangles(input) {

    if (input.size() < 2) {
        throw std::logic_error("There must be at least 2 triangles");
    }

    AABB world_box{triangles[0]};

    for (size_t i = 1; i < input.size(); ++i) {
        world_box.expand(triangles[i]);
    }

    BVHNode root_node{world_box, 0, triangles.size()};
    
    nodes.emplace_back(root_node);
    partition_by_median(0);
}

AABB BVHTree::calculate_box(const size_t start, const size_t n_objs) {
    AABB box{triangles[start]};
    for (size_t i = start + 1; i < start + n_objs; ++i) {
        box.expand(triangles[i]);
    }
    return box;
}

void BVHTree::partition_by_median(const int node_idx) { 
    BVHNode& node = nodes[node_idx];

    if (node.n_objs <= max_leaf_capacity) {
        node.is_leaf = true;
        return;
    }
    
    const float spread_x = std::fabs(nodes[node_idx].box.max.x - nodes[node_idx].box.min.x);
    const float spread_y = std::fabs(nodes[node_idx].box.max.y - nodes[node_idx].box.min.y);
    const float spread_z = std::fabs(nodes[node_idx].box.max.z - nodes[node_idx].box.min.z);

    math::Axis wildest_axis = (spread_x >= spread_y && spread_x >= spread_z) ? math::Axis::X :
                (spread_y >= spread_z ? math::Axis::Y : math::Axis::Z);
               
    const size_t mid_idx = node.start + node.n_objs / 2;
    
    sort_triangles_by_centers(node, wildest_axis, mid_idx);

    AABB left_box{triangles[node.start]};
    for (size_t i = node.start + 1; i < mid_idx; ++i) {
        left_box.expand(triangles[i]);
    }
    
    AABB right_box{triangles[mid_idx]};  
    for (size_t i = mid_idx + 1; i < node.start + node.n_objs; ++i) {
        right_box.expand(triangles[i]);
    }
    
    const size_t left_n_objs = mid_idx - node.start; //mid_idx -> right box
    const size_t right_n_objs = node.n_objs - left_n_objs;

    if (left_n_objs == 0 || right_n_objs == 0) {
        node.is_leaf = true;
        return;
    }

    const int left_child_idx = nodes.size();
    nodes.emplace_back(BVHNode{left_box, node.start, left_n_objs});
    
    const int right_child_idx = nodes.size();
    nodes.emplace_back(BVHNode{right_box, mid_idx, right_n_objs});
    
    node.left_idx = left_child_idx;
    node.right_idx = right_child_idx;
    
    partition_by_median(left_child_idx);
    partition_by_median(right_child_idx);
}

void BVHTree::sort_triangles_by_centers(const BVHNode& node, const math::Axis wildest_axis, const size_t mid_idx) {
    switch(wildest_axis) {
        case math::Axis::X: {
            auto centre_cmp = [](const geometry::Triangle& a, const geometry::Triangle& b) {
                return  a.centroid.x < b.centroid.x;
            };
            std::nth_element(node.start, mid_idx, node.start + node.n_objs, centre_cmp);
        }
        case math::Axis::Y: {
            auto centre_cmp = [](const geometry::Triangle& a, const geometry::Triangle& b) {
                return  a.centroid.y < b.centroid.y;
            };
            std::nth_element(node.start, mid_idx, node.start + node.n_objs, centre_cmp);
        }
        default: {
            auto centre_cmp = [](const geometry::Triangle& a, const geometry::Triangle& b) {
                return  a.centroid.z < b.centroid.z;
            };
            std::nth_element(node.start, mid_idx, node.start + node.n_objs, centre_cmp);
        }
    }
}

int BVHTree::build_node(const size_t start, const size_t n_objs, size_t depth) {
    int node_idx = nodes.size();
    nodes.emplace_back();

    if (depth > max_depth_reached) {
        max_depth_reached = depth;
    }

    if (n_objs <= max_leaf_capacity || depth >= max_depth_reached) {
        nodes[node_idx].init_leaf(calculate_box(start, n_objs), start, n_objs);
        return node_idx;
    }

    int mid_idx = 

}
} // namespace acceleration
    
    // geometry::Vector3D node_box_min = nodes[idx].box.min;
    // geometry::Vector3D node_box_max = nodes[idx].box.max;
    
    // geometry::Vector3D left_min  = node_box_min;
    // geometry::Vector3D right_max = node_box_max;
    
    // geometry::Vector3D left_max;
    // geometry::Vector3D right_min;


    // switch(wildest_axis) {
    //     case math::Axis::X: {
    //         const float split_x = (nodes[idx].box.max.x + nodes[idx].box.min.x) * 0.5f;  
    //         left_max  = geometry::Vector3D{ split_x, node_box_max.y, node_box_max.z };
    //         right_min = geometry::Vector3D{ split_x, node_box_min.y, node_box_min.z };
    //         break;
    //     }
    //     case math::Axis::Y: {
    //         const float split_y = (nodes[idx].box.max.y + nodes[idx].box.min.y) * 0.5f;
    //         left_max  = geometry::Vector3D{ node_box_max.x, split_y, node_box_max.z };
    //         right_min = geometry::Vector3D{ node_box_min.x, split_y, node_box_min.z };
    //         break;
    //     }
    //     default: {
    //         const float split_z = (nodes[idx].box.max.z + nodes[idx].box.min.z) * 0.5f;    
    //         left_max  = geometry::Vector3D{ node_box_max.x, node_box_max.y, split_z };
    //         right_min = geometry::Vector3D{ node_box_min.x, node_box_min.y, split_z };
    //         break;
    //     }
    // }

    
    // AABB left{left_min, left_max};
    // node.left = nodes.size();
    // nodes.emplace_back(left);

    // AABB right{right_min, right_max};
    // node.right = nodes.size();
    // nodes.emplace_back(right);

                // std::sort(start, end,
    //           [wildest_axis](const geometry::Triangle& a, const geometry::Triangle& b) {
    //             switch (wildest_axis) {
    //                 case math::Axis::X:
    //                     return a.get_centre().x < b.get_centre().x;
    //                 case math::Axis::Y:
    //                     return a.get_centre().y < b.get_centre().y;
    //                 default:
    //                     return a.get_centre().z < b.get_centre().z;
    //             }
    //           });
