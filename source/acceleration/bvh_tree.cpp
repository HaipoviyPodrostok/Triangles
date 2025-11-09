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
    : box(box_), first(first_), n_objs(n_objs_) { 
        assert(box_.is_valid());
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
    partition(0);
}

void BVHTree::partition(const int node_idx) { 
    BVHNode node = nodes[node_idx];

    if (node.n_objs < max_leaf_capacity) {
        node.is_leaf = true;
        return;
    }
    
    const float spread_x = fabs(nodes[node_idx].box.max.x - nodes[node_idx].box.min.x);
    const float spread_y = fabs(nodes[node_idx].box.max.y - nodes[node_idx].box.min.y);
    const float spread_z = fabs(nodes[node_idx].box.max.z - nodes[node_idx].box.min.z);

    math::Axis wildest_axis = (spread_x >= spread_y && spread_x >= spread_z) ? math::Axis::X :
                (spread_y >= spread_z ? math::Axis::Y : math::Axis::Z);
               
    size_t mid_idx = node.first + node.n_objs / 2;
    
    sort_triangles_by_centroids(node, wildest_axis, mid_idx);

    AABB left_box{triangles[node.first]};
    AABB right_box{triangles[mid_idx]};
    
    size_t i = node.first;

    for (;i < mid_idx; ++i) {
        left_box.expand(triangles[i]);
    }

    size_t left_n_objs = mid_idx; //mid_idx -> right box

    for (; i < node.first + node.n_objs; ++i) {
        right_box.expand(triangles[i]);
    }

    size_t right_n_objs = node.n_objs - left_n_objs;

    BVHNode left_node{left_box, node.first, left_n_objs};
    BVHNode right_node{right_box, mid_idx, right_n_objs};

    node.left = nodes.size();
    nodes.emplace_back(left_node);
    partition(node.first);       //TODO нормальные имена

    node.right = nodes.size();
    nodes.emplace_back(right_node);
    partition(mid_idx);
}

void BVHTree::sort_triangles_by_centroids(const BVHNode& node, const math::Axis wildest_axis, const size_t mid_idx) {
    switch(wildest_axis) {
        case math::Axis::X: {
            auto centroid_cmp = [](const geometry::Triangle& a, const geometry::Triangle& b) {
                return  a.centroid.x < b.centroid.x;
            };
            std::nth_element(node.first, mid_idx, node.first + node.n_objs, centroid_cmp);
        }
        case math::Axis::Y: {
            auto centroid_cmp = [](const geometry::Triangle& a, const geometry::Triangle& b) {
                return  a.centroid.y < b.centroid.y;
            };
            std::nth_element(node.first, mid_idx, node.first + node.n_objs, centroid_cmp);
        }
        default: {
            auto centroid_cmp = [](const geometry::Triangle& a, const geometry::Triangle& b) {
                return  a.centroid.z < b.centroid.z;
            };
            std::nth_element(node.first, mid_idx, node.first + node.n_objs, centroid_cmp);
        }
    }
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
