#include "acceleration/bvh_tree.hpp"
#include "acceleration/acceleration.hpp"
#include "geometry/triangle.hpp"
#include "math/math_utils.hpp"
#include <cassert>
#include <cstddef>
#include <stdexcept>
#include <algorithm>

namespace acceleration {

BVHNode::BVHNode(const AABB& input_box) 
    : box(input_box) { 
        assert(input_box.is_valid());
    }

BVHTree::BVHTree(const std::vector<geometry::Triangle>& input) 
    : triangles(input) {

    if (input.size() < 2) {
        throw std::logic_error("There must be at least 2 triangles");
    }

    AABB first_tri_box{triangles[0]};

    for (size_t i = 1; i < input.size(); ++i) {
        first_tri_box.expand(triangles[i]);
    }

    nodes.emplace_back(first_tri_box);
    partition(nodes[0], triangles.begin(), triangles.end());
}

void BVHTree::partition(const BVHNode& node, TrianglesIt start, TrianglesIt end) {
    const float spread_x = fabs(node.box.max.x - node.box.min.x);
    const float spread_y = fabs(node.box.max.y - node.box.min.y);
    const float spread_z = fabs(node.box.max.z - node.box.min.z);

    math::Axis wildest_axis = (spread_x >= spread_y && spread_x >= spread_z) ? math::Axis::X :
                (spread_y >= spread_z ? math::Axis::Y : math::Axis::Z);

    switch(wildest_axis) {
        case math::Axis::X: {
            const float split_x = (nodes[0].box.max.x + nodes[0].box.min.x) / 2;
            
            geometry::Vector3D left_min = nodes[0].box.min;
            geometry::Vector3D left_max{split_x, nodes[0].box.max.y, nodes[0].box.max.z};

            geometry::Vector3D right_min{split_x, nodes[0].box.min.y, nodes[0].box.min.z};
            geometry::Vector3D right_max = nodes[0].box.max;

            break;
        }
        case math::Axis::Y: {
            const float split_y = (nodes[0].box.max.y + nodes[0].box.min.y) / 2;
            
            geometry::Vector3D left_min = nodes[0].box.min;
            geometry::Vector3D left_max{nodes[0].box.max.x, split_y, nodes[0].box.max.z};

            geometry::Vector3D right_min{nodes[0].box.min.x, split_y, nodes[0].box.min.z};
            geometry::Vector3D right_max = nodes[0].box.max;

            break;
        }
        default: {
            const float split_z = (nodes[0].box.max.z + nodes[0].box.min.z) / 2;
            
            geometry::Vector3D left_min = nodes[0].box.min;
            geometry::Vector3D left_max{nodes[0].box.max.x, nodes[0].box.max.y, split_z};

            geometry::Vector3D right_min{nodes[0].box.min.x, nodes[0].box.min.y, split_z};
            geometry::Vector3D right_max = nodes[0].box.max;

            break;
        }
    }

    


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

    
    
}



} // namespace acceleration