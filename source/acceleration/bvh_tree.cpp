#include "acceleration/bvh_tree.hpp"
#include "acceleration/acceleration.hpp"
#include "math/math_utils.hpp"
#include <cassert>
#include <cstddef>
#include <stdexcept>

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
}

void BVHTree::partition(const BVHNode& node) {  
    const float spread_x = fabs(node.box.max.x - node.box.min.x);
    const float spread_y = fabs(node.box.max.y - node.box.min.y);
    const float spread_z = fabs(node.box.max.z - node.box.min.z);

    math::Axis wildest_axis = (spread_x >= spread_y && spread_x >= spread_z) ? math::Axis::X :
                (spread_y >= spread_z ? math::Axis::Y : math::Axis::Z);

    switch (wildest_axis) {
        case 0:
            
    
    }
}



} // namespace acceleration