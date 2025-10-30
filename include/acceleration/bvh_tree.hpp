#pragma once
#include "acceleration/AABB.hpp"
#include "geometry/triangle.hpp"

namespace acceleration {

struct BVHNode {
    AABB box;
    int left   = -1;
    int right  = -1;
    int tri_cnt = 0;

    BVHNode(const AABB& input_box);
};
    
struct BVHTree {
    std::vector<BVHNode> nodes;
    std::vector<geometry::Triangle> triangles;

    int root = -1;

    BVHTree(const std::vector<geometry::Triangle>& input);

    using TrianglesIt = typename std::vector<geometry::Triangle>::iterator;

    void partition(const BVHNode& node, TrianglesIt start, TrianglesIt end);
};
} // namespace acceleration
