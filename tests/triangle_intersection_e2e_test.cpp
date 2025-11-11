#include <gtest/gtest.h>
#include <vector>

#include "geometry/triangle.hpp"
#include "geometry/vector_3d.hpp"

namespace {

size_t count_triangle_intersections(const std::vector<geometry::Triangle>& triangles) {
    size_t count = 0;
    for (size_t i = 0; i < triangles.size(); ++i) {
        for (size_t j = i + 1; j < triangles.size(); ++j) {
            if (triangles[i].intersection(triangles[j])) {
                ++count;
            }
        }
    }
    return count;
}

} // namespace

TEST(TriangleIntersectionE2E, CoplanarSceneCount) {
    using geometry::Triangle;
    using geometry::Vector3D;

    const Triangle base(
        Vector3D(0.0f, 0.0f, 0.0f),
        Vector3D(3.0f, 0.0f, 0.0f),
        Vector3D(0.0f, 3.0f, 0.0f)
    );

    const Triangle nested(
        Vector3D(0.5f, 0.5f, 0.0f),
        Vector3D(1.0f, 0.4f, 0.0f),
        Vector3D(0.5f, 1.2f, 0.0f)
    );

    const Triangle edge_touch(
        Vector3D(3.0f, 0.0f, 0.0f),
        Vector3D(4.0f, 0.0f, 0.0f),
        Vector3D(3.0f, 1.0f, 0.0f)
    );

    const Triangle disjoint(
        Vector3D(6.0f, 6.0f, 1.0f),
        Vector3D(7.0f, 6.0f, 1.0f),
        Vector3D(6.5f, 6.5f, 1.0f)
    );

    std::vector<Triangle> scene{base, nested, edge_touch, disjoint};
    EXPECT_EQ(count_triangle_intersections(scene), 2);
}

TEST(TriangleIntersectionE2E, Mixed3DSceneCount) {
    using geometry::Triangle;
    using geometry::Vector3D;

    const Triangle ground(
        Vector3D(0.0f, 0.0f, 0.0f),
        Vector3D(3.0f, 0.0f, 0.0f),
        Vector3D(0.0f, 3.0f, 0.0f)
    );

    const Triangle vertical_slice(
        Vector3D(1.0f, -1.0f, -1.0f),
        Vector3D(1.0f, 1.0f, 1.0f),
        Vector3D(1.0f, 1.0f, -1.0f)
    );

    const Triangle skew_slice(
        Vector3D(0.0f, 0.0f, -1.0f),
        Vector3D(2.0f, 0.0f, 1.0f),
        Vector3D(0.0f, 2.0f, 1.0f)
    );

    const Triangle point_touch(
        Vector3D(0.0f, 0.0f, 0.0f),
        Vector3D(-1.0f, 0.0f, -1.0f),
        Vector3D(0.0f, -1.0f, -1.0f)
    );

    const Triangle raised(
        Vector3D(0.0f, 0.0f, 2.0f),
        Vector3D(2.0f, 0.0f, 2.0f),
        Vector3D(0.0f, 2.0f, 2.0f)
    );

    std::vector<Triangle> scene{ground, vertical_slice, skew_slice, point_touch, raised};
    EXPECT_EQ(count_triangle_intersections(scene), 2);
}
