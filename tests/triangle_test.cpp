#include <gtest/gtest.h>

#include "geometry/triangle.hpp"

using namespace geometry;

// Все треугольники лежат на одной плоскости: x + y + z = 3

TEST(TriangleCoplanarTest, IdenticalTriangles) {
    Triangle t1({1, 1, 1}, {2, 1, 0}, {1, 2, 0});
    Triangle t2({1, 1, 1}, {2, 1, 0}, {1, 2, 0});
    EXPECT_TRUE(t1.intersection(t2));
}

TEST(TriangleCoplanarTest, OneInsideAnother) {
    Triangle outer({1, 1, 1}, {3, 0, 0}, {0, 3, 0});
    Triangle inner({1.5, 1, 0.5}, {2, 0.5, 0.5}, {1, 1.5, 0.5});
    EXPECT_TRUE(outer.intersection(inner));
    EXPECT_TRUE(inner.intersection(outer));
}

TEST(TriangleCoplanarTest, PartialOverlap) {
    Triangle t1({1, 1, 1}, {3, 0, 0}, {0, 3, 0});
    Triangle t2({2, 0.5, 0.5}, {3.5, -0.5, 0}, {1.5, 2, 0.5});
    EXPECT_TRUE(t1.intersection(t2));
}

TEST(TriangleCoplanarTest, TouchAtVertex) {
    Triangle t1({1, 1, 1}, {2, 1, 0}, {1, 2, 0});
    Triangle t2({2, 1, 0}, {3, 1, -1}, {2, 2, -1});
    EXPECT_TRUE(t1.intersection(t2)); // общая вершина (2,1,0)
}

TEST(TriangleCoplanarTest, TouchAlongEdge) {
    Triangle t1({1, 1, 1}, {3, 0, 0}, {0, 3, 0});
    Triangle t2({3, 0, 0}, {0, 3, 0}, {3, 3, -3});
    EXPECT_TRUE(t1.intersection(t2)); // общая грань
}

TEST(TriangleCoplanarTest, NoIntersectionClose) {
    Triangle t1({1, 1, 1}, {2, 1, 0}, {1, 2, 0});
    Triangle t2({2.2, 1, -0.2}, {3.2, 1, -1.2}, {2.2, 2, -1.2});
    EXPECT_FALSE(t1.intersection(t2));
}

TEST(TriangleCoplanarTest, NoIntersectionFarApart) {
    Triangle t1({1, 1, 1}, {2, 1, 0}, {1, 2, 0});
    Triangle t2({6, 1, -4}, {7, 1, -5}, {6, 2, -5});
    EXPECT_FALSE(t1.intersection(t2));
}

TEST(TriangleCoplanarTest, SharedEdgeOppositeNormal) {
    Triangle t1({1, 1, 1}, {3, 0, 0}, {2, 1, 0});
    Triangle t2({1, 1, 1}, {3, 0, 0}, {2, 1, 2}); // отражён относительно плоскости
    EXPECT_TRUE(t1.intersection(t2));
}
