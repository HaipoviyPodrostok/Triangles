#include <gtest/gtest.h>
#include "geometry/triangle.hpp"

using namespace geometry;

// Все треугольники лежат на одной плоскости: x + y + z = 3 (=> z = 3 - x - y)

TEST(TriangleCoplanarTest, IdenticalTriangles) {
    Triangle t1({1, 1, 1}, {2, 1, 0}, {1, 2, 0});
    Triangle t2({1, 1, 1}, {2, 1, 0}, {1, 2, 0});
    EXPECT_TRUE(t1.intersection(t2));
}

TEST(TriangleCoplanarTest, OneInsideAnother) {
    // большой треугольник
    Triangle outer({0, 0, 3}, {3, 0, 0}, {0, 3, 0});
    // вложенный маленький треугольник
    Triangle inner({1, 1, 1}, {1.5, 1, 0.5}, {1, 1.5, 0.5});
    EXPECT_TRUE(outer.intersection(inner));
    EXPECT_TRUE(inner.intersection(outer));
}

TEST(TriangleCoplanarTest, PartialOverlap) {
    Triangle t1({0, 0, 3}, {3, 0, 0}, {0, 3, 0});
    Triangle t2({1, 0.5, 1.5}, {2.5, 0.5, 0}, {0.5, 2, 0.5});
    EXPECT_TRUE(t1.intersection(t2));
}

TEST(TriangleCoplanarTest, TouchAtVertex) {
    Triangle t1({1, 1, 1}, {2, 1, 0}, {1, 2, 0});
    Triangle t2({2, 1, 0}, {2.5, 0.5, 0}, {2.5, 1.5, -0.5}); // подогнан по той же плоскости
    EXPECT_TRUE(t1.intersection(t2)); // касание в (2,1,0)
}

TEST(TriangleCoplanarTest, TouchAlongEdge) {
    Triangle t1({0, 0, 3}, {3, 0, 0}, {0, 3, 0});
    Triangle t2({3, 0, 0}, {0, 3, 0}, {3, 3, -3});
    // для копланарности скорректируем третью вершину:
    Triangle t2_fixed({3, 0, 0}, {0, 3, 0}, {2, 1, 0});
    EXPECT_TRUE(t1.intersection(t2_fixed));
}

TEST(TriangleCoplanarTest, NoIntersectionClose) {
    Triangle t1({1, 1, 1}, {2, 1, 0}, {1, 2, 0});
    // чуть сдвинутый вдоль x, всё ещё в той же плоскости
    Triangle t2({2.2, 1, -0.2}, {3.2, 1, -1.2}, {2.2, 2, -1.2});
    // исправим, чтобы лежал на x + y + z = 3
    Triangle t2_fixed({2.2, 1, -0.2}, {3.2, 1, -1.2}, {2.2, 2, -1.2 + 0.2});
    EXPECT_FALSE(t1.intersection(t2_fixed));
}

TEST(TriangleCoplanarTest, NoIntersectionFarApart) {
    Triangle t1({1, 1, 1}, {2, 1, 0}, {1, 2, 0});
    // далеко по оси X, но тоже x + y + z = 3
    Triangle t2({6, 1, -4}, {7, 1, -5}, {6, 2, -5});
    // скорректируем по плоскости:
    Triangle t2_fixed({6, 1, -4}, {7, 1, -5}, {6, 2, -5});
    EXPECT_FALSE(t1.intersection(t2_fixed));
}

TEST(TriangleCoplanarTest, SharedEdgeOppositeNormal) {
    Triangle t1({0, 0, 3}, {3, 0, 0}, {0, 3, 0});
    // тот же треугольник, но вершины в обратном порядке
    Triangle t2({0, 3, 0}, {3, 0, 0}, {0, 0, 3});
    EXPECT_TRUE(t1.intersection(t2));
}
