#include "geometry/triangle.hpp"
#include <gtest/gtest.h>
#include "geometry/triangle.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
using namespace geometry;
constexpr float eps = math::flt_tolerance;

// === Вспомогательные функции ===
Triangle make_triangle_xy() {
    // Простой треугольник в плоскости Z = 0
    return Triangle(Vector3D(0, 0, 0),
                    Vector3D(1.0f, 0, 0),
                    Vector3D(0, 1.0f, 0));
}

// === ТЕСТЫ ДЛЯ is_inside ===
TEST(TriangleIsInside, InsidePoint) {
    Triangle t = make_triangle_xy();
    EXPECT_TRUE(t.is_inside(Vector3D(0.25, 0.25, 0)));
}

TEST(TriangleIsInside, OnVertex) {
    Triangle t = make_triangle_xy();
    EXPECT_TRUE(t.is_inside(Vector3D(0, 0, 0)));
    EXPECT_TRUE(t.is_inside(Vector3D(1, 0, 0)));
    EXPECT_TRUE(t.is_inside(Vector3D(0, 1, 0)));
}

TEST(TriangleIsInside, OnEdge) {
    Triangle t = make_triangle_xy();
    EXPECT_TRUE(t.is_inside(Vector3D(0.5, 0, 0)));
    EXPECT_TRUE(t.is_inside(Vector3D(0.5, 0.5, 0)));
    EXPECT_TRUE(t.is_inside(Vector3D(0, 0.5, 0)));
}

TEST(TriangleIsInside, OutsidePoint) {
    Triangle t = make_triangle_xy();
    EXPECT_FALSE(t.is_inside(Vector3D(1.0, 1.0, 0)));
    EXPECT_FALSE(t.is_inside(Vector3D(-0.1, 0.1, 0)));
}

TEST(TriangleIsInside, AbovePlaneShouldFail) {
    Triangle t = make_triangle_xy();
    EXPECT_FALSE(t.is_inside(Vector3D(0.25, 0.25, 0.01)));
    EXPECT_FALSE(t.is_inside(Vector3D(0.25, 0.25, -0.01)));
}

// === ТЕСТЫ ДЛЯ intersection() ===
TEST(TriangleIntersection, CoplanarFullOverlap) {
    Triangle t1 = make_triangle_xy();
    Triangle t2(Vector3D(0.2, 0.0, 0),
                Vector3D(1.0f, 0.0, 0),
                Vector3D(0.2, 0.8, 0));
    EXPECT_TRUE(t1.intersection(t2));
}

TEST(TriangleIntersection, CoplanarPartialOverlap) {
    Triangle t1 = make_triangle_xy();
    Triangle t2(Vector3D(0.5, -0.2, 0),
                Vector3D(1.0, -0.2, 0),
                Vector3D(0.5, 0.5, 0));
    EXPECT_TRUE(t1.intersection(t2));
}

TEST(TriangleIntersection, CoplanarTouchAtVertex) {
    Triangle t1 = make_triangle_xy();
    Triangle t2(Vector3D(1.0, 0.0, 0),
                Vector3D(2.0, 0.0, 0),
                Vector3D(1.0, 1.0, 0));
    EXPECT_TRUE(t1.intersection(t2));
}

TEST(TriangleIntersection, CoplanarTouchAlongEdge) {
    Triangle t1 = make_triangle_xy();
    Triangle t2(Vector3D(0.0, 1.0, 0),
                Vector3D(1.0, 0.0, 0),
                Vector3D(1.0, 1.0, 0));
    EXPECT_TRUE(t1.intersection(t2));
}

TEST(TriangleIntersection, CoplanarDisjoint) {
    Triangle t1 = make_triangle_xy();
    Triangle t2(Vector3D(2.0, 0.0, 0),
                Vector3D(3.0, 0.0, 0),
                Vector3D(2.0, 1.0, 0));
    EXPECT_FALSE(t1.intersection(t2));
}

TEST(TriangleIntersection, NonCoplanarShouldFail) {
    Triangle t1 = make_triangle_xy();
    Triangle t2(Vector3D(0, 0, 1),
                Vector3D(1, 0, 1),
                Vector3D(0, 1, 1));
    EXPECT_FALSE(t1.intersection(t2));
}

TEST(SectionOppositeDirection, CollinearOppositeDirectionOverlap) {
    Section s1(Vector3D(0, 0, 0), Vector3D(2, 0, 0));     // слева направо
    Section s2(Vector3D(3, 0, 0), Vector3D(1, 0, 0));     // справа налево
    EXPECT_TRUE(s1.is_intersect(s2));  // перекрываются на [1,2]
    EXPECT_TRUE(s2.is_intersect(s1));  // направление не должно влиять
}

// === КАСАНИЕ концами, противоположные направления ===
TEST(SectionOppositeDirection, CollinearOppositeDirectionTouchAtEndpoint) {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Section s2(Vector3D(2, 0, 0), Vector3D(1, 0, 0));     // в обратную сторону
    EXPECT_TRUE(s1.is_intersect(s2));  // касание в точке (1,0)
    EXPECT_TRUE(s2.is_intersect(s1));  // симметрично
}

// === КОЛЛИНЕАРНЫЕ, но раздельные (не касаются) ===
TEST(SectionOppositeDirection, CollinearOppositeDirectionDisjoint) {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Section s2(Vector3D(2 + eps * 10, 0, 0), Vector3D(3, 0, 0)); // разрыв > eps
    EXPECT_FALSE(s1.is_intersect(s2));
    EXPECT_FALSE(s2.is_intersect(s1));
}

// === НЕ коллинеарные, но пересекаются под углом ===
TEST(SectionOppositeDirection, CrossingUnderAngle) {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 1, 0));
    Section s2(Vector3D(0, 1, 0), Vector3D(1, 0, 0));
    EXPECT_TRUE(s1.is_intersect(s2)); // крест-накрест
}

// === Почти касаются, но не пересекаются (с eps-зазором) ===
TEST(SectionOppositeDirection, AlmostTouchButOutsideEps) {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Section s2(Vector3D(1 + eps * 10, 0, 0), Vector3D(2 + eps * 10, 0, 0));
    EXPECT_FALSE(s1.is_intersect(s2)); // зазор больше eps
}

// === ПРОСТЫЕ НЕПЕРЕСЕКАЮЩИЕСЯ ===
TEST(Triangle3DTest, ParallelNoIntersection) {
    Triangle t1(Vector3D(0,0,0), Vector3D(1,0,0), Vector3D(0,1,0));
    Triangle t2(Vector3D(0,0,1), Vector3D(1,0,1), Vector3D(0,1,1)); // параллельный, выше
    EXPECT_FALSE(t1.intersection(t2));
}

// === КОПЛАНАРНЫЕ (пересечение по линии / вершине) ===
TEST(Triangle3DTest, CoplanarTouchAtEdge) {
    Triangle t1(Vector3D(0,0,0), Vector3D(1,0,0), Vector3D(0,1,0));
    Triangle t2(Vector3D(1,0,0), Vector3D(1,1,0), Vector3D(0,1,0));
    EXPECT_TRUE(t1.intersection(t2));
}

TEST(Triangle3DTest, CoplanarDisjoint) {
    Triangle t1(Vector3D(0,0,0), Vector3D(1,0,0), Vector3D(0,1,0));
    Triangle t2(Vector3D(2,0,0), Vector3D(3,0,0), Vector3D(2,1,0));
    EXPECT_FALSE(t1.intersection(t2));
}

// === НЕ КОПЛАНАРНЫЕ, ПЕРЕСЕКАЮТСЯ ПОД УГЛОМ ===
TEST(Triangle3DTest, IntersectAtLine) {
    // Один треугольник в плоскости z=0, второй наклонён
    Triangle t1(Vector3D(0,0,0), Vector3D(2,0,0), Vector3D(0,2,0));
    Triangle t2(Vector3D(1,-1,-1), Vector3D(1,1,1), Vector3D(1,1,-1));
    EXPECT_TRUE(t1.intersection(t2));
}

TEST(Triangle3DTest, IntersectAtPointOnly) {
    Triangle t1(Vector3D(0,0,0), Vector3D(2,0,0), Vector3D(0,2,0));
    Triangle t2(Vector3D(2,0,0), Vector3D(2,1,1), Vector3D(2,-1,1)); // пересекаются в точке (2,0,0)
    EXPECT_TRUE(t1.intersection(t2));
}

TEST(Triangle3DTest, DisjointDifferentPlanes) {
    Triangle t1(Vector3D(0,0,0), Vector3D(1,0,0), Vector3D(0,1,0));
    Triangle t2(Vector3D(0,0,2), Vector3D(1,0,2), Vector3D(0,1,2));
    EXPECT_FALSE(t1.intersection(t2));
}

// === СЛУЧАЙ "ОДИН ТРЕУГОЛЬНИК ПРОРЕЗАЕТ ДРУГОЙ" ===
TEST(Triangle3DTest, SliceThrough) {
    Triangle t1(Vector3D(0,0,0), Vector3D(3,0,0), Vector3D(0,3,0));
    Triangle t2(Vector3D(0,1,-1), Vector3D(3,1,1), Vector3D(1,1,-1));
    EXPECT_TRUE(t1.intersection(t2));
}

// === ОДИН ВНУТРИ ДРУГОГО (в разных плоскостях, но пересечение отсутствует) ===
TEST(Triangle3DTest, ContainedButDifferentPlanes) {
    Triangle t1(Vector3D(0,0,0), Vector3D(4,0,0), Vector3D(0,4,0));
    Triangle t2(Vector3D(1,1,1), Vector3D(2,1,1), Vector3D(1,2,1)); // полностью выше
    EXPECT_FALSE(t1.intersection(t2));
}

// === ОДИН ВНУТРИ ДРУГОГО (копланарно) ===
TEST(Triangle3DTest, CoplanarOneInsideAnother) {
    Triangle t1(Vector3D(0,0,0), Vector3D(4,0,0), Vector3D(0,4,0));
    Triangle t2(Vector3D(1,1,0), Vector3D(2,1,0), Vector3D(1,2,0));
    EXPECT_TRUE(t1.intersection(t2));
}