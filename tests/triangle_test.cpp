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
// === ТРЕУГОЛЬНИКИ ПЕРЕСЕКАЮТСЯ ПО ОТРЕЗКУ ===
TEST(Triangle3DTest, IntersectBySegment) {
    // Первый треугольник в плоскости z = 0
    Triangle t1(
        Vector3D(0, 0, 0),
        Vector3D(2, 0, 0),
        Vector3D(0, 2, 0)
    );

    // Второй треугольник — вертикальный, в плоскости x = 1
    Triangle t2(
        Vector3D(1, -1, -1),
        Vector3D(1, 1, -1),
        Vector3D(1, 0, 1)
    );

    // Линия пересечения плоскостей: x = 1, z = 0
    // Оба треугольника пересекаются вдоль отрезка (1,0,0) — (1,1,0)
    EXPECT_TRUE(t1.intersection(t2));
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

// =========================================================
// === НАБОР ВСЕХ СЛУЧАЕВ ПЕРЕСЕЧЕНИЙ ТРЕУГОЛЬНИКОВ ========
// =========================================================

// 1. Полное совпадение
TEST(Triangle3DTest, IdenticalTriangles) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({0,0,0}, {1,0,0}, {0,1,0});
    EXPECT_TRUE(t1.intersection(t2));
}

// 2. Копланарные, но не пересекаются
TEST(Triangle3DTest, CoplanarSeparate) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({2,0,0}, {3,0,0}, {2,1,0});
    EXPECT_FALSE(t1.intersection(t2));
}

// 3. Копланарные, касаются в одной вершине
TEST(Triangle3DTest, CoplanarTouchAtVertex) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({1,0,0}, {2,0,0}, {1,1,0});
    EXPECT_TRUE(t1.intersection(t2));
}

// 4. Копланарные, имеют общую сторону
TEST(Triangle3DTest, CoplanarShareEdge) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({1,0,0}, {1,1,0}, {0,1,0});
    EXPECT_TRUE(t1.intersection(t2));
}

// 5. Пересекаются под углом (z=0 и x=1)
TEST(Triangle3DTest, IntersectAtLine2) {
    Triangle t1({0,0,0}, {2,0,0}, {0,2,0});          // плоскость z = 0
    Triangle t2({1,-1,-1}, {1,1,1}, {1,1,-1});       // плоскость x = 1
    EXPECT_TRUE(t1.intersection(t2)); // пересечение по линии x=1, z=0
}

// 6. Касание по одной точке (вершина–плоскость)
TEST(Triangle3DTest, TouchAtSingleVertex) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});          // z=0
    Triangle t2({0,0,0}, {0,1,-1}, {1,0,-1});        // пересекается только в (0,0,0)
    EXPECT_TRUE(t1.intersection(t2));
}

// 7. Касание ребро–ребро (одна точка)
TEST(Triangle3DTest, EdgeEdgeContact) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({1,0,0}, {1,1,1}, {1,-1,1});
    EXPECT_TRUE(t1.intersection(t2)); // только (1,0,0)
}

// 8. Один треугольник полностью лежит внутри другого (копланарно)
TEST(Triangle3DTest, OneInsideAnotherCoplanar) {
    Triangle t1({0,0,0}, {4,0,0}, {0,4,0});
    Triangle t2({1,1,0}, {2,1,0}, {1,2,0});
    EXPECT_TRUE(t1.intersection(t2));
}

// 9. Один треугольник над другим (параллельные плоскости)
TEST(Triangle3DTest, ParallelNoIntersection2) {
    Triangle t1({0,0,0}, {2,0,0}, {0,2,0});
    Triangle t2({0,0,1}, {2,0,1}, {0,2,1});
    EXPECT_FALSE(t1.intersection(t2));
}

// 10. Почти параллельные (угол ≈ 180°, пересечения нет)
TEST(Triangle3DTest, AlmostParallel) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({0,0,1e-5f}, {1,0,1e-5f}, {0,1,1e-5f});
    EXPECT_FALSE(t1.intersection(t2));
}

// 11. Пересекаются ребром другого (отрезок–отрезок)
TEST(Triangle3DTest, EdgeCrossing) {
    Triangle t1({0,0,0}, {2,0,0}, {0,2,0});
    Triangle t2({1,-1,-1}, {1,1,1}, {1,1,-1});
    EXPECT_TRUE(t1.intersection(t2)); // отрезок по линии x=1,z=0
}

// 14. Пересекаются по внутренней линии, не касаясь вершин
TEST(Triangle3DTest, InnerLineCross) {
    Triangle t1({0,0,0}, {3,0,0}, {0,3,0});
    Triangle t2({1,1,-1}, {1,1,1}, {1.5,0.5,0});
    EXPECT_TRUE(t1.intersection(t2));
}

// 16. Пересекаются одной вершиной, но вне сторон
TEST(Triangle3DTest, SingleVertexOnPlane) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({0,0,0}, {1,-1,-1}, {1,1,1});
    EXPECT_TRUE(t1.intersection(t2)); // в точке (0,0,0)
}

// 17. Полное пересечение по линии, треугольники не копланарны
TEST(Triangle3DTest, SkewPlanesLineIntersection) {
    Triangle t1({0,0,0}, {2,0,0}, {0,2,0});
    Triangle t2({0,0,1}, {2,2,-1}, {2,0,1});
    EXPECT_TRUE(t1.intersection(t2));
}

// 18. Треугольники скрещиваются (не копланарны, не пересекаются)
TEST(Triangle3DTest, SkewNoIntersection) {
    Triangle t1({0,0,0}, {2,0,0}, {0,2,0});
    Triangle t2({1,1,1}, {2,1,1}, {1,2,1});
    EXPECT_FALSE(t1.intersection(t2));
}

// 19. Один внутри другого, но в параллельной плоскости (без касания)
TEST(Triangle3DTest, InsideButParallel) {
    Triangle t1({0,0,0}, {4,0,0}, {0,4,0});
    Triangle t2({1,1,1}, {2,1,1}, {1,2,1});
    EXPECT_FALSE(t1.intersection(t2));
}

// 20. Общая сторона, но нормали противоположны (обратный порядок вершин)
TEST(Triangle3DTest, SharedEdgeOppositeNormal) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({1,0,0}, {0,1,0}, {1,1,0}); // обратный порядок
    EXPECT_TRUE(t1.intersection(t2));
}
