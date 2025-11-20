#include "geometry/triangle.hpp"
#include <gtest/gtest.h>
#include "geometry/triangle.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include "geometry/geometry.hpp"
using namespace geometry;

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
    EXPECT_TRUE(t1.is_intersect(t2));
}

TEST(TriangleIntersection, CoplanarPartialOverlap) {
    Triangle t1 = make_triangle_xy();
    Triangle t2(Vector3D(0.5, -0.2, 0),
                Vector3D(1.0, -0.2, 0),
                Vector3D(0.5, 0.5, 0));
    EXPECT_TRUE(t1.is_intersect(t2));
}

TEST(TriangleIntersection, CoplanarTouchAtVertex) {
    Triangle t1 = make_triangle_xy();
    Triangle t2(Vector3D(1.0, 0.0, 0),
                Vector3D(2.0, 0.0, 0),
                Vector3D(1.0, 1.0, 0));
    EXPECT_TRUE(t1.is_intersect(t2));
}

TEST(TriangleIntersection, CoplanarTouchAlongEdge) {
    Triangle t1 = make_triangle_xy();
    Triangle t2(Vector3D(0.0, 1.0, 0),
                Vector3D(1.0, 0.0, 0),
                Vector3D(1.0, 1.0, 0));
    EXPECT_TRUE(t1.is_intersect(t2));
}

TEST(TriangleIntersection, CoplanarDisjoint) {
    Triangle t1 = make_triangle_xy();
    Triangle t2(Vector3D(2.0, 0.0, 0),
                Vector3D(3.0, 0.0, 0),
                Vector3D(2.0, 1.0, 0));
    EXPECT_FALSE(t1.is_intersect(t2));
}

TEST(TriangleIntersection, NonCoplanarShouldFail) {
    Triangle t1 = make_triangle_xy();
    Triangle t2(Vector3D(0, 0, 1),
                Vector3D(1, 0, 1),
                Vector3D(0, 1, 1));
    EXPECT_FALSE(t1.is_intersect(t2));
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
    Section s2(Vector3D(2 + math::eps * 10, 0, 0), Vector3D(3, 0, 0)); // разрыв > eps
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
    Section s2(Vector3D(1 + math::eps * 10, 0, 0), Vector3D(2 + math::eps * 10, 0, 0));
    EXPECT_FALSE(s1.is_intersect(s2)); // зазор больше eps
}

// === ПРОСТЫЕ НЕПЕРЕСЕКАЮЩИЕСЯ ===
TEST(Triangle3DTest, ParallelNoIntersection) {
    Triangle t1(Vector3D(0,0,0), Vector3D(1,0,0), Vector3D(0,1,0));
    Triangle t2(Vector3D(0,0,1), Vector3D(1,0,1), Vector3D(0,1,1)); // параллельный, выше
    EXPECT_FALSE(t1.is_intersect(t2));
}

// === КОПЛАНАРНЫЕ (пересечение по линии / вершине) ===
TEST(Triangle3DTest, CoplanarTouchAtEdge) {
    Triangle t1(Vector3D(0,0,0), Vector3D(1,0,0), Vector3D(0,1,0));
    Triangle t2(Vector3D(1,0,0), Vector3D(1,1,0), Vector3D(0,1,0));
    EXPECT_TRUE(t1.is_intersect(t2));
}

TEST(Triangle3DTest, CoplanarDisjoint) {
    Triangle t1(Vector3D(0,0,0), Vector3D(1,0,0), Vector3D(0,1,0));
    Triangle t2(Vector3D(2,0,0), Vector3D(3,0,0), Vector3D(2,1,0));
    EXPECT_FALSE(t1.is_intersect(t2));
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
    EXPECT_TRUE(t1.is_intersect(t2));
}

// === НЕ КОПЛАНАРНЫЕ, ПЕРЕСЕКАЮТСЯ ПОД УГЛОМ ===
TEST(Triangle3DTest, IntersectAtLine) {
    // Один треугольник в плоскости z=0, второй наклонён
    Triangle t1(Vector3D(0,0,0), Vector3D(2,0,0), Vector3D(0,2,0));
    Triangle t2(Vector3D(1,-1,-1), Vector3D(1,1,1), Vector3D(1,1,-1));
    EXPECT_TRUE(t1.is_intersect(t2));
}

TEST(Triangle3DTest, IntersectAtPointOnly) {
    Triangle t1(Vector3D(0,0,0), Vector3D(2,0,0), Vector3D(0,2,0));
    Triangle t2(Vector3D(2,0,0), Vector3D(2,1,1), Vector3D(2,-1,1)); // пересекаются в точке (2,0,0)
    EXPECT_TRUE(t1.is_intersect(t2));
}

TEST(Triangle3DTest, DisjointDifferentPlanes) {
    Triangle t1(Vector3D(0,0,0), Vector3D(1,0,0), Vector3D(0,1,0));
    Triangle t2(Vector3D(0,0,2), Vector3D(1,0,2), Vector3D(0,1,2));
    EXPECT_FALSE(t1.is_intersect(t2));
}

// === СЛУЧАЙ "ОДИН ТРЕУГОЛЬНИК ПРОРЕЗАЕТ ДРУГОЙ" ===
TEST(Triangle3DTest, SliceThrough) {
    Triangle t1(Vector3D(0,0,0), Vector3D(3,0,0), Vector3D(0,3,0));
    Triangle t2(Vector3D(0,1,-1), Vector3D(3,1,1), Vector3D(1,1,-1));
    EXPECT_TRUE(t1.is_intersect(t2));
}

// === ОДИН ВНУТРИ ДРУГОГО (в разных плоскостях, но пересечение отсутствует) ===
TEST(Triangle3DTest, ContainedButDifferentPlanes) {
    Triangle t1(Vector3D(0,0,0), Vector3D(4,0,0), Vector3D(0,4,0));
    Triangle t2(Vector3D(1,1,1), Vector3D(2,1,1), Vector3D(1,2,1)); // полностью выше
    EXPECT_FALSE(t1.is_intersect(t2));
}

// === ОДИН ВНУТРИ ДРУГОГО (копланарно) ===
TEST(Triangle3DTest, CoplanarOneInsideAnother) {
    Triangle t1(Vector3D(0,0,0), Vector3D(4,0,0), Vector3D(0,4,0));
    Triangle t2(Vector3D(1,1,0), Vector3D(2,1,0), Vector3D(1,2,0));
    EXPECT_TRUE(t1.is_intersect(t2));
}

// =========================================================
// === НАБОР ВСЕХ СЛУЧАЕВ ПЕРЕСЕЧЕНИЙ ТРЕУГОЛЬНИКОВ ========
// =========================================================

// 1. Полное совпадение
TEST(Triangle3DTest, IdenticalTriangles) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({0,0,0}, {1,0,0}, {0,1,0});
    EXPECT_TRUE(t1.is_intersect(t2));
}

// 2. Копланарные, но не пересекаются
TEST(Triangle3DTest, CoplanarSeparate) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({2,0,0}, {3,0,0}, {2,1,0});
    EXPECT_FALSE(t1.is_intersect(t2));
}

// 3. Копланарные, касаются в одной вершине
TEST(Triangle3DTest, CoplanarTouchAtVertex) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({1,0,0}, {2,0,0}, {1,1,0});
    EXPECT_TRUE(t1.is_intersect(t2));
}

// 4. Копланарные, имеют общую сторону
TEST(Triangle3DTest, CoplanarShareEdge) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({1,0,0}, {1,1,0}, {0,1,0});
    EXPECT_TRUE(t1.is_intersect(t2));
}

// 5. Пересекаются под углом (z=0 и x=1)
TEST(Triangle3DTest, IntersectAtLine2) {
    Triangle t1({0,0,0}, {2,0,0}, {0,2,0});          // плоскость z = 0
    Triangle t2({1,-1,-1}, {1,1,1}, {1,1,-1});       // плоскость x = 1
    EXPECT_TRUE(t1.is_intersect(t2)); // пересечение по линии x=1, z=0
}

// 6. Касание по одной точке (вершина–плоскость)
TEST(Triangle3DTest, TouchAtSingleVertex) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});          // z=0
    Triangle t2({0,0,0}, {0,1,-1}, {1,0,-1});        // пересекается только в (0,0,0)
    EXPECT_TRUE(t1.is_intersect(t2));
}

// 7. Касание ребро–ребро (одна точка)
TEST(Triangle3DTest, EdgeEdgeContact) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({1,0,0}, {1,1,1}, {1,-1,1});
    EXPECT_TRUE(t1.is_intersect(t2)); // только (1,0,0)
}

// 8. Один треугольник полностью лежит внутри другого (копланарно)
TEST(Triangle3DTest, OneInsideAnotherCoplanar) {
    Triangle t1({0,0,0}, {4,0,0}, {0,4,0});
    Triangle t2({1,1,0}, {2,1,0}, {1,2,0});
    EXPECT_TRUE(t1.is_intersect(t2));
}

// 9. Один треугольник над другим (параллельные плоскости)
TEST(Triangle3DTest, ParallelNoIntersection2) {
    Triangle t1({0,0,0}, {2,0,0}, {0,2,0});
    Triangle t2({0,0,1}, {2,0,1}, {0,2,1});
    EXPECT_FALSE(t1.is_intersect(t2));
}

// 10. Почти параллельные (угол ≈ 180°, пересечения нет)
TEST(Triangle3DTest, AlmostParallel) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({0,0,1e-5f}, {1,0,1e-5f}, {0,1,1e-5f});
    EXPECT_FALSE(t1.is_intersect(t2));
}

// 11. Пересекаются ребром другого (отрезок–отрезок)
TEST(Triangle3DTest, EdgeCrossing) {
    Triangle t1({0,0,0}, {2,0,0}, {0,2,0});
    Triangle t2({1,-1,-1}, {1,1,1}, {1,1,-1});
    EXPECT_TRUE(t1.is_intersect(t2)); // отрезок по линии x=1,z=0
}

// 14. Пересекаются по внутренней линии, не касаясь вершин
TEST(Triangle3DTest, InnerLineCross) {
    Triangle t1({0,0,0}, {3,0,0}, {0,3,0});
    Triangle t2({1,1,-1}, {1,1,1}, {1.5,0.5,0});
    EXPECT_TRUE(t1.is_intersect(t2));
}

// 16. Пересекаются одной вершиной, но вне сторон
TEST(Triangle3DTest, SingleVertexOnPlane) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({0,0,0}, {1,-1,-1}, {1,1,1});
    EXPECT_TRUE(t1.is_intersect(t2)); // в точке (0,0,0)
}

// 17. Полное пересечение по линии, треугольники не копланарны
TEST(Triangle3DTest, SkewPlanesLineIntersection) {
    Triangle t1({0,0,0}, {2,0,0}, {0,2,0});
    Triangle t2({0,0,1}, {2,2,-1}, {2,0,1});
    EXPECT_TRUE(t1.is_intersect(t2));
}

// 18. Треугольники скрещиваются (не копланарны, не пересекаются)
TEST(Triangle3DTest, SkewNoIntersection) {
    Triangle t1({0,0,0}, {2,0,0}, {0,2,0});
    Triangle t2({1,1,1}, {2,1,1}, {1,2,1});
    EXPECT_FALSE(t1.is_intersect(t2));
}

// 19. Один внутри другого, но в параллельной плоскости (без касания)
TEST(Triangle3DTest, InsideButParallel) {
    Triangle t1({0,0,0}, {4,0,0}, {0,4,0});
    Triangle t2({1,1,1}, {2,1,1}, {1,2,1});
    EXPECT_FALSE(t1.is_intersect(t2));
}

// 20. Общая сторона, но нормали противоположны (обратный порядок вершин)
TEST(Triangle3DTest, SharedEdgeOppositeNormal) {
    Triangle t1({0,0,0}, {1,0,0}, {0,1,0});
    Triangle t2({1,0,0}, {0,1,0}, {1,1,0}); // обратный порядок
    EXPECT_TRUE(t1.is_intersect(t2));
}

Triangle make_point_triangle(const Vector3D& p) {
    return Triangle{p, p, p};
}

// Коллинеарный треугольник: вершины на одной прямой
Triangle make_segment_triangle(const Vector3D& a,
                               const Vector3D& b,
                               const Vector3D& c) {
    return Triangle{a, b, c};
}

// --------- Классификация треугольников ---------

TEST(TriangleDegenerateTest, PointTriangleIsPointNotSection) {
    Triangle t = make_point_triangle({0.f, 0.f, 0.f});

    EXPECT_TRUE(t.is_point());
    EXPECT_FALSE(t.is_section());
}

TEST(TriangleDegenerateTest, SegmentTriangleIsSectionNotPoint_DistinctVertices) {
    // Вершины на оси X: (0,0,0)–(1,0,0)–(2,0,0)
    Triangle t = make_segment_triangle(
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
        {2.f, 0.f, 0.f}
    );

    EXPECT_FALSE(t.is_point());
    EXPECT_TRUE(t.is_section());
}

TEST(TriangleDegenerateTest, SegmentTriangleIsSectionWhenTwoVerticesCoincide) {
    // Две вершины совпадают, третья на прямой → отрезок
    Triangle t = make_segment_triangle(
        {0.f, 0.f, 0.f},
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f}
    );

    EXPECT_FALSE(t.is_point());
    EXPECT_TRUE(t.is_section());
}

// --------- Point–Point ---------

TEST(TriangleDegenerateIntersection, PointPointSameLocationIntersect) {
    Triangle t1 = make_point_triangle({1.f, 2.f, 3.f});
    Triangle t2 = make_point_triangle({1.f, 2.f, 3.f});

    EXPECT_TRUE(t1.is_intersect(t2));
    EXPECT_TRUE(t2.is_intersect(t1));
}

TEST(TriangleDegenerateIntersection, PointPointDifferentLocationDoNotIntersect) {
    Triangle t1 = make_point_triangle({0.f, 0.f, 0.f});
    Triangle t2 = make_point_triangle({1.f, 0.f, 0.f});

    EXPECT_FALSE(t1.is_intersect(t2));
    EXPECT_FALSE(t2.is_intersect(t1));
}

// --------- Point–Section ---------

TEST(TriangleDegenerateIntersection, PointOnSegmentTriangleIntersect) {
    // Треугольник-отрезок на оси X: (0,0,0)–(2,0,0)–(1,0,0)
    Triangle seg = make_segment_triangle(
        {0.f, 0.f, 0.f},
        {2.f, 0.f, 0.f},
        {1.f, 0.f, 0.f}
    );
    Triangle pt  = make_point_triangle({1.f, 0.f, 0.f}); // лежит на отрезке

    // Ветка: this->is_point() && other.is_section()
    EXPECT_TRUE(pt.is_intersect(seg));
    // Ветка: this->is_section() && other.is_point()
    EXPECT_TRUE(seg.is_intersect(pt));
}

TEST(TriangleDegenerateIntersection, PointOutsideSegmentTriangleNoIntersect) {
    Triangle seg = make_segment_triangle(
        {0.f, 0.f, 0.f},
        {2.f, 0.f, 0.f},
        {1.f, 0.f, 0.f}
    );
    Triangle pt  = make_point_triangle({3.f, 0.f, 0.f}); // вне отрезка

    EXPECT_FALSE(pt.is_intersect(seg));
    EXPECT_FALSE(seg.is_intersect(pt));
}

// --------- Section–Section ---------

TEST(TriangleDegenerateIntersection, CollinearOverlappingSegmentsIntersect) {
    // Первый «треугольник-отрезок»: [0,2] по оси X
    Triangle s1 = make_segment_triangle(
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
        {2.f, 0.f, 0.f}
    );
    // Второй: [1,3] по оси X
    Triangle s2 = make_segment_triangle(
        {1.f, 0.f, 0.f},
        {3.f, 0.f, 0.f},
        {2.f, 0.f, 0.f}
    );

    // Ветка: this->is_section() && other->is_section()
    EXPECT_TRUE(s1.is_intersect(s2));
    EXPECT_TRUE(s2.is_intersect(s1));
}

TEST(TriangleDegenerateIntersection, CollinearDisjointSegmentsDoNotIntersect) {
    // Первый: [0,1]
    Triangle s1 = make_segment_triangle(
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
        {0.5f, 0.f, 0.f}
    );
    // Второй: [2,3]
    Triangle s2 = make_segment_triangle(
        {2.f, 0.f, 0.f},
        {3.f, 0.f, 0.f},
        {2.5f, 0.f, 0.f}
    );

    EXPECT_FALSE(s1.is_intersect(s2));
    EXPECT_FALSE(s2.is_intersect(s1));
}

TEST(TriangleDegenerateIntersection, SkewSegmentsIn3DDoNotIntersect) {
    // Отрезок вдоль X в z=0
    Triangle s1 = make_segment_triangle(
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
        {0.5f, 0.f, 0.f}
    );
    // Отрезок вдоль Y в z=1
    Triangle s2 = make_segment_triangle(
        {0.f, 0.f, 1.f},
        {0.f, 1.f, 1.f},
        {0.f, 0.5f, 1.f}
    );

    EXPECT_FALSE(s1.is_intersect(s2));
    EXPECT_FALSE(s2.is_intersect(s1));
}

// --------- Section–Нормальный треугольник ---------

TEST(TriangleDegenerateIntersection, SegmentCrossesProperTriangle) {
    // Нормальный треугольник в плоскости z=0
    Triangle tri(
        {0.f, 0.f, 0.f},
        {2.f, 0.f, 0.f},
        {0.f, 2.f, 0.f}
    );

    // Треугольник-отрезок: от (1, -1, 0) до (1, 3, 0) (пересекает tri по высоте)
    Triangle seg = make_segment_triangle(
        {1.f, -1.f, 0.f},
        {1.f,  3.f, 0.f},
        {1.f,  0.f, 0.f}
    );

    // Ветка: this->is_section() → other.is_intersect(Section)
    EXPECT_TRUE(seg.is_intersect(tri));
    // Ветка: other.is_section() → this->is_intersect(Section)
    EXPECT_TRUE(tri.is_intersect(seg));
}

TEST(TriangleDegenerateIntersection, SegmentFarFromProperTriangleNoIntersect) {
    Triangle tri(
        {0.f, 0.f, 0.f},
        {2.f, 0.f, 0.f},
        {0.f, 2.f, 0.f}
    );

    // Отрезок в параллельной плоскости z=1
    Triangle seg = make_segment_triangle(
        {0.f, 0.f, 1.f},
        {2.f, 0.f, 1.f},
        {1.f, 1.f, 1.f}
    );

    EXPECT_FALSE(seg.is_intersect(tri));
    EXPECT_FALSE(tri.is_intersect(seg));
}