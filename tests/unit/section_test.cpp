#include <gtest/gtest.h>
#include "geometry/section.hpp"
#include "geometry/vector_3d.hpp"
#include "geometry/line.hpp"
#include <cmath>

using namespace geometry;

constexpr float EPS = 1e-6f;

// === Конструктор и базовые проверки ===
TEST(SectionTest, ConstructorAndValidity) {
    Vector3D a(0, 0, 0);
    Vector3D b(1, 1, 1);
    Section s(a, b);

    EXPECT_TRUE(s.is_valid());

    // Невалидный — точки совпадают
    Section invalid(a, a);
    EXPECT_FALSE(invalid.is_valid());
}

// === Проверка длины ===
TEST(SectionTest, Length) {
    Section s(Vector3D(0, 0, 0), Vector3D(3, 4, 0));
    EXPECT_NEAR(s.length(), 5.0f, EPS);
}

// === Проверка get_line ===
TEST(SectionTest, GetLine) {
    Vector3D a(0, 0, 0);
    Vector3D b(1, 1, 1);
    Section s(a, b);

    Line l = s.get_line();
    EXPECT_TRUE(l.is_valid());
}

// === Проверка is_contains (принадлежность точки отрезку) ===
TEST(SectionTest, IsContains) {
    Section s(Vector3D(0, 0, 0), Vector3D(10, 0, 0));

    EXPECT_TRUE(s.is_contains(Vector3D(5, 0, 0)));   // середина
    EXPECT_TRUE(s.is_contains(Vector3D(0, 0, 0)));   // начало
    EXPECT_TRUE(s.is_contains(Vector3D(10, 0, 0)));  // конец
    EXPECT_FALSE(s.is_contains(Vector3D(11, 0, 0))); // за пределами
    EXPECT_FALSE(s.is_contains(Vector3D(5, 1, 0)));  // не на линии
}

// === Проверка пересечения отрезков ===
// Здесь предполагается, что класс Line корректно реализует пересечение линий.
TEST(SectionTest, Intersect) {
    Section s1(Vector3D(0, 0, 0), Vector3D(10, 0, 0));
    Section s2(Vector3D(5, -5, 0), Vector3D(5, 5, 0));

    EXPECT_TRUE(s1.is_intersect(s2)); // пересекаются в (5,0,0)

    Section s3(Vector3D(11, 0, 0), Vector3D(20, 0, 0));
    EXPECT_FALSE(s1.is_intersect(s3)); // не пересекаются
}

// === Проверка невалидных данных ===
TEST(SectionTest, InvalidSection) {
    Section s_invalid(Vector3D(NAN, 0, 0), Vector3D(1, 1, 1));
    EXPECT_FALSE(s_invalid.is_valid());
}

// === Проверка на совпадающие точки (граничный случай) ===
TEST(SectionTest, DegenerateSection) {
    Vector3D p(1, 1, 1);
    Section s(p, p);
    EXPECT_FALSE(s.is_valid());
}
constexpr float eps = 1e-6f;

TEST(SectionTest, ContainsMiddlePoint) {
    Section s(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    EXPECT_TRUE(s.is_contains(Vector3D(0.5, 0, 0)));
}

TEST(SectionTest, ContainsEndpoints) {
    Section s(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    EXPECT_TRUE(s.is_contains(Vector3D(0, 0, 0)));
    EXPECT_TRUE(s.is_contains(Vector3D(1, 0, 0)));
}

TEST(SectionTest, JustOutsideEndpoints) {
    Section s(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    EXPECT_FALSE(s.is_contains(Vector3D(-eps * 10, 0, 0)));
    EXPECT_FALSE(s.is_contains(Vector3D(1 + eps * 10, 0, 0)));
}

TEST(SectionTest, ParallelNonIntersecting) {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Section s2(Vector3D(0, 1, 0), Vector3D(1, 1, 0));
    EXPECT_FALSE(s1.is_intersect(s2));
}

TEST(SectionTest, CrossingAtCenter) {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 1, 0));
    Section s2(Vector3D(0, 1, 0), Vector3D(1, 0, 0));
    EXPECT_TRUE(s1.is_intersect(s2));
}

TEST(SectionTest, TouchAtEndpoint) {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Section s2(Vector3D(1, 0, 0), Vector3D(2, 0, 0));
    EXPECT_TRUE(s1.is_intersect(s2)); // касание концами
}

TEST(SectionTest, CollinearOverlap) {
    Section s1(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Section s2(Vector3D(1, 0, 0), Vector3D(3, 0, 0));
    EXPECT_TRUE(s1.is_intersect(s2)); // пересечение на [1,2]
}

TEST(SectionTest, CollinearDisjoint) {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Section s2(Vector3D(2, 0, 0), Vector3D(3, 0, 0));
    EXPECT_FALSE(s1.is_intersect(s2));
}

TEST(SectionTest, CollinearTouchWithEps) {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Section s2(Vector3D(1 + eps / 2, 0, 0), Vector3D(2, 0, 0));
    // почти касаются — должны считаться пересекающимися из-за допусков
    EXPECT_TRUE(s1.is_intersect(s2));
}

TEST(SectionTest, SkewIn3DNoIntersection) {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Section s2(Vector3D(0.5, 0.5, 1), Vector3D(0.5, -0.5, 1));
    EXPECT_FALSE(s1.is_intersect(s2));
}

TEST(SectionTest, CollinearDiagonalOverlap) {
    Section s1(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
    Section s2(Vector3D(0.5, 0.5, 0.5), Vector3D(2, 2, 2));
    EXPECT_TRUE(s1.is_intersect(s2));
}

TEST(SectionTest, CollinearReversedDirectionOverlap) {
    Section s1(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Section s2(Vector3D(1.5, 0, 0), Vector3D(-1, 0, 0)); // обратное направление
    EXPECT_TRUE(s1.is_intersect(s2));
}
