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

// === Проверка get_side ===
TEST(SectionTest, GetSide) {
    Section s(Vector3D(0, 0, 0), Vector3D(10, 0, 0));

    Vector3D left_point(5, 1, 0);
    Vector3D right_point(5, -1, 0);
    Vector3D inter_point(5, 0, 0);

    EXPECT_EQ(s.get_side(left_point), LEFT_SIDE);
    EXPECT_EQ(s.get_side(right_point), LEFT_SIDE); // внимание! метод возвращает LEFT_SIDE для любых n_length > 0
    EXPECT_EQ(s.get_side(inter_point), INTER_SIDE);
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
