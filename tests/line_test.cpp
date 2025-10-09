#include <gtest/gtest.h>
#include "geometry/line.hpp"
#include "geometry/vector_3d.hpp"
#include <cmath>

using namespace geometry;

constexpr float EPS = 1e-6f;

// === Конструктор и поля ===
TEST(LineTest, ConstructorAndAccessors) {
    Vector3D origin(1, 2, 3);
    Vector3D dir(1, 0, 0);

    Line l(origin, dir);

    EXPECT_FLOAT_EQ(l.origin().x(), 1);
    EXPECT_FLOAT_EQ(l.origin().y(), 2);
    EXPECT_FLOAT_EQ(l.origin().z(), 3);

    EXPECT_FLOAT_EQ(l.dir().x(), 1);
    EXPECT_FLOAT_EQ(l.dir().y(), 0);
    EXPECT_FLOAT_EQ(l.dir().z(), 0);
}

// === Проверка выбрасывания исключения при нулевом направлении ===
TEST(LineTest, ConstructorThrowsOnZeroDir) {
    Vector3D origin(0, 0, 0);
    Vector3D zero(0, 0, 0);
    EXPECT_THROW(Line l(origin, zero), std::invalid_argument);
}

// === Проверка is_valid ===
TEST(LineTest, IsValid) {
    Line l(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    EXPECT_TRUE(l.is_valid());

    // Некорректный случай
    Vector3D nan(NAN, 0, 0);
    Line invalid(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    EXPECT_TRUE(invalid.is_valid()); // оба валидны

    // Проверим вручную невалидный (с помощью NAN)
    Line invalid2(Vector3D(NAN, 0, 0), Vector3D(1, 0, 0));
    EXPECT_FALSE(invalid2.is_valid());
}

// === Проверка параллельности ===
TEST(LineTest, IsParallel) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 1, 0), Vector3D(2, 0, 0)); // параллельны
    Line l3(Vector3D(0, 0, 0), Vector3D(0, 1, 0)); // не параллельны

    EXPECT_TRUE(l1.is_parallel(l2));
    EXPECT_FALSE(l1.is_parallel(l3));
}

// === Проверка принадлежности точки прямой ===
TEST(LineTest, IsContains) {
    Line l(Vector3D(0, 0, 0), Vector3D(1, 1, 1));

    EXPECT_TRUE(l.is_contains(Vector3D(2, 2, 2)));  // на линии
    EXPECT_TRUE(l.is_contains(Vector3D(-1, -1, -1))); // продолжение
    EXPECT_FALSE(l.is_contains(Vector3D(1, 2, 3))); // не на линии
}

// === Проверка пересечения прямых ===
TEST(LineTest, IsIntersect) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 0, 0), Vector3D(0, 1, 0));

    EXPECT_TRUE(l1.is_intersect(l2)); // пересекаются в начале координат

    Line l3(Vector3D(0, 1, 0), Vector3D(1, 0, 0));
    EXPECT_FALSE(l1.is_intersect(l3)); // параллельны и не пересекаются

    Line l4(Vector3D(0, 1, 0), Vector3D(0, 0, 1));
    EXPECT_FALSE(l1.is_intersect(l4)); // скрещивающиеся
}

// === Проверка intersect_point ===
TEST(LineTest, IntersectPoint) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0)); // ось X
    Line l2(Vector3D(0, 0, 0), Vector3D(0, 1, 0)); // ось Y

    EXPECT_TRUE(l1.is_intersect(l2));

    Vector3D p = l1.intersect_point(l2);
    EXPECT_NEAR(p.x(), 0.0f, EPS);
    EXPECT_NEAR(p.y(), 0.0f, EPS);
    EXPECT_NEAR(p.z(), 0.0f, EPS);
}

// === Проверка intersect_point для непересекающихся линий ===
TEST(LineTest, IntersectPointNoIntersection) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 1, 0), Vector3D(0, 0, 1));

    EXPECT_FALSE(l1.is_intersect(l2));
}

// === Проверка точек пересечения при пересекающихся наклонных прямых ===
TEST(LineTest, IntersectPointDiagonal) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 1, 0));
    Line l2(Vector3D(1, 0, 0), Vector3D(-1, 1, 0));

    EXPECT_TRUE(l1.is_intersect(l2));

    Vector3D p = l1.intersect_point(l2);
    EXPECT_NEAR(p.x(), 0.5f, EPS);
    EXPECT_NEAR(p.y(), 0.5f, EPS);
    EXPECT_NEAR(p.z(), 0.0f, EPS);
}