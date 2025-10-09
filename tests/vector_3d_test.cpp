#include <gtest/gtest.h>
#include "geometry/vector_3d.hpp"
#include <cmath>

using namespace geometry;

constexpr float EPS = 1e-6f;

// Проверка конструктора и геттеров
TEST(Vector3DTest, ConstructorAndGetters) {
    Vector3D v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v.x(), 1.0f);
    EXPECT_FLOAT_EQ(v.y(), 2.0f);
    EXPECT_FLOAT_EQ(v.z(), 3.0f);
}

// Проверка is_valid
TEST(Vector3DTest, IsValid) {
    Vector3D v1(1, 2, 3);
    Vector3D v2(NAN, 0, 1);
    EXPECT_TRUE(v1.is_valid());
    EXPECT_FALSE(v2.is_valid());
}

// Проверка is_zero
TEST(Vector3DTest, IsZero) {
    Vector3D v1(0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(v1.is_zero());

    Vector3D v2(1e-3f, 0.0f, 0.0f);
    EXPECT_FALSE(v2.is_zero());
}

// Проверка длины
TEST(Vector3DTest, Length) {
    Vector3D v(3.0f, 4.0f, 0.0f);
    EXPECT_NEAR(v.length(), 5.0f, EPS);
}

// Проверка скалярного произведения
TEST(Vector3DTest, ScalarProduct) {
    Vector3D a(1, 2, 3);
    Vector3D b(4, -5, 6);
    EXPECT_FLOAT_EQ(a.scalar(b), 1*4 + 2*(-5) + 3*6); // 12
}

// Проверка сложения и вычитания
TEST(Vector3DTest, AdditionAndSubtraction) {
    Vector3D a(1, 2, 3);
    Vector3D b(4, 5, 6);

    Vector3D sum = a + b;
    Vector3D diff = a - b;

    EXPECT_FLOAT_EQ(sum.x(), 5);
    EXPECT_FLOAT_EQ(sum.y(), 7);
    EXPECT_FLOAT_EQ(sum.z(), 9);

    EXPECT_FLOAT_EQ(diff.x(), -3);
    EXPECT_FLOAT_EQ(diff.y(), -3);
    EXPECT_FLOAT_EQ(diff.z(), -3);
}

// Проверка умножения на скаляр
TEST(Vector3DTest, ScalarMultiplication) {
    Vector3D v(1, -2, 3);
    Vector3D r = v * 2.0f;
    EXPECT_FLOAT_EQ(r.x(), 2);
    EXPECT_FLOAT_EQ(r.y(), -4);
    EXPECT_FLOAT_EQ(r.z(), 6);
}

// Проверка векторного произведения
TEST(Vector3DTest, CrossProduct) {
    Vector3D a(1, 0, 0);
    Vector3D b(0, 1, 0);
    Vector3D c = a.cross(b);

    EXPECT_NEAR(c.x(), 0, EPS);
    EXPECT_NEAR(c.y(), 0, EPS);
    EXPECT_NEAR(c.z(), 1, EPS);
}

// Проверка исключения при cross с нулевым вектором
TEST(Vector3DTest, CrossProductWithZeroThrows) {
    Vector3D a(1, 0, 0);
    Vector3D zero(0, 0, 0);

    EXPECT_THROW(a.cross(zero), std::invalid_argument);
    EXPECT_THROW(zero.cross(a), std::invalid_argument);
}

// Проверка коллинеарности
TEST(Vector3DTest, Collinearity) {
    Vector3D a(1, 2, 3);
    Vector3D b(2, 4, 6);
    Vector3D c(0, 1, 0);

    EXPECT_TRUE(a.is_collinear(b));
    EXPECT_FALSE(a.is_collinear(c));
}
