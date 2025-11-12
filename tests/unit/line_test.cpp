#include <gtest/gtest.h>
#include "geometry/line.hpp"
#include "geometry/vector_3d.hpp"

using namespace geometry;

TEST(LineTest, ConstructorThrowsOnZeroDirection) {
    EXPECT_THROW(Line(Vector3D(0, 0, 0), Vector3D(0, 0, 0)), std::invalid_argument);
}

TEST(LineTest, ValidLineCheck) {
    Line line(Vector3D(1, 2, 3), Vector3D(1, 0, 0));
    EXPECT_TRUE(line.is_valid());
}

TEST(LineTest, InvalidLineWithZeroDirection) {
    // Валидная линия — не должна выбросить исключение
    EXPECT_NO_THROW(Line(Vector3D(0, 0, 0), Vector3D(0, 0, 1)));

    // Невалидная линия — должна выбросить std::invalid_argument
    EXPECT_THROW(Line(Vector3D(0, 0, 0), Vector3D(0, 0, 0)), std::invalid_argument);
}

TEST(LineTest, ParallelLines) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
    Line l2(Vector3D(1, 2, 3), Vector3D(2, 2, 2));
    EXPECT_TRUE(l1.is_parallel(l2));
}

TEST(LineTest, NotParallelLines) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 0, 0), Vector3D(0, 1, 0));
    EXPECT_FALSE(l1.is_parallel(l2));
}

TEST(LineTest, ContainsPoint) {
    Line l(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    EXPECT_TRUE(l.is_contains(Vector3D(5, 0, 0)));
    EXPECT_FALSE(l.is_contains(Vector3D(0, 1, 0)));
}

TEST(LineTest, IntersectSkewLines) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 1, 1), Vector3D(0, 1, -1));
    EXPECT_FALSE(l1.is_intersect(l2));
}

TEST(LineTest, IntersectCrossingLines) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 0, 0), Vector3D(0, 1, 0));
    EXPECT_TRUE(l1.is_intersect(l2));
}

TEST(LineTest, IntersectPointCorrectness) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 1, 0));
    Line l2(Vector3D(1, 0, 0), Vector3D(-1, 1, 0));
    Vector3D p = l1.intersect_point(l2);
    EXPECT_NEAR(p.x, 0.5, 1e-6);
    EXPECT_NEAR(p.y, 0.5, 1e-6);
    EXPECT_NEAR(p.z, 0.0, 1e-6);
}

TEST(LineTest, NonIntersectingLinesReturnNanPoint) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 1, 0), Vector3D(0, 0, 1));
    Vector3D p = l1.intersect_point(l2);
    EXPECT_TRUE(std::isnan(p.x));
    EXPECT_TRUE(std::isnan(p.y));
    EXPECT_TRUE(std::isnan(p.z));
}

// =============== CONSTRUCTOR TESTS ===============
TEST(LineTest, ThrowsOnZeroDirection) {
    EXPECT_THROW(Line(Vector3D(0, 0, 0), Vector3D(0, 0, 0)), std::invalid_argument);
}

TEST(LineTest, AcceptsValidDirection) {
    EXPECT_NO_THROW(Line(Vector3D(0, 0, 0), Vector3D(1, 2, 3)));
}

TEST(LineTest, OriginAndDirectionAreStoredCorrectly) {
    Vector3D origin(1, 2, 3);
    Vector3D dir(4, 5, 6);
    Line l(origin, dir);
    EXPECT_NEAR(l.origin().x, origin.x, 1e-6);
    EXPECT_NEAR(l.origin().y, origin.y, 1e-6);
    EXPECT_NEAR(l.origin().z, origin.z, 1e-6);
    EXPECT_NEAR(l.dir().x, dir.x, 1e-6);
    EXPECT_NEAR(l.dir().y, dir.y, 1e-6);
    EXPECT_NEAR(l.dir().z, dir.z, 1e-6);
}

// =============== VALIDITY TESTS ===============
TEST(LineTest, ValidityWorksCorrectly) {
    // Валидная линия
    EXPECT_NO_THROW(Line(Vector3D(0, 0, 0), Vector3D(1, 0, 0)));

    // Невалидная линия — должен быть выброс исключения
    EXPECT_THROW(Line(Vector3D(0, 0, 0), Vector3D(0, 0, 0)), std::invalid_argument);
}

// =============== PARALLELISM TESTS ===============
TEST(LineTest, DetectsParallelLines) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 2, 3));
    Line l2(Vector3D(1, 1, 1), Vector3D(2, 4, 6));
    EXPECT_TRUE(l1.is_parallel(l2));
}

TEST(LineTest, DetectsNonParallelLines) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 0, 0), Vector3D(0, 1, 0));
    EXPECT_FALSE(l1.is_parallel(l2));
}

// =============== CONTAINMENT TESTS ===============
TEST(LineTest, ContainsPointOnLine) {
    Line l(Vector3D(1, 2, 3), Vector3D(1, 0, 0));
    EXPECT_TRUE(l.is_contains(Vector3D(5, 2, 3)));
    EXPECT_TRUE(l.is_contains(Vector3D(-2, 2, 3)));
}

TEST(LineTest, DoesNotContainPointOffLine) {
    Line l(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    EXPECT_FALSE(l.is_contains(Vector3D(1, 1, 0)));
}

// =============== INTERSECTION TESTS ===============
TEST(LineTest, IntersectParallelLinesReturnFalse) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 1, 0), Vector3D(1, 0, 0));
    EXPECT_FALSE(l1.is_intersect(l2));
}

TEST(LineTest, IntersectCrossingLinesReturnTrue) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 0, 0), Vector3D(0, 1, 0));
    EXPECT_TRUE(l1.is_intersect(l2));
}

TEST(LineTest, IntersectSkewLinesReturnFalse) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 1, 1), Vector3D(0, 1, -1));
    EXPECT_FALSE(l1.is_intersect(l2));
}

// =============== INTERSECTION POINT TESTS ===============
TEST(LineTest, IntersectionPointIsCorrect) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 1, 0));
    Line l2(Vector3D(1, 0, 0), Vector3D(-1, 1, 0));
    Vector3D p = l1.intersect_point(l2);
    EXPECT_NEAR(p.x, 0.5, 1e-6);
    EXPECT_NEAR(p.y, 0.5, 1e-6);
    EXPECT_NEAR(p.z, 0.0, 1e-6);
}

TEST(LineTest, NonIntersectingLinesReturnNaN) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Line l2(Vector3D(0, 1, 0), Vector3D(0, 0, 1));
    Vector3D p = l1.intersect_point(l2);
    EXPECT_TRUE(std::isnan(p.x));
    EXPECT_TRUE(std::isnan(p.y));
    EXPECT_TRUE(std::isnan(p.z));
}

// =============== EDGE CASES ===============
TEST(LineTest, HandlesLargeNumbers) {
    Line l1(Vector3D(1e9, 1e9, 1e9), Vector3D(1, 1, 0));
    Line l2(Vector3D(1e9, 1e9, 1e9), Vector3D(-1, 1, 0));
    EXPECT_TRUE(l1.is_intersect(l2));
    Vector3D p = l1.intersect_point(l2);
    EXPECT_TRUE(p.is_valid());
    EXPECT_FALSE(std::isnan(p.x));
}

TEST(LineTest, HandlesNegativeCoordinates) {
    Line l1(Vector3D(-10, -10, 0), Vector3D(1, 1, 0));
    Line l2(Vector3D(-9, -11, 0), Vector3D(1, -1, 0));
    Vector3D p = l1.intersect_point(l2);
    EXPECT_NEAR(p.x, -10.0, 1e-6);
    EXPECT_NEAR(p.y, -10.0, 1e-6);
}

TEST(LineTest, IntersectionWithSameLineGivesValidPoint) {
    Line l1(Vector3D(0, 0, 0), Vector3D(1, 1, 0));
    Line l2(Vector3D(2, 2, 0), Vector3D(2, 2, 0)); // параллельна и совпадает
    EXPECT_TRUE(l1.is_match(l2));
    EXPECT_TRUE(l1.is_intersect(l2)); // строго говоря, совпадение = бесконечное множество, не “true”
}
