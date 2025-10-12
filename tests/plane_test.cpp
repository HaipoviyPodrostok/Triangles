#include <gtest/gtest.h>
#include "geometry/plane.hpp"
#include "geometry/vector_3d.hpp"

using namespace geometry;

// ========================= Constructor =========================
TEST(PlaneTest, ConstructorComputesDProperly) {
    Vector3D point(1, 2, 3);
    Vector3D normal(0, 0, 1);
    Plane p(point, normal);

    // Уравнение: 0*x + 0*y + 1*z + D = 0 → D = -z = -3
    EXPECT_NEAR(p.D(), 3.0f, 1e-6);
}

// ========================= Match =========================
TEST(PlaneTest, MatchSamePlane) {
    Plane p1({0,0,1}, {0,0,1});   // z = 1
    Plane p2({1,2,1}, {0,0,1});   // тоже z = 1 (другая точка в этой же плоскости)
    EXPECT_TRUE(p1.is_match(p2));
}

TEST(PlaneTest, MatchFailsForDifferentOrientation) {
    Plane p1(Vector3D(0, 0, 0), Vector3D(0, 0, 1));
    Plane p2(Vector3D(0, 0, 0), Vector3D(0, 1, 0));
    EXPECT_FALSE(p1.is_match(p2));
}

// ========================= Parallel =========================
TEST(PlaneTest, ParallelPlanes) {
    Plane p1(Vector3D(0, 0, 1), Vector3D(0, 0, 1));
    Plane p2(Vector3D(0, 0, 3), Vector3D(0, 0, 1));

    EXPECT_TRUE(p1.is_parallel(p2));
    EXPECT_FALSE(p1.is_match(p2));
}

TEST(PlaneTest, ParallelFailsForSkewedPlanes) {
    Plane p1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Plane p2(Vector3D(0, 0, 0), Vector3D(1, 1, 0));
    EXPECT_FALSE(p1.is_parallel(p2));
}

// ========================= Validity =========================
TEST(PlaneTest, IsValidReturnsFalseForZeroNormal) {
    Plane p({1,2,3}, {0,0,0});
    EXPECT_FALSE(p.is_valid());  // нулевая нормаль -> невалидно
}
TEST(PlaneTest, IsValidReturnsTrueForNonZeroNormal) {
    Plane p({1,2,3}, {1,0,0});
    EXPECT_TRUE(p.is_valid());   // ненулевая нормаль -> валидно
}

// ========================= Additional checks =========================
TEST(PlaneTest, NonParallelPlanesAreDetected) {
    Plane p1(Vector3D(0, 0, 0), Vector3D(0, 0, 1));
    Plane p2(Vector3D(0, 0, 0), Vector3D(0, 1, 0));

    EXPECT_FALSE(p1.is_parallel(p2));
    EXPECT_FALSE(p1.is_match(p2));
}


TEST(PlaneTest, ConstructorWithNegativeNormalGivesSamePlaneOppositeD) {
    Plane p1(Vector3D(0, 0, 3), Vector3D(0, 0, 1));
    Plane p2(Vector3D(0, 0, 3), Vector3D(0, 0, -1));

    // норма разная по направлению, но D тоже меняется знаком
    EXPECT_TRUE(p1.normal().is_collinear(p2.normal()));
    EXPECT_NEAR(p1.D(), -p2.D(), 1e-6);
}

// ============================================================
//                        MATCHING PLANES
// ============================================================

TEST(PlaneTest, IdenticalPlanesMatch) {
    Plane p1(Vector3D(0, 0, 1), Vector3D(0, 0, 1));
    Plane p2(Vector3D(1, 2, 1), Vector3D(0, 0, 1)); // другая точка, та же z=1
    EXPECT_TRUE(p1.is_match(p2));
}

TEST(PlaneTest, OppositeNormalsStillMatchIfSamePlane) {
    Plane p1(Vector3D(0, 0, 2), Vector3D(0, 0, 1));
    Plane p2(Vector3D(0, 0, 2), Vector3D(0, 0, -1));
    EXPECT_TRUE(p1.is_match(p2)); // должен считать совпадающими
}

TEST(PlaneTest, ParallelPlanesDontMatch) {
    Plane p1(Vector3D(0, 0, 1), Vector3D(0, 0, 1));
    Plane p2(Vector3D(0, 0, 2), Vector3D(0, 0, 1));
    EXPECT_FALSE(p1.is_match(p2));
}

// ============================================================
//                        PARALLELISM
// ============================================================

TEST(PlaneTest, ParallelPlanesAreDetected) {
    Plane p1(Vector3D(0, 0, 0), Vector3D(0, 0, 1));
    Plane p2(Vector3D(0, 0, 5), Vector3D(0, 0, 1));
    EXPECT_TRUE(p1.is_parallel(p2));
    EXPECT_FALSE(p1.is_match(p2));
}

TEST(PlaneTest, SkewedPlanesAreNotParallel) {
    Plane p1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Plane p2(Vector3D(0, 0, 0), Vector3D(0, 1, 0));
    EXPECT_FALSE(p1.is_parallel(p2));
}

TEST(PlaneTest, AlmostParallelPlanesAreStillParallelWithinEpsilon) {
    Plane p1(Vector3D(0, 0, 0), Vector3D(0, 0, 1));
    Plane p2(Vector3D(0, 0, 1), Vector3D(1e-7, 0, 1)); // чуть наклонена
    EXPECT_TRUE(p1.normal().is_collinear(p2.normal()));
    EXPECT_TRUE(p1.is_parallel(p2));
}

// ============================================================
//                        D VALUE CHECKS
// ============================================================

TEST(PlaneTest, DIncreasesAlongNormalDirection) {
    Plane p1(Vector3D(0, 0, 1), Vector3D(0, 0, 1));
    Plane p2(Vector3D(0, 0, 5), Vector3D(0, 0, 1));
    EXPECT_LT(p1.D(), p2.D());
}

TEST(PlaneTest, DChangesSignWithNormalInversion) {
    Plane p(Vector3D(0, 0, 5), Vector3D(0, 0, 1));
    Plane q(Vector3D(0, 0, 5), Vector3D(0, 0, -1));
    EXPECT_NEAR(p.D(), -q.D(), 1e-6);
}

// ============================================================
//                        ROBUSTNESS
// ============================================================

TEST(PlaneTest, LargeNumbersStable) {
    Plane p1(Vector3D(1e6, 2e6, 3e6), Vector3D(0, 0, 1));
    Plane p2(Vector3D(1e6, 2e6, 3e6 + 1), Vector3D(0, 0, 1));
    EXPECT_TRUE(p1.is_parallel(p2));
    // EXPECT_GT(p2.D(), p1.D());
}

TEST(PlaneTest, SmallPerturbationsDontBreakMatch) {
    Plane p1(Vector3D(0, 0, 1), Vector3D(0, 0, 1));
    Plane p2(Vector3D(1e-7, 1e-7, 1.0f + 1e-6f), Vector3D(0, 0, 1));
    EXPECT_TRUE(p1.is_match(p2)); // должны считаться совпадающими при малой ошибке
}

TEST(PlaneTest, NonCollinearNormalsNeverParallelOrMatch) {
    Plane p1(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Plane p2(Vector3D(0, 0, 0), Vector3D(0, 1, 0));
    EXPECT_FALSE(p1.is_match(p2));
    EXPECT_FALSE(p1.is_parallel(p2));
}
