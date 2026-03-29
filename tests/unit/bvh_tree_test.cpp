#include "acceleration/bvh_tree.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include "geometry/geometry.hpp"

using namespace geometry;

// ======================== Helpers ========================

static std::vector<bool> brute_force_intersections(
    const std::vector<Triangle>& input) {
  std::vector<bool> result(input.size(), false);
  for (size_t i = 0; i < input.size(); ++i) {
    for (size_t j = i + 1; j < input.size(); ++j) {
      if (input[i].is_intersect(input[j])) {
        result[i] = true;
        result[j] = true;
      }
    }
  }
  return result;
}

// ================= Construction Tests ====================

TEST(BVHTreeTest, EmptyInput) {
  std::vector<Triangle>           input;
  acceleration::BVHTree<Triangle> tree(input);
  EXPECT_TRUE(tree.validate_tree());
}

TEST(BVHTreeTest, SingleTriangle) {
  std::vector<Triangle> input = {
      Triangle({0, 0, 0}, {1, 0, 0}, {0, 1, 0}),
  };
  acceleration::BVHTree<Triangle> tree(input);
  EXPECT_TRUE(tree.validate_tree());
}

TEST(BVHTreeTest, ThreeTriangles) {
  std::vector<Triangle> input = {
      Triangle({0, 0, 0}, {1, 0, 0}, {0, 1, 0}),
      Triangle({2, 0, 0}, {3, 0, 0}, {2, 1, 0}),
      Triangle({4, 0, 0}, {5, 0, 0}, {4, 1, 0}),
  };
  acceleration::BVHTree<Triangle> tree(input);
  EXPECT_TRUE(tree.validate_tree());
}

// ================== Validation Tests =====================

TEST(BVHTreeTest, ValidAfterBuildScattered) {
  std::vector<Triangle> input;
  for (int i = 0; i < 10; ++i) {
    float offset = static_cast<float>(i) * 10.0f;
    input.emplace_back(Vector3D(offset, 0, 0), Vector3D(offset + 1, 0, 0),
        Vector3D(offset, 1, 0));
  }
  acceleration::BVHTree<Triangle> tree(input);
  EXPECT_TRUE(tree.validate_tree());
}

// ================ Intersection Tests =====================

TEST(BVHTreeTest, NoIntersections) {
  std::vector<Triangle> input = {
      Triangle({0, 0, 0}, {1, 0, 0}, {0, 1, 0}),
      Triangle({5, 0, 0}, {6, 0, 0}, {5, 1, 0}),
      Triangle({10, 0, 0}, {11, 0, 0}, {10, 1, 0}),
  };
  acceleration::BVHTree<Triangle> tree(input);
  std::vector<bool>               result = tree.get_intersections();

  for (size_t i = 0; i < result.size(); ++i) {
    EXPECT_FALSE(result[i]) << "Triangle " << i << " should not intersect";
  }
}

TEST(BVHTreeTest, TwoIntersecting) {
  std::vector<Triangle> input = {
      Triangle({0, 0, 0}, {2, 0, 0}, {0, 2, 0}),
      Triangle({1, 0, 0}, {3, 0, 0}, {1, 2, 0}),
      Triangle({100, 0, 0}, {101, 0, 0}, {100, 1, 0}),
  };
  acceleration::BVHTree<Triangle> tree(input);
  std::vector<bool>               result = tree.get_intersections();

  EXPECT_TRUE(result[0]);
  EXPECT_TRUE(result[1]);
  EXPECT_FALSE(result[2]);
}

TEST(BVHTreeTest, AllIntersecting) {
  std::vector<Triangle> input = {
      Triangle({0, 0, 0}, {3, 0, 0}, {0, 3, 0}),
      Triangle({1, 0, 0}, {4, 0, 0}, {1, 3, 0}),
      Triangle({0.5, 0, 0}, {3.5, 0, 0}, {0.5, 3, 0}),
  };
  acceleration::BVHTree<Triangle> tree(input);
  std::vector<bool>               result = tree.get_intersections();

  for (size_t i = 0; i < result.size(); ++i) {
    EXPECT_TRUE(result[i]) << "Triangle " << i << " should intersect";
  }
}

TEST(BVHTreeTest, MatchesBruteForce) {
  std::vector<Triangle> input;

  // Группа 1: пересекающиеся (копланарные с перекрытием)
  input.emplace_back(Vector3D(0, 0, 0), Vector3D(2, 0, 0), Vector3D(0, 2, 0));
  input.emplace_back(Vector3D(1, 0, 0), Vector3D(3, 0, 0), Vector3D(1, 2, 0));
  input.emplace_back(
      Vector3D(0.5, 0.5, 0), Vector3D(2.5, 0.5, 0), Vector3D(0.5, 2.5, 0));

  // Группа 2: далёкие, не пересекающиеся
  for (int i = 0; i < 7; ++i) {
    float offset = 100.0f + static_cast<float>(i) * 10.0f;
    input.emplace_back(Vector3D(offset, 0, 0), Vector3D(offset + 1, 0, 0),
        Vector3D(offset, 1, 0));
  }

  // Группа 3: пара пересекающихся в 3D
  input.emplace_back(
      Vector3D(50, 0, 0), Vector3D(52, 0, 0), Vector3D(50, 2, 0));
  input.emplace_back(
      Vector3D(51, -1, -1), Vector3D(51, 1, 1), Vector3D(51, 1, -1));

  // Группа 4: ещё далёкие
  for (int i = 0; i < 5; ++i) {
    float offset = -100.0f - static_cast<float>(i) * 10.0f;
    input.emplace_back(Vector3D(offset, 0, 0), Vector3D(offset + 1, 0, 0),
        Vector3D(offset, 1, 0));
  }

  std::vector<bool> expected = brute_force_intersections(input);

  acceleration::BVHTree<Triangle> tree(input);
  EXPECT_TRUE(tree.validate_tree());

  std::vector<bool> result = tree.get_intersections();
  ASSERT_EQ(result.size(), expected.size());

  for (size_t i = 0; i < result.size(); ++i) {
    EXPECT_EQ(result[i], expected[i])
        << "Mismatch at triangle " << i << ": BVH=" << result[i]
        << " brute=" << expected[i];
  }
}
