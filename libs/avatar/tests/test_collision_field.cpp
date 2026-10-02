#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

#include "drape/avatar/collision_bake.hpp"
#include "drape/avatar/primitives.hpp"
#include "drape/avatar/test_body.hpp"

using namespace drape;
using namespace drape::avatar;

namespace {

Vec3d randomDirection(std::mt19937& rng) {
  std::normal_distribution<double> n(0, 1);
  Vec3d d(n(rng), n(rng), n(rng));
  return d.normalized();
}

const CollisionField& sphereField() {
  static const CollisionField f = [] {
    std::vector<TriangleMesh> c{icosphere({0, 0, 0}, 0.1, 4)};
    return bakeCollisionField(c);
  }();
  return f;
}

}  // namespace

TEST(CollisionField, SphereDistanceAccuracy) {
  const auto& f = sphereField();
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> d(-0.015, 0.015);
  for (int i = 0; i < 2000; ++i) {
    const Vec3d p = (0.1 + d(rng)) * randomDirection(rng);
    EXPECT_NEAR(f.sample(p), p.norm() - 0.1, 0.001) << p.transpose();
  }
}

TEST(CollisionField, SignInsideOutside) {
  const auto& f = sphereField();
  // Grid values are stored as float, so in-grid samples match the band to float precision.
  EXPECT_NEAR(f.sample({0, 0, 0}), -0.02, 1e-6);
  EXPECT_NEAR(f.sample({0.15, 0, 0}), 0.02, 1e-6);
}

TEST(CollisionField, GradientPointsOutward) {
  const auto& f = sphereField();
  std::mt19937 rng(11);
  for (int i = 0; i < 200; ++i) {
    const Vec3d dir = randomDirection(rng);
    const Vec3d g = f.gradient(0.105 * dir);
    ASSERT_NEAR(g.norm(), 1.0, 1e-9);
    EXPECT_LE(std::acos(std::clamp(g.dot(dir), -1.0, 1.0)) * 180 / kPi, 5.0);
  }
}

TEST(CollisionField, UnionIsMin) {
  std::vector<TriangleMesh> c{icosphere({0.08, 0, 0}, 0.1, 4), icosphere({-0.08, 0, 0}, 0.1, 4)};
  const CollisionField f = bakeCollisionField(c);
  EXPECT_LT(f.sample({0, 0, 0}), 0.0);
  // Away from the crease the union equals the nearer sphere's distance.
  const Vec3d near1(0.08, 0, 0.115);
  EXPECT_NEAR(f.sample(near1), 0.015, 0.001);
  // On the crease (x = 0) min() has a ridge that trilinear sampling rounds down: positive and conservative.
  const Vec3d crease(0, 0, 0.085);
  const double expected = (crease - Vec3d(0.08, 0, 0)).norm() - 0.1;
  EXPECT_GT(f.sample(crease), 0.0);
  EXPECT_LE(f.sample(crease), expected + 1e-6);
  EXPECT_NEAR(f.sample(crease), expected, 0.002);
}

TEST(CollisionField, OutsideGridIsFarOutside) {
  const auto& f = sphereField();
  EXPECT_DOUBLE_EQ(f.sample({10, 10, 10}), f.band);
  EXPECT_TRUE(f.gradient({10, 10, 10}).isZero());
  const Vec3d lo = f.origin;
  const Vec3d hi = f.origin + f.cellSize * Vec3d(f.dims[0] - 1, f.dims[1] - 1, f.dims[2] - 1);
  for (int axis = 0; axis < 3; ++axis) {
    Vec3d below = 0.5 * (lo + hi), above = 0.5 * (lo + hi);
    below[axis] = lo[axis] - 1e-6;
    above[axis] = hi[axis] + 1e-6;
    EXPECT_DOUBLE_EQ(f.sample(below), f.band);
    EXPECT_DOUBLE_EQ(f.sample(above), f.band);
  }
}

TEST(CollisionField, RejectsOpenMesh) {
  TriangleMesh m = icosphere({0, 0, 0}, 0.1, 2);
  m.triangles.pop_back();
  std::vector<TriangleMesh> c{m};
  EXPECT_THROW(bakeCollisionField(c), std::invalid_argument);
}
