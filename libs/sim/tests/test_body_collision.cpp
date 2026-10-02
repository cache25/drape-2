#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "drape/avatar/collision_bake.hpp"
#include "drape/avatar/primitives.hpp"
#include "drape/core/test_fabrics.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "sim_test_utils.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

namespace {

const CollisionField& sphereField() {
  static const CollisionField f = [] {
    std::vector<TriangleMesh> c{avatar::icosphere({0, 0, 0}, 0.15, 4)};
    return avatar::bakeCollisionField(c);
  }();
  return f;
}

constexpr double kContact = 0.003 + 0.0006 / 2;  // collision offset + half testJersey thickness

SimGarment centredSheet(double size, double h, double height) {
  return flatGarment("sheet", garment::rectanglePiece("sheet", size, size), h, testdata::testJersey(),
                     {-size / 2, height, -size / 2});
}

std::uint32_t nearestTo(const SimMesh& m, const Vec2d& p) {
  std::uint32_t best = 0;
  for (std::uint32_t v = 0; v < m.rest.size(); ++v)
    if ((m.rest[v] - p).norm() < (m.rest[best] - p).norm()) best = v;
  return best;
}

}  // namespace

TEST(BodyCollision, ParticlesRestOnSphere) {
  SimScene scene;
  scene.body = &sphereField();
  scene.garments.push_back(centredSheet(0.004, 0.004, 0.15 + 0.05));
  CpuSolver s;
  s.build(scene);
  for (int f = 0; f < 120; ++f) s.step({1.0 / 60.0, 1, 2});
  for (const auto& p : positions(s)) {
    const double phi = sphereField().sample(p.cast<double>());
    EXPECT_NEAR(phi, kContact, 0.0005);
  }
}

// Without self-collision (Phase D) the hanging folds pass through each other and keep drifting, so this
// checks the drape after 5 s instead of waiting for stillness.
TEST(BodyCollision, SheetDrapesOverSphere) {
  SimScene scene;
  scene.body = &sphereField();
  scene.garments.push_back(centredSheet(0.6, 0.01, 0.15 + 0.05));
  CpuSolver s;
  s.build(scene);
  for (int f = 0; f < 300; ++f) s.step({1.0 / 60.0, 1, 2});
  const auto st = s.stats();
  EXPECT_LT(st.rmsSpeed, 0.1);
  EXPECT_FALSE(st.hasNonFinite);
  EXPECT_LE(st.maxBodyPenetration, 0.002);
  const auto centre = nearestTo(scene.garments[0].mesh, {0.3, 0.3});
  EXPECT_NEAR(positions(s)[centre].y(), 0.15 + kContact, 0.002);
}

TEST(BodyCollision, StartsInsideIsResolved) {
  SimScene scene;
  scene.body = &sphereField();
  scene.garments.push_back(centredSheet(0.6, 0.01, 0.15 - 0.01));
  CpuSolver s;
  s.build(scene);
  EXPECT_GT(s.stats().maxBodyPenetration, 0.005);  // really starts inside
  for (int f = 0; f < 60; ++f) s.step({1.0 / 60.0, 1, 2});
  EXPECT_FALSE(s.stats().hasNonFinite);
  EXPECT_LE(s.stats().maxBodyPenetration, 0.002);
}

namespace {

// Displacement along the uphill direction of a 20-degree slope after 120 frames, for a given friction.
double slideOnSlope(double friction) {
  const double a = 20.0 * kPi / 180.0;
  const Mat3d rot = Eigen::AngleAxisd(a, Vec3d(0, 0, 1)).toRotationMatrix();
  const Vec3d up = rot * Vec3d(0, 1, 0);
  const Vec3d uphill = rot * Vec3d(1, 0, 0);
  std::vector<TriangleMesh> c{avatar::box(-0.05 * up, {0.5, 0.05, 0.5}, rot)};
  const CollisionField field = avatar::bakeCollisionField(c);

  auto f = testdata::testJersey();
  f.friction = friction;
  SimGarment g = flatGarment("patch", garment::rectanglePiece("patch", 0.02, 0.02), 0.01, f);
  for (std::size_t v = 0; v < g.mesh.rest.size(); ++v) {
    const Vec2d r = g.mesh.rest[v] - Vec2d(0.01, 0.01);
    g.initialPositions[v] = rot * Vec3d(r.x(), 0, r.y()) + 0.004 * up;
  }
  SimScene scene;
  scene.body = &field;
  scene.garments.push_back(g);
  CpuSolver s;
  s.build(scene);
  auto centroid = [&] {
    Vec3d c0 = Vec3d::Zero();
    for (const auto& p : positions(s)) c0 += p.cast<double>();
    return Vec3d(c0 / static_cast<double>(s.particleCount()));
  };
  const Vec3d start = centroid();
  for (int fr = 0; fr < 120; ++fr) s.step({1.0 / 60.0, 1, 2});
  return (centroid() - start).dot(uphill);
}

}  // namespace

TEST(BodyCollision, FrictionHoldsOnGentleSlope) { EXPECT_LT(std::abs(slideOnSlope(0.5)), 0.002); }

TEST(BodyCollision, SlidesWhenSlippery) { EXPECT_LT(slideOnSlope(0.2), -0.05); }
