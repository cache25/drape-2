#include <gtest/gtest.h>

#include "drape/core/test_fabrics.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "sim_model.hpp"
#include "sim_test_utils.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

namespace {

// The StripStretchWarp strip (0.2 x 0.05 m, pinned at x = 0, pulled at x = 0.2), without gravity.
SimScene stripScene() {
  SimScene scene;
  scene.settings.gravity = Vec3d::Zero();
  scene.garments.push_back(
      flatGarment("strip", garment::rectanglePiece("strip", 0.200, 0.050, {1, 0}), 0.005, testdata::testJersey()));
  const SimMesh& m = scene.garments[0].mesh;
  for (auto v : verticesWhere(m, [&](std::uint32_t i) { return m.rest[i].x() <= 1e-9; }))
    scene.pins.push_back({v, scene.garments[0].initialPositions[v]});
  for (auto v : verticesWhere(m, [&](std::uint32_t i) { return m.rest[i].x() >= 0.2 - 1e-9; }))
    scene.forces.push_back({v, Vec3d(0.05, 0, 0)});
  return scene;
}

}  // namespace

TEST(SimModel, BuildMatchesCpuSolverCounts) {
  const SimScene scene = stripScene();
  CpuSolver cpu;
  cpu.build(scene);
  const SimModel m = SimModel::build(scene);
  EXPECT_EQ(m.x.size(), cpu.particleCount());
  EXPECT_EQ(m.rowStart.size(), m.x.size() + 1);
  ASSERT_EQ(m.diagSlot.size(), m.x.size());
  for (std::uint32_t i = 0; i < m.diagSlot.size(); ++i) EXPECT_EQ(m.colIndex[m.diagSlot[i]], i);
}

TEST(SimModel, SnapshotRoundTrip) {
  SimModel m = SimModel::build(stripScene());
  m.v[3] = Vec3f(0.1f, -0.2f, 0.3f);
  m.contactForce[5] = 0.25;
  const auto x = m.x;
  const auto v = m.v;
  const auto f = m.contactForce;
  m.restore(m.snapshot());
  EXPECT_EQ(m.x, x);
  EXPECT_EQ(m.v, v);
  EXPECT_EQ(m.contactForce, f);
}

TEST(SimModel, StatsMatchCpuSolver) {
  const SimScene scene = stripScene();
  CpuSolver cpu;
  cpu.build(scene);
  for (int f = 0; f < 10; ++f) cpu.step({1.0 / 60.0, 1, 2});
  SimModel m = SimModel::build(scene);
  m.restore(cpu.snapshot());
  const SimStats a = m.stats();
  const SimStats b = cpu.stats();
  EXPECT_EQ(a.hasNonFinite, b.hasNonFinite);
  EXPECT_EQ(a.kineticEnergy, b.kineticEnergy);
  EXPECT_EQ(a.rmsSpeed, b.rmsSpeed);
  EXPECT_EQ(a.maxSeamGap, b.maxSeamGap);
  EXPECT_EQ(a.maxBodyPenetration, b.maxBodyPenetration);
}
