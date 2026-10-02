#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "drape/core/test_fabrics.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "sim_test_utils.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

namespace {

FabricPhysical withDamping(FabricPhysical f, double d) {
  f.damping = d;
  return f;
}

// Strip pinned at x = 0 and pulled along +X with 0.5 N total at x = 0.2. Returns strain over rest x in [0.04, 0.16].
struct StripResult {
  double strain;
  int frames;
};

SimScene stripScene(const Vec2d& grain, const FabricPhysical& f) {
  SimScene scene;
  scene.settings.gravity = Vec3d::Zero();
  scene.garments.push_back(flatGarment("strip", garment::rectanglePiece("strip", 0.200, 0.050, grain), 0.005, f));
  const SimMesh& m = scene.garments[0].mesh;
  for (auto v : verticesWhere(m, [&](std::uint32_t i) { return m.rest[i].x() <= 1e-9; })) {
    scene.pins.push_back({v, scene.garments[0].initialPositions[v]});
  }
  auto loaded = verticesWhere(m, [&](std::uint32_t i) { return m.rest[i].x() >= 0.2 - 1e-9; });
  std::sort(loaded.begin(), loaded.end(), [&](auto a, auto b) { return m.rest[a].y() < m.rest[b].y(); });
  for (std::size_t i = 0; i < loaded.size(); ++i) {
    const double lo = i == 0 ? m.rest[loaded[i]].y() : 0.5 * (m.rest[loaded[i]].y() + m.rest[loaded[i - 1]].y());
    const double hi = i + 1 == loaded.size() ? m.rest[loaded[i]].y()
                                             : 0.5 * (m.rest[loaded[i]].y() + m.rest[loaded[i + 1]].y());
    scene.forces.push_back({loaded[i], Vec3d(0.5 * (hi - lo) / 0.05, 0, 0)});
  }
  return scene;
}

double measuredStrain(const Solver& s, const SimMesh& m) {
  const auto x = positions(s);
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  int n = 0;
  for (std::uint32_t v = 0; v < m.rest.size(); ++v) {
    const double r = m.rest[v].x();
    if (r < 0.04 || r > 0.16) continue;
    sx += r;
    sy += x[v].x();
    sxx += r * r;
    sxy += r * x[v].x();
    ++n;
  }
  const double slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
  return slope - 1.0;
}

}  // namespace

TEST(CpuSolver, SubstepsForQuality) {
  EXPECT_EQ(substepsFor(SimQuality::Draft), 2);
  EXPECT_EQ(substepsFor(SimQuality::Standard), 2);
  EXPECT_EQ(substepsFor(SimQuality::Fine), 4);
  EXPECT_EQ(iterationsFor(SimQuality::Draft), 5);
  EXPECT_EQ(iterationsFor(SimQuality::Standard), 10);
  EXPECT_EQ(iterationsFor(SimQuality::Fine), 10);
}

TEST(CpuSolver, FreeFall) {
  SimScene scene;
  scene.garments.push_back(
      flatGarment("sheet", garment::rectanglePiece("sheet", 0.1, 0.1), 0.02, withDamping(testdata::testJersey(), 0)));
  CpuSolver s;
  s.build(scene);
  const auto before = positions(s);
  for (int f = 0; f < 30; ++f) s.step({1.0 / 60.0, 10});
  const auto after = positions(s);
  // VBD's per-vertex Gauss-Seidel leaves a tiny in-plane drift during free fall (second-order stretch
  // coupling while neighbours update in turn); 0.1 mm over a 1.2 m fall is physically irrelevant.
  for (std::size_t i = 0; i < before.size(); ++i) {
    EXPECT_NEAR((after[i].y() - before[i].y()) / -1.22625, 1.0, 0.005);
    EXPECT_NEAR(after[i].x(), before[i].x(), 1e-4);
    EXPECT_NEAR(after[i].z(), before[i].z(), 1e-4);
  }
}

TEST(CpuSolver, AssemblingDisablesGravity) {
  SimScene scene;
  scene.garments.push_back(flatGarment("sheet", garment::rectanglePiece("sheet", 0.1, 0.1), 0.02, testdata::testJersey()));
  CpuSolver s;
  s.build(scene);
  s.setPhase("sheet", SimPhase::Assembling);
  const auto before = positions(s);
  for (int f = 0; f < 30; ++f) s.step({1.0 / 60.0, 10});
  const auto after = positions(s);
  for (std::size_t i = 0; i < before.size(); ++i) EXPECT_LT((after[i] - before[i]).norm(), 1e-6);
}

TEST(CpuSolver, PinsHold) {
  SimScene scene;
  scene.garments.push_back(flatGarment("sheet", garment::rectanglePiece("sheet", 0.1, 0.1), 0.02, testdata::testJersey()));
  scene.pins.push_back({0, scene.garments[0].initialPositions[0]});
  CpuSolver s;
  s.build(scene);
  for (int f = 0; f < 60; ++f) s.step({1.0 / 60.0, 10});
  EXPECT_LT((positions(s)[0].cast<double>() - scene.pins[0].position).norm(), 1e-7);
}

TEST(CpuSolver, StripStretchWarp) {
  auto scene = stripScene({1, 0}, withDamping(testdata::testJersey(), 1.0));
  CpuSolver s;
  s.build(scene);
  ASSERT_GT(runUntilStill(s, {1.0 / 60.0, 2, 10}, 1e-5, 60, 30.0), 0);
  EXPECT_NEAR(measuredStrain(s, scene.garments[0].mesh), 0.025, 0.05 * 0.025);
}

TEST(CpuSolver, StripStretchWeft) {
  auto scene = stripScene({0, 1}, withDamping(testdata::testJersey(), 1.0));
  CpuSolver s;
  s.build(scene);
  ASSERT_GT(runUntilStill(s, {1.0 / 60.0, 2, 10}, 1e-5, 60, 30.0), 0);
  EXPECT_NEAR(measuredStrain(s, scene.garments[0].mesh), 0.05, 0.05 * 0.05);
}

TEST(CpuSolver, UpdateFabricChangesStiffness) {
  auto f = withDamping(testdata::testJersey(), 1.0);
  auto scene = stripScene({1, 0}, f);
  CpuSolver s;
  s.build(scene);
  ASSERT_GT(runUntilStill(s, {1.0 / 60.0, 2, 10}, 1e-5, 60, 30.0), 0);
  f.stretchWarp = 800;
  s.updateFabric("strip", {f});
  ASSERT_GT(runUntilStill(s, {1.0 / 60.0, 2, 10}, 1e-5, 60, 30.0), 0);
  EXPECT_NEAR(measuredStrain(s, scene.garments[0].mesh), 0.0125, 0.05 * 0.0125);
}

TEST(CpuSolver, SnapshotRestoreIsDeterministic) {
  SimScene scene;
  scene.garments.push_back(flatGarment("sheet", garment::rectanglePiece("sheet", 0.2, 0.2), 0.02, testdata::testJersey()));
  scene.pins.push_back({0, scene.garments[0].initialPositions[0]});
  CpuSolver s;
  s.build(scene);
  for (int f = 0; f < 10; ++f) s.step({1.0 / 60.0, 10});
  const Snapshot snap = s.snapshot();
  for (int f = 0; f < 20; ++f) s.step({1.0 / 60.0, 10});
  const auto first = positions(s);
  s.restore(snap);
  for (int f = 0; f < 20; ++f) s.step({1.0 / 60.0, 10});
  const auto second = positions(s);
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) EXPECT_EQ(first[i], second[i]);
}

TEST(CpuSolver, ThinPieceSteps) {
  SimScene scene;
  scene.garments.push_back(flatGarment("thin", garment::rectanglePiece("thin", 0.5, 0.02), 0.02, testdata::testJersey()));
  CpuSolver s;
  s.build(scene);
  for (int f = 0; f < 60; ++f) s.step({1.0 / 60.0, 10});
  EXPECT_FALSE(s.stats().hasNonFinite);
}

TEST(CpuSolver, RangesAndCounts) {
  SimScene scene;
  scene.garments.push_back(flatGarment("a", garment::rectanglePiece("a", 0.1, 0.1), 0.02, testdata::testJersey()));
  scene.garments.push_back(
      flatGarment("b", garment::rectanglePiece("b", 0.2, 0.1), 0.02, testdata::testJersey(), {0.5, 0, 0}));
  CpuSolver s;
  s.build(scene);
  const auto ra = s.range("a"), rb = s.range("b");
  EXPECT_EQ(ra.offset, 0u);
  EXPECT_EQ(ra.count, scene.garments[0].mesh.rest.size());
  EXPECT_EQ(rb.offset, ra.count);
  EXPECT_EQ(rb.count, scene.garments[1].mesh.rest.size());
  EXPECT_EQ(s.particleCount(), ra.count + rb.count);
  EXPECT_THROW(s.range("zzz"), std::out_of_range);
}
