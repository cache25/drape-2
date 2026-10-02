#include <gtest/gtest.h>

#include "drape/core/test_fabrics.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "sim_test_utils.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

namespace {

// Two w x hgt squares in the XZ plane, sq2 shifted by `gap` beyond sq1's right edge, sewn sq1.right <-> sq2.left.
SimGarment sewnPair(double w, double hgt, double gap, double h, const FabricPhysical& f) {
  Pattern p;
  p.pieces.push_back(garment::rectanglePiece("sq1", w, hgt));
  p.pieces.push_back(garment::rectanglePiece("sq2", w, hgt));
  p.seams.push_back(Seam{"s1", segmentEdge(p.pieces[0], "right", "right", false),
                         segmentEdge(p.pieces[1], "left", "left", true), 0, SeamKind::Normal});
  SimGarment g;
  g.id = "pair";
  g.mesh = garment::buildSimMesh(p, h);
  for (std::size_t v = 0; v < g.mesh.rest.size(); ++v) {
    const Vec2d r = g.mesh.rest[v];
    const double shift = g.mesh.vertexPiece[v] == 1 ? w + gap : 0.0;
    g.initialPositions.push_back(Vec3d(r.x() + shift, 0, r.y()));
  }
  g.pieceFabric = {f, f};
  return g;
}

}  // namespace

TEST(Seams, GapStatAfterBuild) {
  SimScene scene;
  scene.garments.push_back(sewnPair(0.05, 0.05, 0.1, 0.01, testdata::testJersey()));
  CpuSolver s;
  s.build(scene);
  EXPECT_NEAR(s.stats().maxSeamGap, 0.1, 1e-6);
}

TEST(Seams, CloseAtClosingSpeed) {
  SimScene scene;
  scene.settings.gravity = Vec3d::Zero();
  scene.garments.push_back(sewnPair(0.05, 0.05, 0.1, 0.01, testdata::testJersey()));
  CpuSolver s;
  s.build(scene);
  s.setPhase("pair", SimPhase::Assembling);
  for (int f = 0; f < 12; ++f) s.step({1.0 / 60.0, 10});
  EXPECT_GE(s.stats().maxSeamGap, 0.047);
  EXPECT_LE(s.stats().maxSeamGap, 0.053);
  for (int f = 12; f < 30; ++f) s.step({1.0 / 60.0, 10});
  EXPECT_LT(s.stats().maxSeamGap, 0.001);
}

TEST(Seams, HoldWhenSettling) {
  SimScene scene;
  scene.garments.push_back(sewnPair(0.05, 0.05, 0.1, 0.01, testdata::testJersey()));
  const SimMesh& m = scene.garments[0].mesh;
  for (auto v : verticesWhere(m, [&](std::uint32_t i) { return m.vertexPiece[i] == 0 && m.rest[i].x() <= 1e-9; })) {
    scene.pins.push_back({v, scene.garments[0].initialPositions[v]});
  }
  CpuSolver s;
  s.build(scene);
  s.setPhase("pair", SimPhase::Assembling);
  for (int f = 0; f < 30; ++f) s.step({1.0 / 60.0, 10});
  ASSERT_LT(s.stats().maxSeamGap, 0.001);
  s.setPhase("pair", SimPhase::Settling);
  for (int f = 0; f < 120; ++f) s.step({1.0 / 60.0, 2, 10});
  EXPECT_LT(s.stats().maxSeamGap, 0.001);
  EXPECT_FALSE(s.stats().hasNonFinite);
}
