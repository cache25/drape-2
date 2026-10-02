#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

#include "drape/core/test_fabrics.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "sim_test_utils.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

namespace {

// Settled tip drop of a cantilever pinned at rest.x <= 0.01; the garment's pieces sit side by side along x.
double tipDrop(SimGarment g, double tipX) {
  SimScene scene;
  const SimMesh& m = g.mesh;
  for (std::uint32_t v = 0; v < m.rest.size(); ++v) {
    if (g.initialPositions[v].x() <= 0.01) scene.pins.push_back({v, g.initialPositions[v]});
  }
  std::vector<std::uint32_t> tip;
  for (std::uint32_t v = 0; v < m.rest.size(); ++v)
    if (g.initialPositions[v].x() >= tipX - 1e-9) tip.push_back(v);
  scene.garments.push_back(std::move(g));
  CpuSolver s;
  s.build(scene);
  EXPECT_GT(runUntilStill(s, {1.0 / 60.0, 8, 20}, 2e-4, 60, 10.0), 0);
  const auto x = positions(s);
  double drop = 0;
  for (auto v : tip) drop -= x[v].y();
  return drop / static_cast<double>(tip.size());
}

}  // namespace

// Slow: cantilever statics (see test_cantilever.cpp).
TEST(Seams, SeamDoesNotHinge) {
  const auto f = testdata::testStiff();
  const double single = tipDrop(flatGarment("one", garment::rectanglePiece("one", 0.06, 0.02), 0.004, f), 0.06);

  Pattern p;
  p.pieces.push_back(garment::rectanglePiece("a", 0.03, 0.02));
  p.pieces.push_back(garment::rectanglePiece("b", 0.03, 0.02));
  p.seams.push_back(Seam{"s", segmentEdge(p.pieces[0], "right", "right", false),
                         segmentEdge(p.pieces[1], "left", "left", true), 0, SeamKind::Normal});
  SimGarment g;
  g.id = "two";
  g.mesh = garment::buildSimMesh(p, 0.004);
  for (std::size_t v = 0; v < g.mesh.rest.size(); ++v) {
    const Vec2d r = g.mesh.rest[v];
    g.initialPositions.push_back(Vec3d(r.x() + (g.mesh.vertexPiece[v] == 1 ? 0.03 : 0.0), 0, r.y()));
  }
  g.pieceFabric = {f, f};
  const double sewn = tipDrop(std::move(g), 0.06);
  std::printf("tip drop single %.4f sewn %.4f\n", single, sewn);
  EXPECT_GE(sewn / single, 0.8);
  EXPECT_LE(sewn / single, 1.25);
}
