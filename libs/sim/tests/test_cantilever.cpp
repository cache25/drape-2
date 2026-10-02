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

// Peirce cantilever (spec 12.2 #2): returns |G - B| / B.
double cantileverError(const FabricPhysical& f, const Vec2d& grain, double B) {
  const double W = f.weight * kGravity;
  const double l = 2.0 * std::cbrt(B / W);
  const double length = l + 0.01;
  SimScene scene;
  scene.garments.push_back(flatGarment("strip", garment::rectanglePiece("strip", length, 0.025, grain), 0.002, f));
  const SimMesh& m = scene.garments[0].mesh;
  std::vector<char> pinned(m.rest.size(), 0);
  for (auto v : verticesWhere(m, [&](std::uint32_t i) { return m.rest[i].x() <= 0.01; })) {
    pinned[v] = 1;
    scene.pins.push_back({v, scene.garments[0].initialPositions[v]});
  }
  // Clamp line: mean rest x of pinned vertices that have an unpinned neighbour.
  std::vector<char> frontier(m.rest.size(), 0);
  for (const auto& t : m.triangles)
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b)
        if (pinned[t[a]] && !pinned[t[b]]) frontier[t[a]] = 1;
  double clampSum = 0;
  int clampCount = 0;
  for (std::size_t v = 0; v < m.rest.size(); ++v)
    if (frontier[v]) {
      clampSum += m.rest[v].x();
      ++clampCount;
    }
  const double xClamp = clampSum / clampCount;
  const auto tipVerts = verticesWhere(m, [&](std::uint32_t i) { return m.rest[i].x() >= length - 1e-9; });

  CpuSolver s;
  s.build(scene);
  EXPECT_GT(runUntilStill(s, {1.0 / 60.0, 1, 2}, 2e-4, 60, 10.0), 0);
  const auto x = positions(s);
  Vec3d tip = Vec3d::Zero();
  double tipRest = 0;
  for (auto v : tipVerts) {
    tip += x[v].cast<double>();
    tipRest += m.rest[v].x();
  }
  tip /= static_cast<double>(tipVerts.size());
  tipRest /= static_cast<double>(tipVerts.size());
  const double theta = std::atan2(-tip.y(), tip.x() - xClamp);
  const double overhang = tipRest - xClamp;
  const double c = overhang * std::cbrt(std::cos(theta / 2) / (8 * std::tan(theta)));
  const double G = W * c * c * c;
  std::printf("cantilever: theta %.2f deg, G/B %.3f\n", theta * 180 / kPi, G / B);
  return std::abs(G - B) / B;
}

}  // namespace

TEST(Bending, CantileverIsotropic) { EXPECT_LE(cantileverError(testdata::testStiff(), {1, 0}, 100e-6), 0.10); }
TEST(Bending, CantileverAnisoWarp) { EXPECT_LE(cantileverError(testdata::testAniso(), {1, 0}, 100e-6), 0.10); }
TEST(Bending, CantileverAnisoWeft) { EXPECT_LE(cantileverError(testdata::testAniso(), {0, 1}, 40e-6), 0.10); }
