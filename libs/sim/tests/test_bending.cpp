#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>

#include "bending.hpp"
#include "drape/core/test_fabrics.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "sim_test_utils.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

TEST(Bending, Coefficients) {
  auto iso = bendingCoefficients(100e-6, 100e-6);
  EXPECT_NEAR(iso.p, 100e-6, 1e-18);
  EXPECT_NEAR(iso.q, 100e-6, 1e-18);
  auto an = bendingCoefficients(100e-6, 40e-6);
  EXPECT_NEAR(an.p, 130e-6, 1e-15);
  EXPECT_NEAR(an.q, 40e-6, 1e-15);
  auto clamped = bendingCoefficients(5e-6, 100e-6);
  EXPECT_NEAR(clamped.p, 0.25e-6, 1e-15);
  // Edge along the grain bends the weft yarns -> q; edge across the grain -> p.
  EXPECT_NEAR(edgeBendStiffness(an, {1, 0}, {1, 0}), an.q, 1e-15);
  EXPECT_NEAR(edgeBendStiffness(an, {0, 1}, {1, 0}), an.p, 1e-15);
}

TEST(Bending, GradientMatchesFiniteDifference) {
  using V3 = Eigen::Vector3d;
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> j(-0.2, 0.2);
  for (int trial = 0; trial < 20; ++trial) {
    std::array<V3, 4> x{V3(0.5 + j(rng), 1.0 + j(rng), 0.3 + j(rng)), V3(0.5 + j(rng), -1.0 + j(rng), -0.3 + j(rng)),
                        V3(0, 0, 0), V3(1 + j(rng), j(rng), j(rng))};
    double theta;
    std::array<V3, 4> g;
    evalDihedral<double>(x, theta, g);
    for (int k = 0; k < 4; ++k) {
      for (int a = 0; a < 3; ++a) {
        const double h = 1e-6;
        auto xp = x, xm = x;
        xp[k][a] += h;
        xm[k][a] -= h;
        double tp, tm;
        std::array<V3, 4> unused;
        evalDihedral<double>(xp, tp, unused);
        evalDihedral<double>(xm, tm, unused);
        EXPECT_NEAR(g[k][a], (tp - tm) / (2 * h), 1e-6) << "k " << k << " a " << a;
      }
    }
  }
}

TEST(Bending, FlatSheetStaysFlat) {
  SimScene scene;
  scene.settings.gravity = Vec3d::Zero();
  scene.garments.push_back(flatGarment("sheet", garment::rectanglePiece("sheet", 0.2, 0.2), 0.01, testdata::testStiff()));
  CpuSolver s;
  s.build(scene);
  for (int f = 0; f < 60; ++f) s.step({1.0 / 60.0, 2, 10});
  for (const auto& p : positions(s)) EXPECT_LT(std::abs(p.y()), 1e-6);
}

