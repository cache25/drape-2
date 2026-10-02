#include <gtest/gtest.h>

#include <random>

#include "stretch.hpp"

using V3 = Eigen::Vector3d;
using M2 = Eigen::Matrix2d;

TEST(Stretch, GradientsMatchFiniteDifference) {
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> jitter(-0.3, 0.3);
  // Rest triangle in (u, v) and its inverse edge matrix.
  const Eigen::Vector2d r0(0, 0), r1(1, 0.1), r2(0.3, 0.9);
  M2 dm;
  dm << (r1 - r0).x(), (r2 - r0).x(), (r1 - r0).y(), (r2 - r0).y();
  const M2 dmInv = dm.inverse();
  for (int trial = 0; trial < 20; ++trial) {
    std::array<V3, 3> x{V3(jitter(rng), jitter(rng), jitter(rng)), V3(1.2 + jitter(rng), 0.1, jitter(rng)),
                        V3(0.3, 0.9 + jitter(rng), 0.2 + jitter(rng))};
    for (int which = 0; which < 3; ++which) {
      double C;
      std::array<V3, 3> g;
      drape::sim::evalStretch<double>(x, dmInv, which, C, g);
      for (int k = 0; k < 3; ++k) {
        for (int a = 0; a < 3; ++a) {
          const double h = 1e-6;
          auto xp = x, xm = x;
          xp[k][a] += h;
          xm[k][a] -= h;
          double cp, cm;
          std::array<V3, 3> unused;
          drape::sim::evalStretch<double>(xp, dmInv, which, cp, unused);
          drape::sim::evalStretch<double>(xm, dmInv, which, cm, unused);
          EXPECT_NEAR(g[k][a], (cp - cm) / (2 * h), 1e-6) << "which " << which << " k " << k << " a " << a;
        }
      }
    }
  }
}
