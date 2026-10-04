#include <gtest/gtest.h>

#include "energies.hpp"

using namespace drape::sim;

namespace {

const V3 kNormal = V3(0.2, 1.0, -0.3).normalized();
constexpr double kMuLambda = 0.03;  // N
constexpr double kEps = 1e-4;       // m

TermEval<1> friction(const V3& x) {
  TermEval<1> te;
  te.clear();
  evalFrictionTerm(x, V3(0.1, 0.2, 0.3), kNormal, kMuLambda, kEps, te);
  return te;
}

// A tangential slip of length s from x0 = (0.1, 0.2, 0.3), plus a normal offset the term must ignore.
V3 slipped(double s, double normal = 0) {
  const V3 t = kNormal.cross(V3(1, 0, 0)).normalized();
  return V3(0.1, 0.2, 0.3) + s * t + normal * kNormal;
}

}  // namespace

TEST(Friction, GradientMatchesFiniteDifferences) {
  for (double s : {0.3 * kEps, 0.9 * kEps, 3 * kEps}) {
    const V3 x = slipped(s, 0.002) + V3(1e-5, -2e-5, 0.5e-5);
    const auto te = friction(x);
    for (int a = 0; a < 3; ++a) {
      V3 d = V3::Zero();
      d[a] = 1e-9;
      const double fd = (friction(x + d).energy - friction(x - d).energy) / 2e-9;
      EXPECT_NEAR(te.grad[0][a], fd, 1e-6 * kMuLambda) << "s=" << s << " axis " << a;
    }
  }
}

TEST(Friction, HessianMatchesFiniteDifferences) {
  for (double s : {0.3 * kEps, 0.9 * kEps, 3 * kEps}) {
    const V3 x = slipped(s) + V3(1e-6, -2e-6, 0.5e-6);
    const auto te = friction(x);
    for (int a = 0; a < 3; ++a) {
      V3 d = V3::Zero();
      d[a] = 1e-10;
      const V3 fd = (friction(x + d).grad[0] - friction(x - d).grad[0]) / 2e-10;
      for (int b = 0; b < 3; ++b)
        EXPECT_NEAR(te.hess[0][0](b, a), fd[b], 1e-3 * kMuLambda / kEps) << "s=" << s;
    }
  }
}

TEST(Friction, ForceIsCappedAtMuLambdaWhenSliding) {
  EXPECT_NEAR(friction(slipped(5 * kEps)).grad[0].norm(), kMuLambda, 1e-12);
  EXPECT_LT(friction(slipped(0.5 * kEps)).grad[0].norm(), kMuLambda);
}

TEST(Friction, IgnoresNormalMotionAndIsPositiveSemiDefinite) {
  EXPECT_NEAR(friction(slipped(0, 0.004)).grad[0].norm(), 0.0, 1e-12 * kMuLambda);
  for (double s : {0.0, 0.5 * kEps, 2 * kEps}) {
    const Eigen::SelfAdjointEigenSolver<M3> eig(friction(slipped(s)).hess[0][0]);
    EXPECT_GE(eig.eigenvalues().minCoeff(), -1e-9) << "s=" << s;
  }
}
