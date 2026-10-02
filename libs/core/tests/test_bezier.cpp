#include <gtest/gtest.h>

#include "drape/core/bezier.hpp"

using drape::ArcLengthTable;
using drape::CubicBezier;
using drape::Vec2d;

TEST(Bezier, LineLengthIsExact) {
  EXPECT_NEAR(drape::length(CubicBezier::line({0, 0}, {3, 4})), 5.0, 1e-12);
}

TEST(Bezier, QuarterCircleLength) {
  const double k = 0.5522847498;
  CubicBezier q{{1, 0}, {1, k}, {k, 1}, {0, 1}};
  EXPECT_NEAR(drape::length(q), drape::kPi / 2, 1e-3);
}

TEST(Bezier, TAtLengthInverts) {
  const double k = 0.5522847498;
  CubicBezier q{{1, 0}, {1, k}, {k, 1}, {0, 1}};
  ArcLengthTable table(q);
  const double total = table.length();
  for (double frac : {0.0, 0.25, 0.5, 1.0}) {
    const double s = frac * total;
    const double t = table.tAtLength(s);
    // Length of the sub-curve [0, t], measured by fine chord summation.
    double acc = 0;
    Vec2d prev = q.point(0);
    const int n = 20000;
    for (int i = 1; i <= n; ++i) {
      Vec2d p = q.point(t * i / n);
      acc += (p - prev).norm();
      prev = p;
    }
    EXPECT_NEAR(acc, s, 1e-6) << "frac " << frac;
  }
}

TEST(Bezier, ReversedAndMirrored) {
  CubicBezier c{{0, 0}, {1, 2}, {3, 2}, {4, 0}};
  EXPECT_TRUE(c.reversed().point(0.3).isApprox(c.point(0.7)));
  Vec2d p = c.point(0.3);
  EXPECT_TRUE(c.mirroredX().point(0.3).isApprox(Vec2d(-p.x(), p.y())));
}
