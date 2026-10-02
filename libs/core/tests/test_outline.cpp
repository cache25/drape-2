#include <gtest/gtest.h>

#include "drape/core/outline.hpp"

using drape::CubicBezier;
using drape::Outline;
using drape::Vec2d;

namespace {
Outline unitSquare() {
  Outline sq;
  sq.add("bottom", CubicBezier::line({0, 0}, {1, 0}));
  sq.add("right", CubicBezier::line({1, 0}, {1, 1}));
  sq.add("top", CubicBezier::line({1, 1}, {0, 1}));
  sq.add("left", CubicBezier::line({0, 1}, {0, 0}));
  return sq;
}
}  // namespace

TEST(Outline, SquarePerimeterAndPoints) {
  Outline sq = unitSquare();
  EXPECT_DOUBLE_EQ(sq.perimeter(), 4.0);
  EXPECT_TRUE(sq.pointAt(1.5).isApprox(Vec2d(1, 0.5)));
  EXPECT_TRUE(sq.tangentAt(1.5).isApprox(Vec2d(0, 1)));
  EXPECT_TRUE(sq.pointAt(4.0).isApprox(Vec2d(0, 0)));
  EXPECT_DOUBLE_EQ(sq.segmentStart(2), 2.0);
  EXPECT_DOUBLE_EQ(sq.segmentEnd(2), 3.0);
  EXPECT_EQ(sq.find("top"), 2u);
  EXPECT_FALSE(sq.find("nope").has_value());
  EXPECT_TRUE(sq.isClosed());
}

TEST(Outline, OpenOutlineDetected) {
  Outline o;
  o.add("bottom", CubicBezier::line({0, 0}, {1, 0}));
  o.add("right", CubicBezier::line({1, 0}, {1, 1}));
  o.add("top", CubicBezier::line({1, 1}, {0, 1}));
  EXPECT_FALSE(o.isClosed());
}

TEST(Outline, PolylineSpacing) {
  Outline sq = unitSquare();
  auto poly = sq.polyline(0.3);
  ASSERT_GE(poly.size(), 4u);
  for (Vec2d corner : {Vec2d(0, 0), Vec2d(1, 0), Vec2d(1, 1), Vec2d(0, 1)}) {
    bool found = false;
    for (const auto& p : poly) found = found || (p - corner).norm() < 1e-12;
    EXPECT_TRUE(found) << corner.transpose();
  }
  for (std::size_t i = 0; i < poly.size(); ++i) {
    const auto& a = poly[i];
    const auto& b = poly[(i + 1) % poly.size()];
    EXPECT_LE((b - a).norm(), 0.3 + 1e-12);
  }
  EXPECT_GT((poly.front() - poly.back()).norm(), 1e-12);
}
