#include <gtest/gtest.h>

#include <stdexcept>

#include "drape/core/body_regions.hpp"
#include "drape/core/pattern.hpp"
#include "drape/core/test_fabrics.hpp"
#include "sample_project.hpp"

TEST(Pattern, SegmentEdgeSpansNamedSegments) {
  auto sq = drape::testing::squarePiece("sq", 1.0);
  drape::EdgeRef e = drape::segmentEdge(sq, "right", "top", false);
  EXPECT_EQ(e.piece, "sq");
  EXPECT_DOUBLE_EQ(e.start, 1.0);
  EXPECT_DOUBLE_EQ(e.end, 3.0);
  EXPECT_FALSE(e.reversed);
  EXPECT_TRUE(drape::segmentEdge(sq, "left", "left", true).reversed);
  EXPECT_THROW(drape::segmentEdge(sq, "nope", "top", false), std::invalid_argument);
  EXPECT_THROW(drape::segmentEdge(sq, "top", "right", false), std::invalid_argument);
}

TEST(Pattern, FindPiece) {
  drape::Pattern p;
  p.pieces.push_back(drape::testing::squarePiece("a", 0.1));
  ASSERT_NE(p.findPiece("a"), nullptr);
  EXPECT_EQ(p.findPiece("zz"), nullptr);
}

TEST(Regions, E1IsAxisCrossE0) {
  drape::RegionCylinder c{{0, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.1};
  EXPECT_TRUE(c.e1().isApprox(drape::Vec3d(1, 0, 0)));
}

TEST(TestFabrics, AreSI) {
  EXPECT_DOUBLE_EQ(drape::testdata::testJersey().weight, 0.160);
  EXPECT_DOUBLE_EQ(drape::testdata::testJersey().bendWarp, 2e-6);
  EXPECT_DOUBLE_EQ(drape::testdata::testRib().stretchWeft, 100);
  EXPECT_DOUBLE_EQ(drape::testdata::testStiff().bendWeft, 100e-6);
  EXPECT_DOUBLE_EQ(drape::testdata::testAniso().bendWeft, 40e-6);
  EXPECT_DOUBLE_EQ(drape::testdata::testAniso().bendWarp, 100e-6);
}
