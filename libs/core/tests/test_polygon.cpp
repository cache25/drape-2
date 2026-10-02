#include <gtest/gtest.h>

#include <vector>

#include "drape/core/polygon.hpp"

using drape::Vec2d;

namespace {
const std::vector<Vec2d> kSquare{{0, 0}, {1, 0}, {1, 1}, {0, 1}};
}

TEST(Polygon, SignedAreaOrientation) {
  EXPECT_DOUBLE_EQ(drape::signedArea(kSquare), 1.0);
  std::vector<Vec2d> cw(kSquare.rbegin(), kSquare.rend());
  EXPECT_DOUBLE_EQ(drape::signedArea(cw), -1.0);
}

TEST(Polygon, PointInPolygon) {
  EXPECT_TRUE(drape::pointInPolygon({0.5, 0.5}, kSquare));
  EXPECT_FALSE(drape::pointInPolygon({1.5, 0.5}, kSquare));
}

TEST(Polygon, DistancePointSegment) {
  EXPECT_DOUBLE_EQ(drape::distancePointSegment({0, 1}, {-1, 0}, {1, 0}), 1.0);
  EXPECT_DOUBLE_EQ(drape::distancePointSegment({2, 0}, {-1, 0}, {1, 0}), 1.0);
}

TEST(Polygon, BowtieIsNotSimple) {
  std::vector<Vec2d> bowtie{{0, 0}, {1, 1}, {1, 0}, {0, 1}};
  EXPECT_FALSE(drape::isSimplePolygon(bowtie));
  EXPECT_TRUE(drape::isSimplePolygon(kSquare));
}

TEST(Polygon, SegmentsIntersect) {
  EXPECT_TRUE(drape::segmentsIntersect({0, 0}, {1, 1}, {0, 1}, {1, 0}));
  EXPECT_TRUE(drape::segmentsIntersect({0, 0}, {1, 0}, {1, 0}, {1, 1}));  // touching
  EXPECT_FALSE(drape::segmentsIntersect({0, 0}, {1, 0}, {0, 1}, {1, 1}));
}
