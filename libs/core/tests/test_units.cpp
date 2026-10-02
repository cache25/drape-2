#include <gtest/gtest.h>

#include "drape/core/math.hpp"
#include "drape/core/units.hpp"
#include "drape/core/version.hpp"

TEST(Units, Conversions) {
  EXPECT_DOUBLE_EQ(drape::units::inToM(1.0), 0.0254);
  EXPECT_DOUBLE_EQ(drape::units::gsmToKgPerM2(160.0), 0.160);
  EXPECT_DOUBLE_EQ(drape::units::microNewtonMeterToNewtonMeter(40.0), 40e-6);
  EXPECT_NEAR(drape::units::ozPerSqYdToGsm(13.5), 457.727, 1e-3);
}

TEST(Version, IsDev) { EXPECT_STREQ(drape::appVersion(), "0.1.0-dev"); }

TEST(Math, EigenAliases) {
  drape::Vec3d v(1, 2, 2);
  EXPECT_DOUBLE_EQ(v.norm(), 3.0);
}
