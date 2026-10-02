#include <gtest/gtest.h>

#include <stdexcept>

#include "drape/garment/test_tee.hpp"

using namespace drape::garment;

TEST(SpecTable, GradesFromBase) {
  auto t = TestTeeGenerator{}.specTable();
  EXPECT_EQ(t.baseSize, "M");
  EXPECT_NEAR(resolveSpec(t, "XS").at("chest_width"), 0.480, 1e-12);
  EXPECT_NEAR(resolveSpec(t, "M").at("chest_width"), 0.530, 1e-12);
  EXPECT_NEAR(resolveSpec(t, "XXL").at("chest_width"), 0.605, 1e-12);
  EXPECT_NEAR(resolveSpec(t, "XXL").at("back_neck_drop"), 0.020, 1e-12);
}

TEST(SpecTable, OverrideWins) {
  auto t = TestTeeGenerator{}.specTable();
  SpecOverrides o{{"M", {{"chest_width", 0.55}}}};
  EXPECT_NEAR(resolveSpec(t, "M", o).at("chest_width"), 0.55, 1e-12);
  EXPECT_NEAR(resolveSpec(t, "L", o).at("chest_width"), 0.555, 1e-12);
}

TEST(SpecTable, UnknownSizeThrows) {
  auto t = TestTeeGenerator{}.specTable();
  EXPECT_THROW(resolveSpec(t, "XXXL"), std::invalid_argument);
}
