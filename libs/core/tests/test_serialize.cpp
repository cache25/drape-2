#include <gtest/gtest.h>

#include "drape/core/serialize.hpp"
#include "sample_project.hpp"

using drape::testing::makeSampleProject;
using nlohmann::json;

TEST(Serialize, ProjectRoundTrip) {
  auto p = makeSampleProject();
  json j = p;
  EXPECT_EQ(json::parse(j.dump()).get<drape::Project>(), p);
}

TEST(Serialize, OptionalFieldsOmitted) {
  auto p = makeSampleProject();
  json piece = p.garments[0].pattern.pieces[1];  // "b" has no layer
  EXPECT_FALSE(piece.contains("layer"));
  EXPECT_FALSE(piece.contains("elastic"));
  json withLayer = p.garments[0].pattern.pieces[0];
  EXPECT_TRUE(withLayer.contains("layer"));
}

TEST(Serialize, EnumsAsStrings) {
  EXPECT_EQ(json(drape::BaseBody::Female), "female");
  EXPECT_EQ(json(drape::BodyRegion::ArmLeft), "armLeft");
  EXPECT_EQ(json(drape::SeamKind::ClosedZipper), "closedZipper");
  EXPECT_EQ(json(drape::LightingPreset::StudioSoft), "studioSoft");
}

TEST(Serialize, EnumKeyedMapsAreObjects) {
  auto p = makeSampleProject();
  json avatar = p.avatar;
  ASSERT_TRUE(avatar["targets"].is_object());
  EXPECT_DOUBLE_EQ(avatar["targets"]["height"].get<double>(), 1.80);
}

TEST(Serialize, MissingKeyThrows) {
  auto p = makeSampleProject();
  json pattern = p.garments[0].pattern;
  pattern.erase("seams");
  EXPECT_THROW(pattern.get<drape::Pattern>(), json::exception);
}
