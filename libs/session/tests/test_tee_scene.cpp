#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>
#include <string>

#include "drape/garment/spec_table.hpp"
#include "drape/garment/test_tee.hpp"
#include "drape/session/tee_scene.hpp"

using namespace drape;
using namespace drape::session;

TEST(TeeScene, MaleDraft) {
  const auto ts = makeTeeScene(avatar::BodyType::Male, SimQuality::Draft);
  ASSERT_EQ(ts->scene.garments.size(), 1u);
  EXPECT_EQ(ts->scene.garments[0].id, "tee");
  EXPECT_EQ(ts->scene.garments[0].initialPositions.size(), ts->mesh.rest.size());
  EXPECT_EQ(ts->scene.body, &ts->field);
  EXPECT_EQ(ts->bodyType, avatar::BodyType::Male);
  EXPECT_EQ(ts->quality, SimQuality::Draft);
}

TEST(TeeScene, FemaleUsesWomensCut) {
  const auto ts = makeTeeScene(avatar::BodyType::Female, SimQuality::Draft);
  const garment::WomensTestTeeGenerator womens;
  const garment::TestTeeGenerator mens;
  EXPECT_EQ(ts->pattern, womens.generate(garment::resolveSpec(womens.specTable(), "M")));
  EXPECT_NE(ts->pattern, mens.generate(garment::resolveSpec(mens.specTable(), "M")));
  EXPECT_EQ(&teeGenerator(avatar::BodyType::Female), &teeGenerator(avatar::BodyType::Female));
}

TEST(TeeScene, BadMannequinDirThrowsWithPath) {
  const std::filesystem::path dir = "no-such-mannequin-dir";
  try {
    makeTeeScene(avatar::BodyType::Male, SimQuality::Draft, "M", dir);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find(std::filesystem::absolute(dir).string()), std::string::npos) << e.what();
  }
}
