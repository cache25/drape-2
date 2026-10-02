#include <gtest/gtest.h>

#include <set>

#include "drape/garment/test_tee.hpp"
#include "drape/garment/validate.hpp"

using namespace drape;
using namespace drape::garment;

namespace {

Pattern teeFor(const std::string& size, const SpecOverrides& o = {}) {
  TestTeeGenerator g;
  return g.generate(resolveSpec(g.specTable(), size, o));
}

Vec2d segStart(const PatternPiece& p, const char* name) { return p.outline.segment(*p.outline.find(name)).p0; }
Vec2d segEnd(const PatternPiece& p, const char* name) { return p.outline.segment(*p.outline.find(name)).p3; }
double segLen(const PatternPiece& p, const char* name) {
  auto i = *p.outline.find(name);
  return p.outline.segmentEnd(i) - p.outline.segmentStart(i);
}

}  // namespace

TEST(TestTee, Identity) {
  TestTeeGenerator g;
  EXPECT_EQ(g.id(), "test_tee");
  EXPECT_EQ(g.version(), 1);
  EXPECT_EQ(g.slot(), OutfitSlot::Top);
}

TEST(TestTee, ValidForAllSizes) {
  for (const auto& size : kStandardSizes) {
    auto issues = validatePattern(teeFor(size));
    EXPECT_TRUE(issues.empty()) << size << ": " << (issues.empty() ? "" : issues[0].message);
  }
}

TEST(TestTee, PomsReproduceSpec) {
  TestTeeGenerator g;
  for (const auto& size : kStandardSizes) {
    auto spec = resolveSpec(g.specTable(), size);
    Pattern p = g.generate(spec);
    const auto& front = *p.findPiece("front");
    const auto& back = *p.findPiece("back");
    const auto& sleeve = *p.findPiece("sleeve_l");
    const double tol = 0.001;
    EXPECT_NEAR(segEnd(front, "side_r").x() - segStart(front, "side_l").x(), spec["chest_width"], tol) << size;
    EXPECT_NEAR(segEnd(front, "shoulder_r").y() - segStart(front, "hem").y(), spec["body_length"], tol) << size;
    EXPECT_NEAR((segEnd(back, "armhole_r") - segStart(back, "armhole_l")).norm(), spec["shoulder_width"], tol) << size;
    EXPECT_NEAR(segEnd(sleeve, "cap_r").y() - segStart(sleeve, "hem").y(), spec["sleeve_length"], tol) << size;
    EXPECT_NEAR((segEnd(sleeve, "hem").x() - segStart(sleeve, "hem").x()) / 2, spec["sleeve_opening"], tol) << size;
    EXPECT_NEAR((segEnd(sleeve, "underarm_r").x() - segStart(sleeve, "underarm_l").x()) / 2, spec["bicep_width"], tol)
        << size;
    EXPECT_NEAR((segEnd(front, "shoulder_r") - segStart(front, "shoulder_l")).norm(), spec["neck_width"], tol) << size;
  }
}

TEST(TestTee, HasThirteenNamedSeams) {
  std::set<std::string> ids;
  for (const auto& s : teeFor("M").seams) ids.insert(s.id);
  const std::set<std::string> expected{"shoulder_wl",      "shoulder_wr",     "side_wl",          "side_wr",
                                       "armhole_front_wl", "armhole_back_wl", "armhole_front_wr", "armhole_back_wr",
                                       "underarm_l",       "underarm_r",      "neck_front",       "neck_back",
                                       "rib_close"};
  EXPECT_EQ(ids, expected);
}

TEST(TestTee, CapMatchesArmhole) {
  for (const auto& size : kStandardSizes) {
    Pattern p = teeFor(size);
    EXPECT_NEAR(segLen(*p.findPiece("sleeve_l"), "cap_l"), segLen(*p.findPiece("front"), "armhole_r"), 1e-4) << size;
  }
}

TEST(TestTee, PiecesSlotsAndGrain) {
  Pattern p = teeFor("M");
  std::vector<std::string> ids;
  for (const auto& piece : p.pieces) {
    ids.push_back(piece.id);
    EXPECT_TRUE(piece.grainline.isApprox(Vec2d(0, 1))) << piece.id;
    EXPECT_EQ(piece.fabricSlot, piece.id == "neck_rib" ? "rib" : "body") << piece.id;
  }
  EXPECT_EQ(ids, (std::vector<std::string>{"front", "back", "sleeve_l", "sleeve_r", "neck_rib"}));
  EXPECT_EQ(p.findPiece("sleeve_l")->placement.region, BodyRegion::ArmLeft);
  EXPECT_EQ(p.findPiece("back")->placement.centerAngle, kPi);
  ASSERT_TRUE(p.findPiece("neck_rib")->placement.wrapRadius.has_value());
}

TEST(TestTee, InfeasibleSpecThrows) {
  auto expectThrowMentioning = [](const SpecOverrides& o, const std::string& word) {
    try {
      teeFor("M", o);
      ADD_FAILURE() << "expected GeneratorError for " << word;
    } catch (const GeneratorError& e) {
      EXPECT_NE(std::string(e.what()).find(word), std::string::npos) << e.what();
    }
  };
  expectThrowMentioning({{"M", {{"bicep_width", 0.30}}}}, "bicep_width");
  expectThrowMentioning({{"M", {{"chest_width", 0.0}}}}, "chest_width");
  expectThrowMentioning({{"M", {{"sleeve_length", -0.1}}}}, "sleeve_length");
  expectThrowMentioning({{"M", {{"armhole_depth", 0.05}}}}, "armhole_depth");
}
