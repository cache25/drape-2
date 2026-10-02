#include <gtest/gtest.h>

#include <algorithm>

#include "drape/garment/shapes.hpp"
#include "drape/garment/validate.hpp"

using namespace drape;
using namespace drape::garment;

namespace {

Pattern twoSquaresSewn() {
  Pattern p;
  p.pieces.push_back(rectanglePiece("sq1", 0.1, 0.1));
  p.pieces.push_back(rectanglePiece("sq2", 0.1, 0.1));
  p.seams.push_back(Seam{"s1", segmentEdge(p.pieces[0], "right", "right", false),
                         segmentEdge(p.pieces[1], "left", "left", true), 0, SeamKind::Normal});
  return p;
}

bool hasRule(const std::vector<ValidationIssue>& issues, const std::string& rule, const std::string& mention = "") {
  return std::any_of(issues.begin(), issues.end(), [&](const ValidationIssue& i) {
    return i.rule == rule && (mention.empty() || i.message.find(mention) != std::string::npos);
  });
}

Outline outlineFrom(std::vector<Vec2d> pts) {
  Outline o;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    o.add("e" + std::to_string(i), CubicBezier::line(pts[i], pts[(i + 1) % pts.size()]));
  }
  return o;
}

}  // namespace

TEST(Validate, RectanglePieceShape) {
  auto r = rectanglePiece("r", 0.2, 0.1, {1, 0}, "rib");
  EXPECT_EQ(r.outline.size(), 4u);
  EXPECT_EQ(r.outline.name(0), "bottom");
  EXPECT_NEAR(r.outline.perimeter(), 0.6, 1e-12);
  EXPECT_TRUE(r.grainline.isApprox(Vec2d(1, 0)));
  EXPECT_EQ(r.fabricSlot, "rib");
}

TEST(Validate, TwoSquaresSewnIsValid) { EXPECT_TRUE(validatePattern(twoSquaresSewn()).empty()); }

TEST(Validate, OpenOutline) {
  auto p = twoSquaresSewn();
  Outline o;
  for (std::size_t i = 0; i + 1 < p.pieces[1].outline.size(); ++i) o.add(p.pieces[1].outline.name(i), p.pieces[1].outline.segment(i));
  p.pieces[1].outline = o;
  EXPECT_TRUE(hasRule(validatePattern(p), "outline-closed", "sq2"));
}

TEST(Validate, SelfIntersecting) {
  auto p = twoSquaresSewn();
  p.pieces[1].outline = outlineFrom({{0, 0}, {0.1, 0.1}, {0.1, 0}, {0, 0.1}});
  p.seams.clear();
  EXPECT_TRUE(hasRule(validatePattern(p), "outline-simple", "sq2"));
}

TEST(Validate, Clockwise) {
  auto p = twoSquaresSewn();
  p.pieces[1].outline = outlineFrom({{0, 0}, {0, 0.1}, {0.1, 0.1}, {0.1, 0}});
  p.seams.clear();
  EXPECT_TRUE(hasRule(validatePattern(p), "outline-ccw", "sq2"));
}

TEST(Validate, ZeroGrainline) {
  auto p = twoSquaresSewn();
  p.pieces[0].grainline = {0, 0};
  EXPECT_TRUE(hasRule(validatePattern(p), "grainline", "sq1"));
}

TEST(Validate, EmptyFabricSlot) {
  auto p = twoSquaresSewn();
  p.pieces[0].fabricSlot = "";
  EXPECT_TRUE(hasRule(validatePattern(p), "fabric-slot", "sq1"));
}

TEST(Validate, UnknownPiece) {
  auto p = twoSquaresSewn();
  p.seams[0].b.piece = "nope";
  EXPECT_TRUE(hasRule(validatePattern(p), "seam-piece", "s1"));
}

TEST(Validate, RangeOutside) {
  auto p = twoSquaresSewn();
  p.seams[0].a.end = 5.0;
  EXPECT_TRUE(hasRule(validatePattern(p), "seam-range", "s1"));
  auto q = twoSquaresSewn();
  q.seams[0].a.start = q.seams[0].a.end;
  EXPECT_TRUE(hasRule(validatePattern(q), "seam-range", "s1"));
}

TEST(Validate, Overlap) {
  auto p = twoSquaresSewn();
  p.pieces.push_back(rectanglePiece("sq3", 0.1, 0.1));
  p.seams.push_back(Seam{"s2", segmentEdge(p.pieces[0], "right", "right", false),
                         segmentEdge(p.pieces[2], "left", "left", true), 0, SeamKind::Normal});
  EXPECT_TRUE(hasRule(validatePattern(p), "seam-overlap", "sq1"));
}

TEST(Validate, Twisted) {
  auto p = twoSquaresSewn();
  p.seams[0].b.reversed = false;
  EXPECT_TRUE(hasRule(validatePattern(p), "seam-twisted", "s1"));
}

TEST(Validate, LengthMismatch) {
  auto p = twoSquaresSewn();
  p.pieces[1] = rectanglePiece("sq2", 0.1, 0.103);
  p.seams[0].b = segmentEdge(p.pieces[1], "left", "left", true);
  EXPECT_TRUE(hasRule(validatePattern(p), "seam-length", "s1"));
  p.seams[0].ease = 0.003;
  EXPECT_FALSE(hasRule(validatePattern(p), "seam-length"));
}

TEST(Validate, EdgeEndingAtPerimeterIsValid) {
  auto p = twoSquaresSewn();
  EXPECT_DOUBLE_EQ(p.seams[0].b.end, p.pieces[1].outline.perimeter());
  EXPECT_FALSE(hasRule(validatePattern(p), "seam-range"));
}
