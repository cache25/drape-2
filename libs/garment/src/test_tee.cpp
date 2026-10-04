#include "drape/garment/test_tee.hpp"

#include <cmath>
#include <functional>
#include <string>

#include "drape/garment/shapes.hpp"
#include "drape/garment/validate.hpp"

namespace drape::garment {

namespace {

struct TeeSpec {
  double C, L, S, SL, SO, W, NW, FND, BND, AD, SD, H;
};

TeeSpec readSpec(const std::map<std::string, double>& spec, const SpecTable& table) {
  for (const auto& pom : table.poms) {
    auto it = spec.find(pom.key);
    if (it == spec.end()) throw GeneratorError("test_tee: missing '" + pom.key + "'");
    if (!(it->second > 0)) throw GeneratorError("test_tee: '" + pom.key + "' must be > 0");
  }
  TeeSpec s{spec.at("chest_width"),  spec.at("body_length"),     spec.at("shoulder_width"), spec.at("sleeve_length"),
            spec.at("sleeve_opening"), spec.at("bicep_width"),   spec.at("neck_width"),     spec.at("front_neck_drop"),
            spec.at("back_neck_drop"), spec.at("armhole_depth"), spec.at("shoulder_drop"),  spec.at("neck_rib_height")};
  // Each requirement keeps at least 2 cm between the two points the POMs place.
  auto require = [](bool ok, const char* message) {
    if (!ok) throw GeneratorError(std::string("test_tee: ") + message);
  };
  require(s.AD > s.SD + 0.02, "'armhole_depth' must exceed 'shoulder_drop' by at least 0.02 m");
  require(s.NW < s.S - 0.02, "'neck_width' must be at least 0.02 m less than 'shoulder_width'");
  require(s.AD < s.L - 0.02, "'armhole_depth' must be at least 0.02 m less than 'body_length'");
  require(s.FND < s.L - 0.02, "'front_neck_drop' must be at least 0.02 m less than 'body_length'");
  require(s.BND < s.L - 0.02, "'back_neck_drop' must be at least 0.02 m less than 'body_length'");
  return s;
}

// Front or back body piece; only the neck drop differs.
PatternPiece bodyPiece(const Id& id, const TeeSpec& s, double neckDrop, double centerAngle) {
  const Vec2d hps(s.NW / 2, s.L);
  const Vec2d sp(s.S / 2, s.L - s.SD);
  const Vec2d ab(s.C / 2, s.L - s.AD);
  const Vec2d cf(0, s.L - neckDrop);
  const CubicBezier armhole{sp, {s.S / 2, s.L - s.SD - 0.5 * (s.AD - s.SD)}, {s.S / 2 + 0.5 * (s.C / 2 - s.S / 2), s.L - s.AD},
                            ab};
  const CubicBezier neck{cf, {0.55 * s.NW / 2, s.L - neckDrop}, {s.NW / 2, s.L - 0.45 * neckDrop}, hps};
  const Vec2d mirror(-1, 1);

  PatternPiece p;
  p.id = id;
  p.name = id;
  p.outline.add("hem", CubicBezier::line({-s.C / 2, 0}, {s.C / 2, 0}));
  p.outline.add("side_r", CubicBezier::line({s.C / 2, 0}, ab));
  p.outline.add("armhole_r", armhole.reversed());
  p.outline.add("shoulder_r", CubicBezier::line(sp, hps));
  p.outline.add("neck_r", neck.reversed());
  p.outline.add("neck_l", neck.mirroredX());
  p.outline.add("shoulder_l", CubicBezier::line(hps.cwiseProduct(mirror), sp.cwiseProduct(mirror)));
  p.outline.add("armhole_l", armhole.mirroredX());
  p.outline.add("side_l", CubicBezier::line(ab.cwiseProduct(mirror), {-s.C / 2, 0}));
  p.grainline = {0, 1};
  p.fabricSlot = "body";
  p.placement = Placement{BodyRegion::Torso, centerAngle, -s.L, 1, 0.06, std::nullopt};
  return p;
}

CubicBezier capRight(const TeeSpec& s, double ch) {
  return {{s.W, s.SL - ch}, {0.75 * s.W, s.SL - 0.45 * ch}, {0.40 * s.W, s.SL}, {0, s.SL}};
}

double solveCapHeight(const TeeSpec& s, double armholeLength) {
  auto f = [&](double ch) { return length(capRight(s, ch)) - armholeLength; };
  double lo = 0.02;
  double hi = s.SL - 0.02;
  if (hi <= lo || f(hi) < 0) {
    throw GeneratorError("test_tee: no sleeve cap fits: 'sleeve_length' is too short for the armhole set by "
                         "'chest_width', 'shoulder_width', 'armhole_depth' and 'shoulder_drop'");
  }
  if (f(lo) > 0) {
    throw GeneratorError("test_tee: no sleeve cap fits: 'bicep_width' is too large for the armhole set by "
                         "'chest_width', 'shoulder_width', 'armhole_depth' and 'shoulder_drop'");
  }
  for (int i = 0; i < 100 && hi - lo > 1e-10; ++i) {
    const double mid = 0.5 * (lo + hi);
    (f(mid) > 0 ? hi : lo) = mid;
  }
  return 0.5 * (lo + hi);
}

PatternPiece sleevePiece(const Id& id, const TeeSpec& s, double ch, BodyRegion region) {
  const CubicBezier cap = capRight(s, ch);
  const Vec2d u(s.W, s.SL - ch);
  PatternPiece p;
  p.id = id;
  p.name = id;
  p.outline.add("hem", CubicBezier::line({-s.SO, 0}, {s.SO, 0}));
  p.outline.add("underarm_r", CubicBezier::line({s.SO, 0}, u));
  p.outline.add("cap_r", cap);
  p.outline.add("cap_l", cap.mirroredX().reversed());
  p.outline.add("underarm_l", CubicBezier::line({-u.x(), u.y()}, {-s.SO, 0}));
  p.grainline = {0, 1};
  p.fabricSlot = "body";
  p.placement = Placement{region, 0, s.SL, -1, 0.06, std::nullopt};
  return p;
}

double segLen(const PatternPiece& p, const char* name) {
  const auto i = *p.outline.find(name);
  return p.outline.segmentEnd(i) - p.outline.segmentStart(i);
}

}  // namespace

SpecTable TestTeeGenerator::specTable() const {
  SpecTable t;
  t.baseSize = "M";
  auto pom = [&](const char* key, double base, double grade) {
    std::string label = key;
    for (auto& c : label) c = (c == '_') ? ' ' : c;
    t.poms.push_back({key, label, base, grade});
  };
  pom("chest_width", 0.530, 0.025);
  pom("body_length", 0.720, 0.020);
  pom("shoulder_width", 0.460, 0.020);
  pom("sleeve_length", 0.210, 0.010);
  pom("sleeve_opening", 0.170, 0.008);
  pom("bicep_width", 0.190, 0.008);
  pom("neck_width", 0.180, 0.005);
  pom("front_neck_drop", 0.090, 0.003);
  pom("back_neck_drop", 0.020, 0.0);
  pom("armhole_depth", 0.250, 0.010);
  pom("shoulder_drop", 0.045, 0.0);
  pom("neck_rib_height", 0.020, 0.0);
  return t;
}

Pattern TestTeeGenerator::generate(const std::map<std::string, double>& spec) const {
  const TeeSpec s = readSpec(spec, specTable());

  PatternPiece front = bodyPiece("front", s, s.FND, 0.0);
  PatternPiece back = bodyPiece("back", s, s.BND, kPi);
  const double ch = solveCapHeight(s, segLen(front, "armhole_r"));
  PatternPiece sleeveL = sleevePiece("sleeve_l", s, ch, BodyRegion::ArmLeft);
  PatternPiece sleeveR = sleevePiece("sleeve_r", s, ch, BodyRegion::ArmRight);

  const double lf = segLen(front, "neck_r") + segLen(front, "neck_l");
  const double lb = segLen(back, "neck_r") + segLen(back, "neck_l");
  const double lr = lf + lb;
  PatternPiece rib = rectanglePiece("neck_rib", lr, s.H, {0, 1}, "rib");
  rib.placement = Placement{BodyRegion::Neck, -kPi / 2, 0.0, 1, 0.0, lr / (2 * kPi)};

  Pattern p;
  p.pieces = {front, back, sleeveL, sleeveR, rib};
  auto seam = [&](const char* id, const EdgeRef& a, const EdgeRef& b) {
    p.seams.push_back(Seam{id, a, b, 0.0, SeamKind::Normal});
  };
  auto edge = [](const PatternPiece& piece, const char* first, const char* last, bool reversed) {
    return segmentEdge(piece, first, last, reversed);
  };
  seam("shoulder_wl", edge(front, "shoulder_r", "shoulder_r", false), edge(back, "shoulder_l", "shoulder_l", true));
  seam("shoulder_wr", edge(front, "shoulder_l", "shoulder_l", false), edge(back, "shoulder_r", "shoulder_r", true));
  seam("side_wl", edge(front, "side_r", "side_r", false), edge(back, "side_l", "side_l", true));
  seam("side_wr", edge(front, "side_l", "side_l", false), edge(back, "side_r", "side_r", true));
  seam("armhole_front_wl", edge(front, "armhole_r", "armhole_r", false), edge(sleeveL, "cap_l", "cap_l", true));
  seam("armhole_back_wl", edge(back, "armhole_l", "armhole_l", false), edge(sleeveL, "cap_r", "cap_r", true));
  seam("armhole_front_wr", edge(front, "armhole_l", "armhole_l", false), edge(sleeveR, "cap_r", "cap_r", true));
  seam("armhole_back_wr", edge(back, "armhole_r", "armhole_r", false), edge(sleeveR, "cap_l", "cap_l", true));
  seam("underarm_l", edge(sleeveL, "underarm_r", "underarm_r", false), edge(sleeveL, "underarm_l", "underarm_l", true));
  seam("underarm_r", edge(sleeveR, "underarm_r", "underarm_r", false), edge(sleeveR, "underarm_l", "underarm_l", true));
  seam("neck_front", EdgeRef{"neck_rib", 0.0, lf, false}, edge(front, "neck_r", "neck_l", true));
  seam("neck_back", EdgeRef{"neck_rib", lf, lr, false}, edge(back, "neck_r", "neck_l", true));
  seam("rib_close", edge(rib, "right", "right", false), edge(rib, "left", "left", true));
  // Backstop for combinations the checks above do not foresee: never hand out a pattern that fails validation.
  if (const auto issues = validatePattern(p); !issues.empty()) {
    throw GeneratorError("test_tee: these measurements give an invalid pattern (" + issues.front().rule + ": " +
                         issues.front().message + ")");
  }
  return p;
}

}  // namespace drape::garment
