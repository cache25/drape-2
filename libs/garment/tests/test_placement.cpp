#include <gtest/gtest.h>

#include <cmath>

#include "drape/avatar/test_body.hpp"
#include "drape/garment/placement.hpp"
#include "drape/garment/sim_mesh_builder.hpp"
#include "drape/garment/test_tee.hpp"

using namespace drape;
using namespace drape::garment;

namespace {

struct Placed {
  Pattern pattern;
  SimMesh mesh;
  std::vector<Vec3d> pos;
  BodyRegions regions;
};

const Placed& teeOnTestBody() {
  static const Placed p = [] {
    Placed out;
    TestTeeGenerator g;
    out.pattern = g.generate(resolveSpec(g.specTable(), "M"));
    out.mesh = buildSimMesh(out.pattern, 0.02);
    out.regions = avatar::makeTestBody().regions;
    out.pos = placePattern(out.pattern, out.mesh, out.regions);
    return out;
  }();
  return p;
}

// Distance from point to the line through c.origin along c.axis, and the radial unit vector.
Vec3d radial(const RegionCylinder& c, const Vec3d& p) {
  const Vec3d d = p - c.origin;
  return d - d.dot(c.axis) * c.axis;
}

}  // namespace

TEST(Placement, FrontWrapsTorsoCylinder) {
  const auto& t = teeOnTestBody();
  const auto& c = t.regions.cylinders.at(BodyRegion::Torso);
  const double L = 0.72, R = 0.23;
  ASSERT_EQ(t.pos.size(), t.mesh.rest.size());
  int checked = 0;
  for (std::size_t v = 0; v < t.mesh.rest.size(); ++v) {
    if (t.mesh.vertexPiece[v] != 0) continue;  // front
    const Vec2d r = t.mesh.rest[v];
    const Vec3d rad = radial(c, t.pos[v]);
    EXPECT_NEAR(rad.norm(), R, 1e-9);
    const double angle = std::atan2(rad.dot(c.e1()), rad.dot(c.e0));
    EXPECT_NEAR(angle, r.x() / R, 1e-9);
    EXPECT_NEAR(t.pos[v].y(), 1.47 - L + r.y(), 1e-9);
    ++checked;
  }
  EXPECT_GT(checked, 100);
}

TEST(Placement, OutsideFacesPointOutward) {
  const auto& t = teeOnTestBody();
  for (std::size_t i = 0; i < t.mesh.triangles.size(); ++i) {
    const auto& tri = t.mesh.triangles[i];
    const auto& piece = t.pattern.pieces[t.mesh.trianglePiece[i]];
    const auto& c = t.regions.cylinders.at(piece.placement.region);
    const Vec3d n = (t.pos[tri[1]] - t.pos[tri[0]]).cross(t.pos[tri[2]] - t.pos[tri[0]]);
    const Vec3d centroid = (t.pos[tri[0]] + t.pos[tri[1]] + t.pos[tri[2]]) / 3.0;
    ASSERT_GT(n.dot(radial(c, centroid)), 0.0) << piece.id << " triangle " << i;
  }
}

TEST(Placement, SeamEndsStartClose) {
  // Pairs must line up the right way round (pairing beats the reversed pairing) and be closable
  // well inside the 3 s assembly budget at 0.25 m/s.
  const auto& t = teeOnTestBody();
  std::map<std::uint32_t, std::vector<const SeamPair*>> bySeam;
  for (const auto& sp : t.mesh.seamPairs) bySeam[sp.seam].push_back(&sp);
  ASSERT_EQ(bySeam.size(), 13u);
  for (const auto& [seam, pairs] : bySeam) {
    const auto& first = *pairs.front();
    const auto& last = *pairs.back();
    const double straight = (t.pos[first.a] - t.pos[first.b]).norm() + (t.pos[last.a] - t.pos[last.b]).norm();
    const double crossed = (t.pos[first.a] - t.pos[last.b]).norm() + (t.pos[last.a] - t.pos[first.b]).norm();
    const auto& id = t.pattern.seams[seam].id;
    if (id != "rib_close") {
      EXPECT_LT(straight, crossed) << id;
    }
    EXPECT_LT((t.pos[first.a] - t.pos[first.b]).norm(), 0.5) << id;
    EXPECT_LT((t.pos[last.a] - t.pos[last.b]).norm(), 0.5) << id;
  }
}

TEST(Placement, RibRingCloses) {
  const auto& t = teeOnTestBody();
  int count = 0;
  for (const auto& sp : t.mesh.seamPairs) {
    if (t.pattern.seams[sp.seam].id != "rib_close") continue;
    EXPECT_LT((t.pos[sp.a] - t.pos[sp.b]).norm(), 0.001);
    ++count;
  }
  EXPECT_GT(count, 0);
}

TEST(Placement, MissingRegionThrows) {
  const auto& t = teeOnTestBody();
  BodyRegions regions = t.regions;
  regions.cylinders.erase(BodyRegion::ArmLeft);
  try {
    placePattern(t.pattern, t.mesh, regions);
    FAIL() << "expected PlacementError";
  } catch (const PlacementError& e) {
    EXPECT_NE(std::string(e.what()).find("ArmLeft"), std::string::npos) << e.what();
  }
}
