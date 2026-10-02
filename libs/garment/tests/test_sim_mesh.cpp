#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <set>

#include "drape/garment/shapes.hpp"
#include "drape/garment/sim_mesh_builder.hpp"
#include "drape/garment/test_tee.hpp"

using namespace drape;
using namespace drape::garment;

namespace {

double triArea(const SimMesh& m, const std::array<std::uint32_t, 3>& t) {
  const Vec2d a = m.rest[t[1]] - m.rest[t[0]];
  const Vec2d b = m.rest[t[2]] - m.rest[t[0]];
  return 0.5 * (a.x() * b.y() - a.y() * b.x());
}

double minAngleDeg(const SimMesh& m) {
  double worst = 180;
  for (const auto& t : m.triangles) {
    for (int k = 0; k < 3; ++k) {
      const Vec2d p = m.rest[t[k]];
      const Vec2d a = (m.rest[t[(k + 1) % 3]] - p).normalized();
      const Vec2d b = (m.rest[t[(k + 2) % 3]] - p).normalized();
      worst = std::min(worst, std::acos(std::clamp(a.dot(b), -1.0, 1.0)) * 180 / kPi);
    }
  }
  return worst;
}

Pattern single(PatternPiece p) {
  Pattern pat;
  pat.pieces.push_back(std::move(p));
  return pat;
}

PatternPiece disc(double r) {
  const double k = 0.5522847498 * r;
  const Vec2d c(r, r);
  PatternPiece p;
  p.id = "disc";
  p.name = "disc";
  p.fabricSlot = "body";
  p.outline.add("q0", {c + Vec2d(r, 0), c + Vec2d(r, k), c + Vec2d(k, r), c + Vec2d(0, r)});
  p.outline.add("q1", {c + Vec2d(0, r), c + Vec2d(-k, r), c + Vec2d(-r, k), c + Vec2d(-r, 0)});
  p.outline.add("q2", {c + Vec2d(-r, 0), c + Vec2d(-r, -k), c + Vec2d(-k, -r), c + Vec2d(0, -r)});
  p.outline.add("q3", {c + Vec2d(0, -r), c + Vec2d(k, -r), c + Vec2d(r, -k), c + Vec2d(r, 0)});
  return p;
}

Pattern tee(const std::string& size = "M") {
  TestTeeGenerator g;
  return g.generate(resolveSpec(g.specTable(), size));
}

Pattern twoRectsSewn() {
  Pattern p;
  p.pieces.push_back(rectanglePiece("sq1", 0.2, 0.1));
  p.pieces.push_back(rectanglePiece("sq2", 0.2, 0.1));
  p.seams.push_back(Seam{"s1", segmentEdge(p.pieces[0], "right", "right", false),
                         segmentEdge(p.pieces[1], "left", "left", true), 0, SeamKind::Normal});
  return p;
}

}  // namespace

TEST(SimMesh, ParticleDistances) {
  EXPECT_DOUBLE_EQ(particleDistance(SimQuality::Draft), 0.020);
  EXPECT_DOUBLE_EQ(particleDistance(SimQuality::Standard), 0.010);
  EXPECT_DOUBLE_EQ(particleDistance(SimQuality::Fine), 0.005);
}

TEST(SimMesh, SeamPairsAreOneToOne) {
  SimMesh m = buildSimMesh(twoRectsSewn(), 0.01);
  ASSERT_EQ(m.seamPairs.size(), 11u);
  for (std::uint32_t i = 0; i < 11; ++i) {
    const auto& sp = m.seamPairs[i];
    EXPECT_EQ(sp.index, i);
    EXPECT_EQ(sp.seam, 0u);
    EXPECT_EQ(m.vertexPiece[sp.a], 0u);
    EXPECT_EQ(m.vertexPiece[sp.b], 1u);
    EXPECT_NEAR(m.rest[sp.a].y(), m.rest[sp.b].y(), 1e-9);
    EXPECT_NEAR(m.rest[sp.a].x(), 0.2, 1e-12);
    EXPECT_NEAR(m.rest[sp.b].x(), 0.0, 1e-12);
  }
}

TEST(SimMesh, AreaIsPreserved) {
  SimMesh m = buildSimMesh(single(rectanglePiece("r", 0.2, 0.1)), 0.01);
  double total = 0;
  for (const auto& t : m.triangles) total += triArea(m, t);
  EXPECT_NEAR(total, 0.02, 1e-9);
}

TEST(SimMesh, VertexAreaSumsToTotal) {
  SimMesh m = buildSimMesh(tee(), 0.02);
  double tri = 0, vert = 0;
  for (const auto& t : m.triangles) tri += triArea(m, t);
  for (double a : m.vertexArea) vert += a;
  EXPECT_NEAR(vert, tri, 1e-12);
  ASSERT_EQ(m.vertexArea.size(), m.rest.size());
  ASSERT_EQ(m.boundary.size(), m.rest.size());
  ASSERT_EQ(m.trianglePiece.size(), m.triangles.size());
  ASSERT_EQ(m.pieceGrain.size(), 5u);
}

TEST(SimMesh, TrianglesAreWellShaped) {
  EXPECT_GE(minAngleDeg(buildSimMesh(single(rectanglePiece("r", 0.2, 0.1)), 0.01)), 15.0);
  EXPECT_GE(minAngleDeg(buildSimMesh(single(disc(0.1)), 0.01)), 15.0);
  for (double h : {0.020, 0.010, 0.005}) {
    SimMesh m = buildSimMesh(tee(), h);
    EXPECT_GE(minAngleDeg(m), 15.0) << "h " << h;
    for (const auto& t : m.triangles) ASSERT_GT(triArea(m, t), 0.0);
  }
}

TEST(SimMesh, LatticeFollowsGrain) {
  for (double grainDeg : {0.0, 30.0}) {
    const double g = grainDeg * kPi / 180;
    SimMesh m = buildSimMesh(single(rectanglePiece("r", 0.3, 0.3, {std::cos(g), std::sin(g)})), 0.01);
    std::set<std::pair<std::uint32_t, std::uint32_t>> edges;
    for (const auto& t : m.triangles) {
      for (int k = 0; k < 3; ++k) {
        auto a = t[k], b = t[(k + 1) % 3];
        if (m.boundary[a] || m.boundary[b]) continue;
        edges.insert({std::min(a, b), std::max(a, b)});
      }
    }
    int aligned = 0;
    for (const auto& [a, b] : edges) {
      const Vec2d d = m.rest[b] - m.rest[a];
      double ang = std::atan2(d.y(), d.x()) * 180 / kPi - grainDeg;
      ang = std::fmod(std::fmod(ang, 60.0) + 60.0, 60.0);
      if (std::min(ang, 60.0 - ang) <= 2.0) ++aligned;
    }
    EXPECT_GE(aligned, 0.8 * static_cast<double>(edges.size())) << "grain " << grainDeg;
  }
}

TEST(SimMesh, CornersAreVertices) {
  Pattern p = tee();
  SimMesh m = buildSimMesh(p, 0.02);
  const auto& front = p.pieces[0];
  const auto& o = front.outline;
  int corners = 0;
  for (std::size_t i = 0; i < o.size(); ++i) {
    const auto& prev = o.segment((i + o.size() - 1) % o.size());
    const auto& next = o.segment(i);
    Vec2d tin = prev.derivative(1.0).normalized();
    Vec2d tout = next.derivative(0.0).normalized();
    if (std::acos(std::clamp(tin.dot(tout), -1.0, 1.0)) * 180 / kPi <= 10.0) continue;
    ++corners;
    bool found = false;
    for (std::size_t v = 0; v < m.rest.size(); ++v) {
      if (m.vertexPiece[v] == 0 && (m.rest[v] - next.p0).norm() < 1e-12) found = true;
    }
    EXPECT_TRUE(found) << o.name(i);
  }
  EXPECT_GE(corners, 6);
}

TEST(SimMesh, ThinPieceIsValid) {
  SimMesh m = buildSimMesh(single(rectanglePiece("thin", 0.5, 0.02)), 0.02);
  ASSERT_FALSE(m.triangles.empty());
  for (const auto& t : m.triangles) EXPECT_GT(triArea(m, t), 1e-12);
  for (const auto& r : m.rest) EXPECT_TRUE(r.allFinite());
}

TEST(SimMesh, InvalidPatternThrows) {
  PatternPiece p = rectanglePiece("open", 0.1, 0.1);
  Outline o;
  for (std::size_t i = 0; i + 1 < p.outline.size(); ++i) o.add(p.outline.name(i), p.outline.segment(i));
  p.outline = o;
  try {
    buildSimMesh(single(p), 0.01);
    FAIL() << "expected MeshError";
  } catch (const MeshError& e) {
    EXPECT_NE(std::string(e.what()).find("outline-closed"), std::string::npos) << e.what();
  }
}
