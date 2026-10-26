#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drape/render/image_metrics.hpp"
#include "drape/render/normals.hpp"
#include "drape/render/orbit_camera.hpp"

using namespace drape;
using namespace drape::render;

TEST(Normals, FlatSheetPointsUp) {
  // A 3x3 grid in the XZ plane, triangles wound counter-clockwise seen from +Y.
  std::vector<Vec3f> x;
  for (int j = 0; j < 3; ++j)
    for (int i = 0; i < 3; ++i) x.emplace_back(0.1f * i, 0.0f, -0.1f * j);
  std::vector<std::array<std::uint32_t, 3>> tris;
  for (std::uint32_t j = 0; j < 2; ++j)
    for (std::uint32_t i = 0; i < 2; ++i) {
      const std::uint32_t a = j * 3 + i, b = a + 1, c = a + 3, d = c + 1;
      tris.push_back({a, b, d});
      tris.push_back({a, d, c});
    }
  const auto n = vertexNormals(x, tris, {});
  ASSERT_EQ(n.size(), x.size());
  for (const auto& v : n) EXPECT_LE((v - Vec3f(0, 1, 0)).norm(), 1e-6f);
}

TEST(Normals, WeldPairsShareNormal) {
  // Quad A in the XZ plane (normal +Y), quad B in the XY plane (normal +Z), meeting along the X axis; B's edge
  // vertices duplicate A's and are welded to them.
  const std::vector<Vec3f> x{{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1},    // A: 0..3
                             {0, 0, 0}, {1, 0, 0}, {1, -1, 0}, {0, -1, 0}};  // B: 4..7
  const std::vector<std::array<std::uint32_t, 3>> tris{{0, 2, 1}, {0, 3, 2}, {4, 6, 5}, {4, 7, 6}};
  const std::vector<std::pair<std::uint32_t, std::uint32_t>> weld{{0, 4}, {1, 5}};
  const auto n = vertexNormals(x, tris, weld);
  for (const auto& [a, b] : weld) {
    EXPECT_LE((n[a] - n[b]).norm(), 1e-6f);
    EXPECT_NEAR(std::acos(n[a].dot(Vec3f(0, 1, 0))), kPi / 4, 1e-5);
    EXPECT_NEAR(std::acos(n[a].dot(Vec3f(0, 0, 1))), kPi / 4, 1e-5);
  }
  EXPECT_LE((n[2] - Vec3f(0, 1, 0)).norm(), 1e-6f);
}

TEST(Normals, DegenerateVertexPointsUp) {
  const std::vector<Vec3f> x{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  const std::vector<std::array<std::uint32_t, 3>> tris{{0, 1, 2}};
  for (const auto& v : vertexNormals(x, tris, {})) EXPECT_EQ(v, Vec3f(0, 1, 0));
}

namespace {

// True when `p` lies inside the camera's view frustum.
bool inFrustum(const OrbitCamera& cam, const Vec3d& p, double vfov, double aspect) {
  const Vec3d eye = cam.eye();
  const Vec3d forward = (cam.target - eye).normalized();
  const Vec3d right = forward.cross(cam.up()).normalized();
  const Vec3d up = right.cross(forward);
  const Vec3d d = p - eye;
  const double z = d.dot(forward);
  if (z <= 0) return false;
  const double t = std::tan(vfov / 2);
  return std::abs(d.dot(up)) / z <= t && std::abs(d.dot(right)) / z <= t * aspect;
}

}  // namespace

TEST(OrbitCamera, FrameContainsBox) {
  const Vec3d lo(-0.9, 0.0, -0.3), hi(0.9, 1.8, 0.3);  // a mannequin's bounds with arms out
  for (double aspect : {16.0 / 9.0, 1.0, 0.6}) {
    OrbitCamera cam;
    const double vfov = 40.0 * kPi / 180.0;
    cam.frame(lo, hi, vfov, aspect);
    EXPECT_GT(cam.eye().z(), hi.z()) << "camera in front of the body (+Z), looking toward -Z";
    for (int k = 0; k < 8; ++k) {
      const Vec3d c((k & 1) ? hi.x() : lo.x(), (k & 2) ? hi.y() : lo.y(), (k & 4) ? hi.z() : lo.z());
      EXPECT_TRUE(inFrustum(cam, c, vfov, aspect)) << "aspect " << aspect << " corner " << k;
    }
  }
}

// The body fills the view: a standing figure's height takes at least 80% of a portrait viewport.
TEST(OrbitCamera, FrameIsTight) {
  const Vec3d lo(-0.9, 0.0, -0.3), hi(0.9, 1.8, 0.3);
  const double vfov = 40.0 * kPi / 180.0, aspect = 0.8;
  OrbitCamera cam;
  cam.frame(lo, hi, vfov, aspect);
  // Projected half-height of the box's front face, as a fraction of the view's half-height.
  const double depth = cam.distance - hi.z() + cam.target.z();
  const double fraction = (0.5 * (hi.y() - lo.y()) / depth) / std::tan(vfov / 2);
  EXPECT_GE(fraction, 0.8);
  EXPECT_LE(fraction, 1.0);
}

TEST(OrbitCamera, PitchClamped) {
  OrbitCamera cam;
  cam.orbit(0, 1e6);
  EXPECT_DOUBLE_EQ(std::abs(cam.pitch), 1.5);
  cam.orbit(0, -1e7);
  EXPECT_DOUBLE_EQ(cam.pitch, -1.5);
}

TEST(OrbitCamera, ZoomClamped) {
  OrbitCamera cam;
  cam.distance = 3;
  cam.zoom(1000);
  EXPECT_DOUBLE_EQ(cam.distance, 0.2);
  cam.zoom(-1000);
  EXPECT_DOUBLE_EQ(cam.distance, 20.0);
}

TEST(OrbitCamera, PanMovesTargetSideways) {
  OrbitCamera cam;
  cam.distance = 2;
  const Vec3d before = cam.target;
  cam.pan(100, 0, 1000, 40.0 * kPi / 180.0);
  EXPECT_LT(cam.target.x(), before.x()) << "dragging right moves the scene right, so the target moves left";
  EXPECT_NEAR(cam.target.y(), before.y(), 1e-12);
}

TEST(ImageMetrics, Coverage) {
  std::vector<std::uint8_t> rgba(10 * 10 * 4, 255);
  for (int i = 0; i < 25; ++i) {
    rgba[4 * i + 0] = 40;
    rgba[4 * i + 1] = 90;
    rgba[4 * i + 2] = 210;
  }
  EXPECT_DOUBLE_EQ(coverage(rgba, 10, 10, isBlueTint), 0.25);
  EXPECT_FALSE(isBlueTint(235, 232, 227));  // the mannequin's off-white
}

TEST(ImageMetrics, WritePngWritesASignedFile) {
  const std::vector<std::uint8_t> rgba{255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255};
  const auto path = std::filesystem::temp_directory_path() / "drape_write_png_test.png";
  writePng(path, rgba, 2, 2);
  std::ifstream in(path, std::ios::binary);
  char sig[8] = {};
  in.read(sig, 8);
  EXPECT_EQ(std::string(sig + 1, 3), "PNG");
  std::filesystem::remove(path);
  EXPECT_THROW(writePng("/nonexistent-dir/x.png", rgba, 2, 2), std::runtime_error);
}
