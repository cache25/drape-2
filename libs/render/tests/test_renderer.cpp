#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "drape/render/image_metrics.hpp"
#include "drape/render/renderer.hpp"
#include "drape/session/tee_scene.hpp"
#include "drape/sim/drape_runner.hpp"
#include "drape/sim/solver_factory.hpp"

using namespace drape;
using namespace drape::render;

namespace {

RendererAssets assets() { return {DRAPE_RENDER_ASSETS "/materials", DRAPE_RENDER_ASSETS "/ibl"}; }

// The tee on the male mannequin, draped to rest on the CPU once per test binary.
struct DrapedTee {
  std::unique_ptr<const session::TeeScene> scene;
  std::vector<Vec3f> positions;
};

const DrapedTee& drapedTee() {
  static const DrapedTee t = [] {
    DrapedTee d;
    d.scene = session::makeTeeScene(avatar::BodyType::Male, SimQuality::Draft);
    auto solver = sim::makeSolver(sim::SolverBackend::Cpu);
    solver->build(d.scene->scene);
    sim::DrapeOptions o;
    o.substeps = sim::substepsFor(SimQuality::Draft);
    o.iterations = sim::iterationsFor(SimQuality::Draft);
    sim::drapeToRest(*solver, {"tee"}, o);
    d.positions.resize(solver->particleCount());
    solver->readPositions(d.positions);
    return d;
  }();
  return t;
}

std::vector<std::pair<std::uint32_t, std::uint32_t>> weldPairs(const SimMesh& mesh) {
  std::vector<std::pair<std::uint32_t, std::uint32_t>> out;
  for (const auto& p : mesh.seamPairs) out.emplace_back(p.a, p.b);
  return out;
}

void loadTee(Renderer& r) {
  const DrapedTee& t = drapedTee();
  r.setBody(t.scene->mannequin.mesh);
  r.setGarment(t.scene->mesh.triangles, weldPairs(t.scene->mesh), t.positions.size());
  r.updateGarment(t.positions);
  Vec3d lo = Vec3d::Constant(1e9), hi = Vec3d::Constant(-1e9);
  for (const auto& v : t.scene->mannequin.mesh.vertices) {
    lo = lo.cwiseMin(v);
    hi = hi.cwiseMax(v);
  }
  r.camera().frame(lo, hi, Renderer::kVerticalFovRad, 0.8);
}

bool singleColor(const RgbaImage& img) {
  for (std::size_t i = 4; i < img.pixels.size(); i += 4)
    if (!std::equal(img.pixels.begin() + i, img.pixels.begin() + i + 3, img.pixels.begin())) return false;
  return true;
}

}  // namespace

TEST(Renderer, OffscreenTeeCoverage) {
  Renderer r(assets());
  loadTee(r);
  const RgbaImage img = r.renderOffscreen(800, 1000);
  ASSERT_EQ(img.pixels.size(), 800u * 1000u * 4u);
  writePng(DRAPE_BINARY_DIR "/render_smoke.png", img.pixels, img.width, img.height);
  EXPECT_FALSE(singleColor(img));
  const double c = coverage(img.pixels, img.width, img.height, isBlueTint);
  std::printf("tee coverage %.3f\n", c);
  EXPECT_GE(c, 0.05);
  EXPECT_LE(c, 0.40);
}

TEST(Renderer, OffscreenRendersAtTwoSizes) {
  Renderer r(assets());
  loadTee(r);
  const RgbaImage small = r.renderOffscreen(400, 500);
  const RgbaImage big = r.renderOffscreen(1200, 1500);
  EXPECT_FALSE(singleColor(small));
  EXPECT_FALSE(singleColor(big));
  const double a = coverage(small.pixels, small.width, small.height, isBlueTint);
  const double b = coverage(big.pixels, big.width, big.height, isBlueTint);
  EXPECT_LT(std::abs(a - b), 0.02) << a << " vs " << b;
}

TEST(Renderer, MissingMaterialThrows) {
  const auto empty = std::filesystem::temp_directory_path() / "drape-no-materials";
  std::filesystem::create_directories(empty);
  try {
    Renderer r({empty, empty});
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("filament"), std::string::npos) << e.what();
  }
}
