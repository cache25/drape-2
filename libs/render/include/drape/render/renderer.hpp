#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include "drape/core/math.hpp"
#include "drape/core/triangle_mesh.hpp"
#include "drape/render/orbit_camera.hpp"

namespace drape::render {

struct RendererAssets {
  std::filesystem::path materialsDir;  // mannequin.filamat, cloth.filamat
  std::filesystem::path iblDir;        // cmgen output: a *_ibl.ktx somewhere below
};

struct RgbaImage {
  std::uint32_t width = 0, height = 0;
  std::vector<std::uint8_t> pixels;  // RGBA8, top row first
};

// The 3D viewport (spec 9; Phase B spec 4.2): Filament on Metal. The mannequin in a white satin finish, the garment in
// Filament's cloth shading (two-sided, inside darkened), a CC0 studio HDRI as image-based light plus a soft-shadow key
// light. Draws into a CAMetalLayer (attachLayer) or offscreen (renderOffscreen). Use from one thread.
class Renderer {
 public:
  static constexpr double kVerticalFovRad = 0.6981317007977318;  // 40 degrees

  explicit Renderer(const RendererAssets& assets);  // throws std::runtime_error("filament: <cause>")
  ~Renderer();
  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  void setBody(const TriangleMesh& body);
  // Replaces the garment mesh; weldPairs are the seam particle pairs whose normals are shared.
  void setGarment(std::span<const std::array<std::uint32_t, 3>> triangles,
                  std::span<const std::pair<std::uint32_t, std::uint32_t>> weldPairs, std::size_t vertexCount);
  void updateGarment(std::span<const Vec3f> positions);  // recomputes normals and uploads

  OrbitCamera& camera();

  void attachLayer(void* caMetalLayer);  // window rendering into this CAMetalLayer
  void resize(std::uint32_t widthPx, std::uint32_t heightPx);
  void renderFrame();  // no-op until attachLayer
  RgbaImage renderOffscreen(std::uint32_t width, std::uint32_t height);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Installs a CAMetalLayer as the NSView's backing layer (contents scale = the window's backing scale) and returns it.
void* makeMetalLayerForView(void* nsView);
// The view's current backing scale (2.0 on Retina); call after moving between displays.
double backingScale(void* nsView);
void setMetalLayerScale(void* caMetalLayer, double scale);

}  // namespace drape::render
