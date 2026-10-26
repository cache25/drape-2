#include "drape/render/renderer.hpp"

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/Skybox.h>
#include <filament/SwapChain.h>
#include <filament/Texture.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <geometry/SurfaceOrientation.h>
#include <image/Ktx1Bundle.h>
#include <ktxreader/Ktx1Reader.h>
#include <utils/EntityManager.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#include "drape/render/normals.hpp"

namespace drape::render {

namespace fl = filament;
using fl::math::float3;
using fl::math::short4;

namespace {

constexpr float kTeeColor[3] = {0.16f, 0.38f, 0.85f};  // the developer-preview blue
constexpr float kSheen[3] = {0.25f, 0.30f, 0.40f};
constexpr float kBackground[4] = {0.93f, 0.93f, 0.94f, 1.0f};

std::vector<char> readFile(const std::filesystem::path& p) {
  std::ifstream in(p, std::ios::binary);
  if (!in) throw std::runtime_error("filament: cannot read " + std::filesystem::absolute(p).string());
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// A heap copy handed to Filament, freed by its callback once uploaded.
template <class T>
fl::backend::BufferDescriptor copyToBuffer(const std::vector<T>& v) {
  const std::size_t bytes = v.size() * sizeof(T);
  void* mem = std::malloc(std::max<std::size_t>(bytes, 1));
  std::memcpy(mem, v.data(), bytes);
  return fl::backend::BufferDescriptor(mem, bytes, [](void* buf, size_t, void*) { std::free(buf); });
}

std::vector<short4> tangentQuats(const std::vector<Vec3f>& normals) {
  std::vector<float3> n(normals.size());
  for (std::size_t i = 0; i < normals.size(); ++i) n[i] = {normals[i].x(), normals[i].y(), normals[i].z()};
  std::vector<short4> q(normals.size());
  if (normals.empty()) return q;
  auto* so = geometry::SurfaceOrientation::Builder().vertexCount(n.size()).normals(n.data()).build();
  so->getQuats(q.data(), q.size());
  delete so;
  return q;
}

fl::Box boundsOf(std::span<const Vec3f> x) {
  Vec3f lo = Vec3f::Constant(1e9f), hi = Vec3f::Constant(-1e9f);
  for (const auto& p : x) {
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  if (x.empty()) lo = hi = Vec3f::Zero();
  const Vec3f c = 0.5f * (lo + hi), h = 0.5f * (hi - lo) + Vec3f::Constant(1e-3f);
  return fl::Box({c.x(), c.y(), c.z()}, {h.x(), h.y(), h.z()});
}

}  // namespace

struct Renderer::Impl {
  fl::Engine* engine = nullptr;
  fl::Renderer* renderer = nullptr;
  fl::Scene* scene = nullptr;
  fl::View* view = nullptr;
  utils::Entity cameraEntity, sun;
  fl::Camera* cam = nullptr;
  fl::SwapChain* swapChain = nullptr;
  fl::Material* mannequinMat = nullptr;
  fl::Material* clothMat = nullptr;
  fl::MaterialInstance* mannequinMi = nullptr;
  fl::MaterialInstance* clothMi = nullptr;
  fl::Texture* iblTexture = nullptr;
  fl::IndirectLight* ibl = nullptr;
  fl::Skybox* skybox = nullptr;
  OrbitCamera orbit;
  std::uint32_t width = 1, height = 1;

  struct Mesh {
    utils::Entity entity;
    fl::VertexBuffer* vb = nullptr;
    fl::IndexBuffer* ib = nullptr;
    std::size_t vertexCount = 0;
  };
  Mesh body, garment;
  std::vector<std::array<std::uint32_t, 3>> garmentTriangles;
  std::vector<std::pair<std::uint32_t, std::uint32_t>> garmentWelds;

  void destroyMesh(Mesh& m) {
    if (m.vb) {
      scene->remove(m.entity);
      engine->destroy(m.entity);
      utils::EntityManager::get().destroy(m.entity);
      engine->destroy(m.vb);
      engine->destroy(m.ib);
    }
    m = Mesh{};
  }

  Mesh makeMesh(std::size_t vertexCount, const std::vector<std::uint32_t>& indices, fl::MaterialInstance* mi) {
    Mesh m;
    m.vertexCount = vertexCount;
    m.vb = fl::VertexBuffer::Builder()
               .vertexCount(static_cast<uint32_t>(vertexCount))
               .bufferCount(2)
               .attribute(fl::VertexAttribute::POSITION, 0, fl::VertexBuffer::AttributeType::FLOAT3)
               .attribute(fl::VertexAttribute::TANGENTS, 1, fl::VertexBuffer::AttributeType::SHORT4)
               .normalized(fl::VertexAttribute::TANGENTS)
               .build(*engine);
    m.ib = fl::IndexBuffer::Builder()
               .indexCount(static_cast<uint32_t>(indices.size()))
               .bufferType(fl::IndexBuffer::IndexType::UINT)
               .build(*engine);
    m.ib->setBuffer(*engine, copyToBuffer(indices));
    m.entity = utils::EntityManager::get().create();
    fl::RenderableManager::Builder(1)
        .boundingBox(fl::Box({0, 1, 0}, {1, 1, 1}))
        .material(0, mi)
        .geometry(0, fl::RenderableManager::PrimitiveType::TRIANGLES, m.vb, m.ib, 0, indices.size())
        .culling(false)
        .castShadows(true)
        .receiveShadows(true)
        .build(*engine, m.entity);
    scene->addEntity(m.entity);
    return m;
  }

  void upload(Mesh& m, std::span<const Vec3f> positions, const std::vector<Vec3f>& normals) {
    std::vector<float3> p(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) p[i] = {positions[i].x(), positions[i].y(), positions[i].z()};
    m.vb->setBufferAt(*engine, 0, copyToBuffer(p));
    m.vb->setBufferAt(*engine, 1, copyToBuffer(tangentQuats(normals)));
    auto& rm = engine->getRenderableManager();
    rm.setAxisAlignedBoundingBox(rm.getInstance(m.entity), boundsOf(positions));
  }

  void updateCamera(std::uint32_t w, std::uint32_t h) {
    const double aspect = static_cast<double>(w) / std::max<std::uint32_t>(h, 1);
    cam->setProjection(kVerticalFovRad * 180.0 / kPi, aspect, 0.05, 50.0, fl::Camera::Fov::VERTICAL);
    const Vec3d e = orbit.eye(), t = orbit.target, u = orbit.up();
    cam->lookAt({e.x(), e.y(), e.z()}, {t.x(), t.y(), t.z()}, {u.x(), u.y(), u.z()});
  }
};

Renderer::Renderer(const RendererAssets& assets) : impl_(std::make_unique<Impl>()) {
  auto& s = *impl_;
  // Read every asset first so a missing file fails before the engine exists.
  const auto mannequinData = readFile(assets.materialsDir / "mannequin.filamat");
  const auto clothData = readFile(assets.materialsDir / "cloth.filamat");
  std::filesystem::path iblPath;
  std::error_code ec;
  if (std::filesystem::is_directory(assets.iblDir, ec))
    for (const auto& entry : std::filesystem::recursive_directory_iterator(assets.iblDir))
      if (entry.path().filename().string().ends_with("_ibl.ktx")) iblPath = entry.path();
  if (iblPath.empty())
    throw std::runtime_error("filament: no *_ibl.ktx under " + std::filesystem::absolute(assets.iblDir).string());
  const auto iblData = readFile(iblPath);

  s.engine = fl::Engine::create(fl::Engine::Backend::METAL);
  if (!s.engine) throw std::runtime_error("filament: cannot create a Metal engine");
  s.renderer = s.engine->createRenderer();
  s.scene = s.engine->createScene();
  s.view = s.engine->createView();
  s.cameraEntity = utils::EntityManager::get().create();
  s.cam = s.engine->createCamera(s.cameraEntity);
  s.view->setScene(s.scene);
  s.view->setCamera(s.cam);
  s.view->setShadowType(fl::View::ShadowType::PCSS);
  s.view->setAntiAliasing(fl::View::AntiAliasing::FXAA);
  fl::View::MultiSampleAntiAliasingOptions msaa;
  msaa.enabled = true;
  msaa.sampleCount = 4;
  s.view->setMultiSampleAntiAliasingOptions(msaa);
  fl::View::AmbientOcclusionOptions ao;
  ao.enabled = true;
  s.view->setAmbientOcclusionOptions(ao);
  fl::Renderer::ClearOptions clear;
  clear.clearColor = {kBackground[0], kBackground[1], kBackground[2], kBackground[3]};
  clear.clear = true;
  s.renderer->setClearOptions(clear);

  s.mannequinMat = fl::Material::Builder().package(mannequinData.data(), mannequinData.size()).build(*s.engine);
  s.clothMat = fl::Material::Builder().package(clothData.data(), clothData.size()).build(*s.engine);
  if (!s.mannequinMat || !s.clothMat) throw std::runtime_error("filament: cannot load the materials");
  s.mannequinMi = s.mannequinMat->createInstance();
  s.clothMi = s.clothMat->createInstance();
  s.clothMi->setParameter("baseColor", fl::RgbType::sRGB, float3{kTeeColor[0], kTeeColor[1], kTeeColor[2]});
  s.clothMi->setParameter("sheenColor", fl::RgbType::sRGB, float3{kSheen[0], kSheen[1], kSheen[2]});

  auto* bundle = new image::Ktx1Bundle(reinterpret_cast<const uint8_t*>(iblData.data()),
                                       static_cast<uint32_t>(iblData.size()));
  float3 sh[9];
  if (!bundle->getSphericalHarmonics(sh)) {
    delete bundle;
    throw std::runtime_error("filament: " + iblPath.string() + " has no spherical harmonics (run cmgen --deploy)");
  }
  s.iblTexture = ktxreader::Ktx1Reader::createTexture(s.engine, bundle, false);  // takes the bundle
  s.ibl = fl::IndirectLight::Builder().reflections(s.iblTexture).irradiance(3, sh).intensity(30000.0f).build(*s.engine);
  s.scene->setIndirectLight(s.ibl);
  s.skybox = fl::Skybox::Builder()
                 .color({kBackground[0], kBackground[1], kBackground[2], kBackground[3]})
                 .build(*s.engine);
  s.scene->setSkybox(s.skybox);

  s.sun = utils::EntityManager::get().create();
  fl::LightManager::Builder(fl::LightManager::Type::DIRECTIONAL)
      .color({1.0f, 0.98f, 0.95f})
      .intensity(60000.0f)
      .direction({-0.4f, -1.0f, -0.6f})
      .castShadows(true)
      .build(*s.engine, s.sun);
  s.scene->addEntity(s.sun);
}

Renderer::~Renderer() {
  auto& s = *impl_;
  if (!s.engine) return;
  s.destroyMesh(s.body);
  s.destroyMesh(s.garment);
  s.engine->destroy(s.sun);
  utils::EntityManager::get().destroy(s.sun);
  if (s.ibl) s.engine->destroy(s.ibl);
  if (s.iblTexture) s.engine->destroy(s.iblTexture);
  if (s.skybox) s.engine->destroy(s.skybox);
  if (s.clothMi) s.engine->destroy(s.clothMi);
  if (s.mannequinMi) s.engine->destroy(s.mannequinMi);
  if (s.clothMat) s.engine->destroy(s.clothMat);
  if (s.mannequinMat) s.engine->destroy(s.mannequinMat);
  s.engine->destroyCameraComponent(s.cameraEntity);
  utils::EntityManager::get().destroy(s.cameraEntity);
  s.engine->destroy(s.view);
  s.engine->destroy(s.scene);
  s.engine->destroy(s.renderer);
  if (s.swapChain) s.engine->destroy(s.swapChain);
  fl::Engine::destroy(&s.engine);
}

void Renderer::setBody(const TriangleMesh& body) {
  auto& s = *impl_;
  s.destroyMesh(s.body);
  std::vector<Vec3f> x(body.vertices.size());
  for (std::size_t i = 0; i < x.size(); ++i) x[i] = body.vertices[i].cast<float>();
  std::vector<std::uint32_t> idx;
  for (const auto& t : body.triangles) idx.insert(idx.end(), t.begin(), t.end());
  s.body = s.makeMesh(x.size(), idx, s.mannequinMi);
  s.upload(s.body, x, vertexNormals(x, body.triangles, {}));
}

void Renderer::setGarment(std::span<const std::array<std::uint32_t, 3>> triangles,
                          std::span<const std::pair<std::uint32_t, std::uint32_t>> weldPairs,
                          std::size_t vertexCount) {
  auto& s = *impl_;
  s.destroyMesh(s.garment);
  s.garmentTriangles.assign(triangles.begin(), triangles.end());
  s.garmentWelds.assign(weldPairs.begin(), weldPairs.end());
  std::vector<std::uint32_t> idx;
  for (const auto& t : triangles) idx.insert(idx.end(), t.begin(), t.end());
  s.garment = s.makeMesh(vertexCount, idx, s.clothMi);
}

void Renderer::updateGarment(std::span<const Vec3f> positions) {
  auto& s = *impl_;
  if (!s.garment.vb || positions.size() != s.garment.vertexCount) return;
  s.upload(s.garment, positions, vertexNormals(positions, s.garmentTriangles, s.garmentWelds));
}

OrbitCamera& Renderer::camera() { return impl_->orbit; }

void Renderer::attachLayer(void* caMetalLayer) {
  auto& s = *impl_;
  if (s.swapChain) s.engine->destroy(s.swapChain);
  s.swapChain = s.engine->createSwapChain(caMetalLayer);
}

void Renderer::resize(std::uint32_t widthPx, std::uint32_t heightPx) {
  impl_->width = std::max<std::uint32_t>(widthPx, 1);
  impl_->height = std::max<std::uint32_t>(heightPx, 1);
}

void Renderer::renderFrame() {
  auto& s = *impl_;
  if (!s.swapChain) return;
  s.view->setViewport({0, 0, s.width, s.height});
  s.updateCamera(s.width, s.height);
  if (s.renderer->beginFrame(s.swapChain)) {
    s.renderer->render(s.view);
    s.renderer->endFrame();
  }
}

RgbaImage Renderer::renderOffscreen(std::uint32_t width, std::uint32_t height) {
  auto& s = *impl_;
  RgbaImage img;
  img.width = width;
  img.height = height;
  std::vector<std::uint8_t> raw(static_cast<std::size_t>(width) * height * 4);
  fl::SwapChain* sc = s.engine->createSwapChain(width, height, fl::SwapChain::CONFIG_READABLE);
  s.view->setViewport({0, 0, width, height});
  s.updateCamera(width, height);
  bool done = false;
  for (int attempt = 0; attempt < 100 && !done; ++attempt) {
    if (!s.renderer->beginFrame(sc)) {
      s.engine->flushAndWait();
      continue;
    }
    s.renderer->render(s.view);
    s.renderer->readPixels(0, 0, width, height,
                           fl::backend::PixelBufferDescriptor(raw.data(), raw.size(), fl::backend::PixelDataFormat::RGBA,
                                                              fl::backend::PixelDataType::UBYTE,
                                                              [](void*, size_t, void* flag) { *static_cast<bool*>(flag) = true; },
                                                              &done));
    s.renderer->endFrame();
    s.engine->flushAndWait();
  }
  s.engine->destroy(sc);
  s.engine->flushAndWait();
  if (!done) throw std::runtime_error("filament: offscreen frame was never rendered");
  // Filament reads bottom row first; flip to top-first.
  img.pixels.resize(raw.size());
  const std::size_t row = static_cast<std::size_t>(width) * 4;
  for (std::uint32_t y = 0; y < height; ++y)
    std::memcpy(img.pixels.data() + y * row, raw.data() + (height - 1 - y) * row, row);
  s.view->setViewport({0, 0, s.width, s.height});
  return img;
}

}  // namespace drape::render
