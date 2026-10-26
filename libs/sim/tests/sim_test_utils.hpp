#pragma once

#include <cmath>
#include <functional>
#include <vector>

#include "drape/core/test_fabrics.hpp"
#include "drape/garment/shapes.hpp"
#include "drape/garment/sim_mesh_builder.hpp"
#include "drape/sim/solver.hpp"

namespace drape::sim::testing {

// One flat piece meshed at h, laid in the XZ plane: world = origin + (rest.x, 0, rest.y).
inline SimGarment flatGarment(const GarmentId& id, PatternPiece piece, double h, const FabricPhysical& f,
                              const Vec3d& origin = {0, 0, 0}) {
  Pattern p;
  p.pieces.push_back(std::move(piece));
  SimGarment g;
  g.id = id;
  g.mesh = garment::buildSimMesh(p, h);
  for (const auto& r : g.mesh.rest) g.initialPositions.push_back(origin + Vec3d(r.x(), 0, r.y()));
  g.pieceFabric = {f};
  return g;
}

inline std::vector<std::uint32_t> verticesWhere(const SimMesh& m, const std::function<bool(std::uint32_t)>& pred) {
  std::vector<std::uint32_t> out;
  for (std::uint32_t v = 0; v < m.rest.size(); ++v)
    if (pred(v)) out.push_back(v);
  return out;
}

// Steps until rmsSpeed < rms for holdFrames consecutive frames. Returns frames run, or -1 on timeout.
inline int runUntilStill(Solver& s, StepInput in, double rms, int holdFrames, double maxSeconds) {
  int still = 0;
  const int maxFrames = static_cast<int>(std::ceil(maxSeconds / in.dt));
  for (int f = 1; f <= maxFrames; ++f) {
    s.step(in);
    still = s.stats().rmsSpeed < rms ? still + 1 : 0;
    if (still >= holdFrames) return f;
  }
  return -1;
}

inline std::vector<Vec3f> positions(const Solver& s) {
  std::vector<Vec3f> out(s.particleCount());
  s.readPositions(out);
  return out;
}

// A 0.5 x 0.5 m testJersey sheet (h = 1 cm) hanging in the XY plane from its two top corners.
struct HangingSheet {
  SimScene scene;
  std::uint32_t bottomLeft = 0, bottomRight = 0;
};

inline HangingSheet hangingSheet() {
  HangingSheet hs;
  SimGarment g;
  Pattern p;
  p.pieces.push_back(garment::rectanglePiece("sheet", 0.5, 0.5));
  g.id = "sheet";
  g.mesh = garment::buildSimMesh(p, 0.01);
  for (const auto& r : g.mesh.rest) g.initialPositions.push_back(Vec3d(r.x(), 0.5 + r.y(), 0));
  g.pieceFabric = {testdata::testJersey()};
  const SimMesh& m = g.mesh;
  std::uint32_t tl = 0, tr = 0;
  for (std::uint32_t v = 0; v < m.rest.size(); ++v) {
    const Vec2d r = m.rest[v];
    if ((r - Vec2d(0, 0.5)).norm() < 1e-9) tl = v;
    if ((r - Vec2d(0.5, 0.5)).norm() < 1e-9) tr = v;
    if ((r - Vec2d(0, 0)).norm() < 1e-9) hs.bottomLeft = v;
    if ((r - Vec2d(0.5, 0)).norm() < 1e-9) hs.bottomRight = v;
  }
  hs.scene.pins.push_back({tl, g.initialPositions[tl]});
  hs.scene.pins.push_back({tr, g.initialPositions[tr]});
  hs.scene.garments.push_back(std::move(g));
  return hs;
}

}  // namespace drape::sim::testing
