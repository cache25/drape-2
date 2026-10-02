#pragma once

#include <cmath>
#include <functional>
#include <vector>

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

}  // namespace drape::sim::testing
