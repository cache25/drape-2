#include "drape/sim/cpu_solver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <map>

#include "bending.hpp"
#include "coloring.hpp"
#include "stretch.hpp"

namespace drape::sim {

int substepsFor(SimQuality q) {
  switch (q) {
    case SimQuality::Draft: return 2;
    case SimQuality::Standard: return 2;
    case SimQuality::Fine: return 4;
  }
  return 2;
}

int iterationsFor(SimQuality q) {
  switch (q) {
    case SimQuality::Draft: return 5;
    case SimQuality::Standard: return 10;
    case SimQuality::Fine: return 10;
  }
  return 10;
}

struct CpuSolver::Impl {
  struct Garment {
    GarmentId id;
    ParticleRange range;
    SimPhase phase = SimPhase::Settling;
    double assemblyTime = 0;
    std::uint32_t fabricOffset = 0;
    std::uint32_t pieceCount = 0;
  };

  SimSettings settings;
  const CollisionField* body = nullptr;
  std::vector<Garment> garments;
  std::vector<FabricPhysical> fabrics;  // garment.fabricOffset + piece index

  std::vector<Vec3f> x, v, xPrev, y, fExt;
  std::vector<float> w;
  std::vector<double> vertexArea;
  std::vector<std::uint32_t> pFabric, pGarment;
  std::vector<std::uint8_t> pinned;
  std::vector<std::pair<std::uint32_t, Vec3f>> pins;

  std::vector<StretchElement> stretch;
  // Per-vertex incident stretch elements (CSR): incStretch[incStretchStart[i] .. incStretchStart[i+1]).
  std::vector<std::uint32_t> incStretchStart;
  std::vector<std::pair<std::uint32_t, std::uint8_t>> incStretch;  // (element, local vertex)
  std::vector<Hinge> hinges;
  std::vector<std::uint32_t> hingeFabricPiece;  // fabric index per hinge (for updateFabric)
  std::vector<std::array<double, 2>> hingeGeom;  // per hinge: |e|^2/(A1+A2), and the edge's bend coefficient selector
  std::vector<Vec2d> hingeEdgeDir, hingeGrain;
  std::vector<std::uint32_t> incHingeStart;
  std::vector<std::pair<std::uint32_t, std::uint8_t>> incHinge;  // (hinge, local vertex)
  std::vector<std::vector<std::uint32_t>> colors;  // vertices per color

  float hingeStiffness(std::size_t hgi) const {
    const auto& f = fabrics[hingeFabricPiece[hgi]];
    const double B = edgeBendStiffness(bendingCoefficients(f.bendWarp, f.bendWeft), hingeEdgeDir[hgi], hingeGrain[hgi]);
    return static_cast<float>(B * hingeGeom[hgi][0]);
  }

  std::size_t garmentIndex(const GarmentId& id) const {
    for (std::size_t i = 0; i < garments.size(); ++i)
      if (garments[i].id == id) return i;
    throw std::out_of_range("unknown garment '" + id + "'");
  }

  void updateMass(std::uint32_t i) {
    const double m = vertexArea[i] * fabrics[pFabric[i]].weight;
    w[i] = pinned[i] ? 0.0f : static_cast<float>(1.0 / std::max(m, 1e-12));
  }

  // Newton step for one vertex on inertia + incident energies (Gauss-Newton Hessian).
  void solveVertex(std::uint32_t i, float invH2) {
    const float m = 1.0f / w[i];
    Vec3f force = m * invH2 * (y[i] - x[i]);
    Eigen::Matrix3f hessian = (m * invH2) * Eigen::Matrix3f::Identity();
    const float limitScale = static_cast<float>(settings.strainLimitStiffnessScale);
    for (std::uint32_t k = incStretchStart[i]; k < incStretchStart[i + 1]; ++k) {
      const auto& [e, local] = incStretch[k];
      accumulateStretch(stretch[e], local, x, fabrics[stretch[e].fabric], limitScale, force, hessian);
    }
    for (std::uint32_t k = incHingeStart[i]; k < incHingeStart[i + 1]; ++k) {
      const auto& [hg, local] = incHinge[k];
      accumulateHinge(hinges[hg], local, x, force, hessian);
    }
    const Vec3f dx = hessian.ldlt().solve(force);
    if (dx.allFinite()) x[i] += dx;
  }

  void substep(float h, int iterations) {
    const float invH2 = 1.0f / (h * h);
    const Vec3f gravity = settings.gravity.cast<float>();
    const std::size_t n = x.size();
    for (std::size_t i = 0; i < n; ++i) {
      xPrev[i] = x[i];
      y[i] = x[i];
      if (w[i] == 0.0f) continue;
      const Vec3f g = garments[pGarment[i]].phase == SimPhase::Settling ? gravity : Vec3f::Zero();
      y[i] = x[i] + h * v[i] + h * h * (g + w[i] * fExt[i]);
      // Initial guess leaves out all accelerations (gravity and loads stay in the inertia target y): at rest
      // the guess is then the equilibrium itself, so partial iterations cannot bias the resting shape.
      // (VBD's adaptive guess, gravity scaled by last substep's acceleration, oscillated on hanging cloth.)
      x[i] = x[i] + h * v[i];
    }
    for (int it = 0; it < iterations; ++it) {
      for (const auto& color : colors) {
        for (auto i : color) {
          if (w[i] != 0.0f) solveVertex(i, invH2);
        }
      }
    }
    for (const auto& [i, p] : pins) x[i] = p;
    for (std::size_t i = 0; i < n; ++i) {
      v[i] = (x[i] - xPrev[i]) / h;
      const auto& g = garments[pGarment[i]];
      double c = settings.dampingScale * fabrics[pFabric[i]].damping;
      if (g.phase == SimPhase::Assembling) c += settings.assemblyExtraDamping;
      v[i] *= static_cast<float>(std::max(0.0, 1.0 - c * h));
    }
    for (auto& g : garments) {
      if (g.phase == SimPhase::Assembling) g.assemblyTime += h;
    }
  }
};

CpuSolver::CpuSolver() : impl_(std::make_unique<Impl>()) {}
CpuSolver::~CpuSolver() = default;

void CpuSolver::build(const SimScene& scene) {
  Impl next;
  next.settings = scene.settings;
  next.body = scene.body;
  std::vector<std::array<std::uint32_t, 4>> triItems;
  for (const auto& sg : scene.garments) {
    if (sg.initialPositions.size() != sg.mesh.rest.size()) {
      throw std::invalid_argument("garment '" + sg.id + "': one initial position per mesh vertex required");
    }
    if (sg.pieceFabric.size() != sg.mesh.pieceGrain.size()) {
      throw std::invalid_argument("garment '" + sg.id + "': one fabric per pattern piece required");
    }
    Impl::Garment g;
    g.id = sg.id;
    g.range = {static_cast<std::uint32_t>(next.x.size()), static_cast<std::uint32_t>(sg.mesh.rest.size())};
    g.fabricOffset = static_cast<std::uint32_t>(next.fabrics.size());
    g.pieceCount = static_cast<std::uint32_t>(sg.pieceFabric.size());
    const auto gi = static_cast<std::uint32_t>(next.garments.size());
    next.fabrics.insert(next.fabrics.end(), sg.pieceFabric.begin(), sg.pieceFabric.end());
    for (std::size_t v = 0; v < sg.mesh.rest.size(); ++v) {
      next.x.push_back(sg.initialPositions[v].cast<float>());
      next.vertexArea.push_back(sg.mesh.vertexArea[v]);
      next.pFabric.push_back(g.fabricOffset + sg.mesh.vertexPiece[v]);
      next.pGarment.push_back(gi);
    }
    for (std::size_t t = 0; t < sg.mesh.triangles.size(); ++t) {
      const auto& tri = sg.mesh.triangles[t];
      const auto piece = sg.mesh.trianglePiece[t];
      const Vec2d u = sg.mesh.pieceGrain[piece];
      const Vec2d vdir(-u.y(), u.x());
      Vec2d r[3];
      for (int k = 0; k < 3; ++k) r[k] = Vec2d(sg.mesh.rest[tri[k]].dot(u), sg.mesh.rest[tri[k]].dot(vdir));
      Eigen::Matrix2d dm;
      dm << (r[1] - r[0]).x(), (r[2] - r[0]).x(), (r[1] - r[0]).y(), (r[2] - r[0]).y();
      StretchElement e;
      for (int k = 0; k < 3; ++k) e.v[k] = g.range.offset + tri[k];
      e.dmInv = dm.inverse().cast<float>();
      e.area = static_cast<float>(0.5 * std::abs(dm.determinant()));
      e.fabric = g.fabricOffset + piece;
      next.stretch.push_back(e);
      triItems.push_back({e.v[0], e.v[1], e.v[2], kNoParticle});
    }
    // Hinges: one per interior edge shared by two triangles of the same piece.
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::uint32_t>> edgeTris;
    for (std::uint32_t t = 0; t < sg.mesh.triangles.size(); ++t) {
      const auto& tri = sg.mesh.triangles[t];
      for (int k = 0; k < 3; ++k) edgeTris[std::minmax(tri[k], tri[(k + 1) % 3])].push_back(t);
    }
    auto triArea = [&](std::uint32_t t) {
      const auto& tri = sg.mesh.triangles[t];
      const Vec2d a = sg.mesh.rest[tri[1]] - sg.mesh.rest[tri[0]];
      const Vec2d b = sg.mesh.rest[tri[2]] - sg.mesh.rest[tri[0]];
      return 0.5 * std::abs(a.x() * b.y() - a.y() * b.x());
    };
    auto opposite = [&](std::uint32_t t, std::uint32_t e0, std::uint32_t e1) {
      for (auto vtx : sg.mesh.triangles[t])
        if (vtx != e0 && vtx != e1) return vtx;
      return e0;
    };
    for (const auto& [edge, tris] : edgeTris) {
      if (tris.size() != 2) continue;
      if (sg.mesh.trianglePiece[tris[0]] != sg.mesh.trianglePiece[tris[1]]) continue;
      const auto piece = sg.mesh.trianglePiece[tris[0]];
      const Vec2d d = sg.mesh.rest[edge.second] - sg.mesh.rest[edge.first];
      Hinge hg;
      hg.v = {g.range.offset + opposite(tris[0], edge.first, edge.second),
              g.range.offset + opposite(tris[1], edge.first, edge.second), g.range.offset + edge.first,
              g.range.offset + edge.second};
      next.hinges.push_back(hg);
      next.hingeFabricPiece.push_back(g.fabricOffset + piece);
      next.hingeGeom.push_back({d.squaredNorm() / (triArea(tris[0]) + triArea(tris[1])), 0.0});
      next.hingeEdgeDir.push_back(d);
      next.hingeGrain.push_back(sg.mesh.pieceGrain[piece]);
      triItems.push_back(hg.v);
    }
    next.garments.push_back(g);
  }
  const std::size_t n = next.x.size();
  next.v.assign(n, Vec3f::Zero());
  next.xPrev = next.x;
  next.y = next.x;
  next.fExt.assign(n, Vec3f::Zero());
  next.pinned.assign(n, 0);
  next.w.assign(n, 0.0f);
  for (const auto& pin : scene.pins) {
    if (pin.particle >= n) throw std::out_of_range("pin particle out of range");
    next.pinned[pin.particle] = 1;
    next.pins.push_back({pin.particle, pin.position.cast<float>()});
    next.x[pin.particle] = pin.position.cast<float>();
  }
  next.xPrev = next.x;
  for (const auto& f : scene.forces) {
    if (f.particle >= n) throw std::out_of_range("force particle out of range");
    next.fExt[f.particle] += f.force.cast<float>();
  }
  for (std::uint32_t i = 0; i < n; ++i) next.updateMass(i);
  next.incStretchStart.assign(n + 1, 0);
  for (const auto& e : next.stretch)
    for (auto vtx : e.v) ++next.incStretchStart[vtx + 1];
  for (std::size_t i = 0; i < n; ++i) next.incStretchStart[i + 1] += next.incStretchStart[i];
  next.incStretch.resize(next.incStretchStart[n]);
  {
    std::vector<std::uint32_t> fill(next.incStretchStart.begin(), next.incStretchStart.end() - 1);
    for (std::uint32_t e = 0; e < next.stretch.size(); ++e)
      for (std::uint8_t k = 0; k < 3; ++k) next.incStretch[fill[next.stretch[e].v[k]]++] = {e, k};
  }
  for (std::size_t hgi = 0; hgi < next.hinges.size(); ++hgi) next.hinges[hgi].K = next.hingeStiffness(hgi);
  next.incHingeStart.assign(n + 1, 0);
  for (const auto& hg : next.hinges)
    for (auto vtx : hg.v) ++next.incHingeStart[vtx + 1];
  for (std::size_t i = 0; i < n; ++i) next.incHingeStart[i + 1] += next.incHingeStart[i];
  next.incHinge.resize(next.incHingeStart[n]);
  {
    std::vector<std::uint32_t> fill(next.incHingeStart.begin(), next.incHingeStart.end() - 1);
    for (std::uint32_t h = 0; h < next.hinges.size(); ++h)
      for (std::uint8_t k = 0; k < 4; ++k) next.incHinge[fill[next.hinges[h].v[k]]++] = {h, k};
  }
  next.colors = greedyVertexColor(triItems, static_cast<std::uint32_t>(n));
  *impl_ = std::move(next);
}

void CpuSolver::updateFabric(const GarmentId& id, const std::vector<FabricPhysical>& pieceFabric) {
  auto& g = impl_->garments[impl_->garmentIndex(id)];
  if (pieceFabric.size() != g.pieceCount) throw std::invalid_argument("one fabric per pattern piece required");
  std::copy(pieceFabric.begin(), pieceFabric.end(), impl_->fabrics.begin() + g.fabricOffset);
  for (std::uint32_t i = g.range.offset; i < g.range.offset + g.range.count; ++i) impl_->updateMass(i);
  for (std::size_t hgi = 0; hgi < impl_->hinges.size(); ++hgi) {
    const auto fi = impl_->hingeFabricPiece[hgi];
    if (fi >= g.fabricOffset && fi < g.fabricOffset + g.pieceCount) impl_->hinges[hgi].K = impl_->hingeStiffness(hgi);
  }
}

void CpuSolver::setPhase(const GarmentId& id, SimPhase phase) {
  auto& g = impl_->garments[impl_->garmentIndex(id)];
  g.phase = phase;
  if (phase == SimPhase::Assembling) g.assemblyTime = 0;
}

void CpuSolver::step(const StepInput& in) {
  const int substeps = std::max(1, in.substeps);
  const float h = static_cast<float>(in.dt / substeps);
  const int iterations = std::max(1, in.iterations);
  for (int s = 0; s < substeps; ++s) impl_->substep(h, iterations);
}

void CpuSolver::readPositions(std::span<Vec3f> out) const {
  if (out.size() != impl_->x.size()) throw std::invalid_argument("readPositions: span size must equal particleCount()");
  std::copy(impl_->x.begin(), impl_->x.end(), out.begin());
}

SimStats CpuSolver::stats() const {
  const auto& s = *impl_;
  SimStats st;
  double sumV2 = 0;
  std::size_t moving = 0;
  for (std::size_t i = 0; i < s.x.size(); ++i) {
    if (!s.x[i].allFinite() || !s.v[i].allFinite()) st.hasNonFinite = true;
    if (s.w[i] == 0.0f) continue;
    const double v2 = s.v[i].cast<double>().squaredNorm();
    st.kineticEnergy += 0.5 * v2 / s.w[i];
    sumV2 += v2;
    ++moving;
  }
  st.rmsSpeed = moving ? std::sqrt(sumV2 / static_cast<double>(moving)) : 0.0;
  return st;
}

Snapshot CpuSolver::snapshot() const {
  Snapshot snap;
  snap.positions = impl_->x;
  snap.velocities = impl_->v;
  for (const auto& g : impl_->garments) {
    snap.phases.push_back(g.phase);
    snap.assemblyTime.push_back(g.assemblyTime);
  }
  return snap;
}

void CpuSolver::restore(const Snapshot& snap) {
  auto& s = *impl_;
  if (snap.positions.size() != s.x.size() || snap.phases.size() != s.garments.size()) {
    throw std::invalid_argument("snapshot does not match this scene");
  }
  s.x = snap.positions;
  s.v = snap.velocities;
  s.xPrev = s.x;
  s.y = s.x;
  for (std::size_t i = 0; i < s.garments.size(); ++i) {
    s.garments[i].phase = snap.phases[i];
    s.garments[i].assemblyTime = snap.assemblyTime[i];
  }
}

ParticleRange CpuSolver::range(const GarmentId& id) const { return impl_->garments[impl_->garmentIndex(id)].range; }

std::size_t CpuSolver::particleCount() const { return impl_->x.size(); }

}  // namespace drape::sim
