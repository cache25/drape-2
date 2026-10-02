#include "drape/sim/cpu_solver.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

#include "bending.hpp"
#include "collision.hpp"
#include "energies.hpp"
#include "seams.hpp"

namespace drape::sim {

int substepsFor(SimQuality q) {
  switch (q) {
    case SimQuality::Draft: return 1;
    case SimQuality::Standard: return 1;
    case SimQuality::Fine: return 2;
  }
  return 1;
}

int iterationsFor(SimQuality q) {
  switch (q) {
    case SimQuality::Draft: return 1;
    case SimQuality::Standard: return 2;
    case SimQuality::Fine: return 2;
  }
  return 2;
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
  struct HingeSide {
    std::uint32_t fabric = 0;
    Vec2d edgeDir{1, 0}, grain{0, 1};
  };

  SimSettings settings;
  const CollisionField* body = nullptr;
  std::vector<Garment> garments;
  std::vector<FabricPhysical> fabrics;  // garment.fabricOffset + piece index

  // Particle state.
  std::vector<Vec3f> x, v;
  std::vector<double> mass, vertexArea;
  std::vector<Vec3d> fExt;
  std::vector<std::uint32_t> pFabric, pGarment;
  std::vector<std::uint8_t> pinned;
  std::vector<std::pair<std::uint32_t, Vec3f>> pins;

  // Energy terms.
  std::vector<StretchTerm> stretch;
  std::vector<HingeTerm> hinges;
  std::vector<std::array<HingeSide, 2>> hingeSides;
  std::vector<double> hingeGeom;  // |e|^2 / (A1 + A2)
  std::vector<SeamSpring> seams;

  // Block-sparse Hessian structure (CSR over 3x3 blocks) and each term's block slots.
  std::vector<std::uint32_t> rowStart, colIndex, diagSlot;
  std::vector<std::array<std::uint32_t, 9>> stretchSlots;
  std::vector<std::array<std::uint32_t, 16>> hingeSlots;
  std::vector<std::array<std::uint32_t, 4>> seamSlots;

  // Solver work arrays (double).
  std::vector<V3> xd, xStart, y, grad, step, r, z, p, ap, trial;
  std::vector<M3> blocks, precond;

  std::size_t garmentIndex(const GarmentId& id) const {
    for (std::size_t i = 0; i < garments.size(); ++i)
      if (garments[i].id == id) return i;
    throw std::out_of_range("unknown garment '" + id + "'");
  }

  void updateMass(std::uint32_t i) { mass[i] = std::max(vertexArea[i] * fabrics[pFabric[i]].weight, 1e-12); }

  double hingeStiffness(std::size_t hgi) const {
    double B = 0;
    for (const auto& side : hingeSides[hgi]) {
      const auto& f = fabrics[side.fabric];
      B += 0.5 * edgeBendStiffness(bendingCoefficients(f.bendWarp, f.bendWeft), side.edgeDir, side.grain);
    }
    return B * hingeGeom[hgi];
  }

  double seamTarget(const SeamSpring& sp) const {
    const auto& g = garments[sp.garment];
    if (g.phase != SimPhase::Assembling) return 0.0;
    return std::max(0.0, static_cast<double>(sp.startGap) - settings.assemblyClosingSpeed * g.assemblyTime);
  }

  std::uint32_t slot(std::uint32_t row, std::uint32_t col) const {
    const auto begin = colIndex.begin() + rowStart[row];
    const auto end = colIndex.begin() + rowStart[row + 1];
    return static_cast<std::uint32_t>(std::lower_bound(begin, end, col) - colIndex.begin());
  }

  void buildStructure() {
    const std::size_t n = x.size();
    std::vector<std::vector<std::uint32_t>> cols(n);
    auto couple = [&](const auto& vs) {
      for (auto a : vs)
        for (auto b : vs) cols[a].push_back(b);
    };
    for (std::size_t i = 0; i < n; ++i) cols[i].push_back(static_cast<std::uint32_t>(i));
    for (const auto& t : stretch) couple(t.v);
    for (const auto& h : hinges) couple(h.v);
    for (const auto& s : seams) couple(std::array<std::uint32_t, 2>{s.a, s.b});
    rowStart.assign(n + 1, 0);
    colIndex.clear();
    for (std::size_t i = 0; i < n; ++i) {
      std::sort(cols[i].begin(), cols[i].end());
      cols[i].erase(std::unique(cols[i].begin(), cols[i].end()), cols[i].end());
      colIndex.insert(colIndex.end(), cols[i].begin(), cols[i].end());
      rowStart[i + 1] = static_cast<std::uint32_t>(colIndex.size());
    }
    diagSlot.resize(n);
    for (std::uint32_t i = 0; i < n; ++i) diagSlot[i] = slot(i, i);
    auto slotsFor = [&](const auto& vs, auto& out) {
      const int m = static_cast<int>(vs.size());
      for (int a = 0; a < m; ++a)
        for (int b = 0; b < m; ++b) out[a * m + b] = slot(vs[a], vs[b]);
    };
    stretchSlots.resize(stretch.size());
    for (std::size_t k = 0; k < stretch.size(); ++k) slotsFor(stretch[k].v, stretchSlots[k]);
    hingeSlots.resize(hinges.size());
    for (std::size_t k = 0; k < hinges.size(); ++k) slotsFor(hinges[k].v, hingeSlots[k]);
    seamSlots.resize(seams.size());
    for (std::size_t k = 0; k < seams.size(); ++k) slotsFor(std::array<std::uint32_t, 2>{seams[k].a, seams[k].b}, seamSlots[k]);
    blocks.assign(colIndex.size(), M3::Zero());
    precond.assign(n, M3::Identity());
    for (auto* vec : {&xd, &xStart, &y, &grad, &step, &r, &z, &p, &ap, &trial}) vec->assign(n, V3::Zero());
  }

  // Total energy at `pos`; with withDerivatives also fills grad and the block Hessian.
  template <std::size_t N, class Term, class Slots, class Eval>
  void addTerm(const Term& vs, const Slots& slots, const std::vector<V3>& pos, bool withDerivatives, double& energy,
               Eval&& eval) {
    std::array<V3, N> xs;
    for (std::size_t a = 0; a < N; ++a) xs[a] = pos[vs[a]];
    TermEval<N> te;
    te.clear();
    eval(xs, te);
    energy += te.energy;
    if (!withDerivatives) return;
    for (std::size_t a = 0; a < N; ++a) {
      grad[vs[a]] += te.grad[a];
      for (std::size_t b = 0; b < N; ++b) blocks[slots[a * N + b]] += te.hess[a][b];
    }
  }

  double evaluate(const std::vector<V3>& pos, double invH2, bool withDerivatives) {
    const std::size_t n = pos.size();
    double energy = 0;
    if (withDerivatives) {
      std::fill(grad.begin(), grad.end(), V3::Zero());
      std::fill(blocks.begin(), blocks.end(), M3::Zero());
    }
    for (std::size_t i = 0; i < n; ++i) {
      if (pinned[i]) continue;
      const double k = mass[i] * invH2;
      const V3 d = pos[i] - y[i];
      energy += 0.5 * k * d.squaredNorm();
      if (withDerivatives) {
        grad[i] += k * d;
        blocks[diagSlot[i]] += k * M3::Identity();
      }
    }
    const double limitScale = settings.strainLimitStiffnessScale;
    for (std::size_t k = 0; k < stretch.size(); ++k) {
      const auto& t = stretch[k];
      addTerm<3>(t.v, stretchSlots[k], pos, withDerivatives, energy, [&](const auto& xs, auto& te) {
        evalStretchTerm(t, xs, fabrics[t.fabric], limitScale, te);
      });
    }
    for (std::size_t k = 0; k < hinges.size(); ++k) {
      const auto& h = hinges[k];
      addTerm<4>(h.v, hingeSlots[k], pos, withDerivatives, energy,
                 [&](const auto& xs, auto& te) { evalHingeTerm(h, xs, te); });
    }
    for (std::size_t k = 0; k < seams.size(); ++k) {
      const auto& s = seams[k];
      const double target = seamTarget(s);
      addTerm<2>(std::array<std::uint32_t, 2>{s.a, s.b}, seamSlots[k], pos, withDerivatives, energy,
                 [&](const auto& xs, auto& te) { evalSeamTerm(xs, target, settings.seamStiffness, te); });
    }
    if (withDerivatives) {
      for (std::size_t i = 0; i < n; ++i)
        if (pinned[i]) grad[i].setZero();
    }
    return energy;
  }

  // out = A in, over free particles only (pinned rows and columns are excluded).
  void multiply(const std::vector<V3>& in, std::vector<V3>& out) const {
    const std::size_t n = in.size();
    for (std::size_t i = 0; i < n; ++i) {
      V3 acc = V3::Zero();
      if (!pinned[i]) {
        for (std::uint32_t s = rowStart[i]; s < rowStart[i + 1]; ++s) {
          const auto j = colIndex[s];
          if (!pinned[j]) acc += blocks[s] * in[j];
        }
      }
      out[i] = acc;
    }
  }

  static double dot(const std::vector<V3>& a, const std::vector<V3>& b) {
    double s = 0;
    for (std::size_t i = 0; i < a.size(); ++i) s += a[i].dot(b[i]);
    return s;
  }

  // Solves A step = -grad with block-Jacobi preconditioned conjugate gradients.
  void solveLinear() {
    const std::size_t n = grad.size();
    for (std::size_t i = 0; i < n; ++i) {
      if (pinned[i]) {
        precond[i].setZero();
        continue;
      }
      Eigen::LDLT<M3> ldlt(blocks[diagSlot[i]]);
      precond[i] = ldlt.info() == Eigen::Success ? M3(ldlt.solve(M3::Identity())) : M3::Identity();
    }
    for (std::size_t i = 0; i < n; ++i) {
      step[i].setZero();
      r[i] = -grad[i];
      z[i] = precond[i] * r[i];
      p[i] = z[i];
    }
    double rz = dot(r, z);
    const double r0 = std::sqrt(dot(r, r));
    if (r0 == 0) return;
    for (int it = 0; it < settings.maxCgIterations; ++it) {
      multiply(p, ap);
      const double pAp = dot(p, ap);
      if (!(pAp > 0)) break;
      const double alpha = rz / pAp;
      for (std::size_t i = 0; i < n; ++i) {
        step[i] += alpha * p[i];
        r[i] -= alpha * ap[i];
      }
      if (std::sqrt(dot(r, r)) <= settings.cgTolerance * r0) break;
      for (std::size_t i = 0; i < n; ++i) z[i] = precond[i] * r[i];
      const double rzNew = dot(r, z);
      const double beta = rzNew / rz;
      rz = rzNew;
      for (std::size_t i = 0; i < n; ++i) p[i] = z[i] + beta * p[i];
    }
  }

  void substep(double h, int newtonIterations) {
    const double invH2 = 1.0 / (h * h);
    const Vec3d gravity = settings.gravity;
    const std::size_t n = x.size();
    for (std::size_t i = 0; i < n; ++i) {
      xStart[i] = x[i].cast<double>();
      const V3 vel = v[i].cast<double>();
      if (pinned[i]) {
        y[i] = xStart[i];
        xd[i] = xStart[i];
        continue;
      }
      const V3 g = garments[pGarment[i]].phase == SimPhase::Settling ? gravity : V3::Zero();
      y[i] = xStart[i] + h * vel + h * h * (g + fExt[i] / mass[i]);
      xd[i] = xStart[i] + h * vel;
    }
    for (int it = 0; it < newtonIterations; ++it) {
      const double e0 = evaluate(xd, invH2, true);
      solveLinear();
      const double slope = dot(grad, step);
      if (!(slope < 0)) break;
      double alpha = 1.0;
      for (int ls = 0; ls < 20; ++ls) {
        for (std::size_t i = 0; i < n; ++i) trial[i] = xd[i] + alpha * step[i];
        if (evaluate(trial, invH2, false) <= e0 + 1e-4 * alpha * slope) break;
        alpha *= 0.5;
      }
      double maxMove = 0;
      for (std::size_t i = 0; i < n; ++i) {
        xd[i] += alpha * step[i];
        maxMove = std::max(maxMove, (alpha * step[i]).cwiseAbs().maxCoeff());
      }
      if (maxMove < 1e-9) break;
    }
    for (std::size_t i = 0; i < n; ++i) x[i] = xd[i].cast<float>();
    if (body) {
      for (std::size_t i = 0; i < n; ++i) {
        if (pinned[i]) continue;
        const auto& f = fabrics[pFabric[i]];
        const float radius = static_cast<float>(settings.collisionOffset + 0.5 * f.thickness);
        collideBody(*body, radius, static_cast<float>(f.friction), xStart[i].cast<float>(), x[i]);
      }
    }
    for (const auto& [i, pos] : pins) x[i] = pos;
    for (std::size_t i = 0; i < n; ++i) {
      v[i] = ((x[i].cast<double>() - xStart[i]) / h).cast<float>();
      double c = settings.dampingScale * fabrics[pFabric[i]].damping;
      if (garments[pGarment[i]].phase == SimPhase::Assembling) c += settings.assemblyExtraDamping;
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
    const std::uint32_t off = g.range.offset;
    next.fabrics.insert(next.fabrics.end(), sg.pieceFabric.begin(), sg.pieceFabric.end());
    for (std::size_t v = 0; v < sg.mesh.rest.size(); ++v) {
      next.x.push_back(sg.initialPositions[v].cast<float>());
      next.vertexArea.push_back(sg.mesh.vertexArea[v]);
      next.pFabric.push_back(g.fabricOffset + sg.mesh.vertexPiece[v]);
      next.pGarment.push_back(gi);
    }
    auto triArea = [&](std::uint32_t t) {
      const auto& tri = sg.mesh.triangles[t];
      const Vec2d a = sg.mesh.rest[tri[1]] - sg.mesh.rest[tri[0]];
      const Vec2d b = sg.mesh.rest[tri[2]] - sg.mesh.rest[tri[0]];
      return 0.5 * std::abs(a.x() * b.y() - a.y() * b.x());
    };
    // Stretch terms in each piece's grain frame.
    for (std::uint32_t t = 0; t < sg.mesh.triangles.size(); ++t) {
      const auto& tri = sg.mesh.triangles[t];
      const auto piece = sg.mesh.trianglePiece[t];
      const Vec2d u = sg.mesh.pieceGrain[piece];
      const Vec2d vdir(-u.y(), u.x());
      Vec2d rr[3];
      for (int k = 0; k < 3; ++k) rr[k] = Vec2d(sg.mesh.rest[tri[k]].dot(u), sg.mesh.rest[tri[k]].dot(vdir));
      Eigen::Matrix2d dm;
      dm << (rr[1] - rr[0]).x(), (rr[2] - rr[0]).x(), (rr[1] - rr[0]).y(), (rr[2] - rr[0]).y();
      StretchTerm st;
      for (int k = 0; k < 3; ++k) st.v[k] = off + tri[k];
      st.dmInv = dm.inverse();
      st.area = 0.5 * std::abs(dm.determinant());
      st.fabric = g.fabricOffset + piece;
      next.stretch.push_back(st);
    }
    // Hinges: one per interior edge shared by two triangles of the same piece.
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::uint32_t>> edgeTris;
    for (std::uint32_t t = 0; t < sg.mesh.triangles.size(); ++t) {
      const auto& tri = sg.mesh.triangles[t];
      for (int k = 0; k < 3; ++k) edgeTris[std::minmax(tri[k], tri[(k + 1) % 3])].push_back(t);
    }
    auto opposite = [&](std::uint32_t t, std::uint32_t e0, std::uint32_t e1) {
      for (auto vtx : sg.mesh.triangles[t])
        if (vtx != e0 && vtx != e1) return vtx;
      return e0;
    };
    for (const auto& [edge, tris] : edgeTris) {
      if (tris.size() != 2 || sg.mesh.trianglePiece[tris[0]] != sg.mesh.trianglePiece[tris[1]]) continue;
      const auto piece = sg.mesh.trianglePiece[tris[0]];
      const Vec2d d = sg.mesh.rest[edge.second] - sg.mesh.rest[edge.first];
      HingeTerm hg;
      hg.v = {off + opposite(tris[0], edge.first, edge.second), off + opposite(tris[1], edge.first, edge.second),
              off + edge.first, off + edge.second};
      next.hinges.push_back(hg);
      const Impl::HingeSide side{g.fabricOffset + piece, d, sg.mesh.pieceGrain[piece]};
      next.hingeSides.push_back({side, side});
      next.hingeGeom.push_back(d.squaredNorm() / (triArea(tris[0]) + triArea(tris[1])));
    }
    // Seam springs, plus seam hinges across consecutive pairs so seams do not act as free hinges.
    auto triangleWithEdge = [&](std::uint32_t e0, std::uint32_t e1) -> std::int64_t {
      auto it = edgeTris.find(std::minmax(e0, e1));
      return (it == edgeTris.end() || it->second.size() != 1) ? -1 : it->second[0];
    };
    for (std::size_t k = 0; k < sg.mesh.seamPairs.size(); ++k) {
      const auto& sp = sg.mesh.seamPairs[k];
      SeamSpring spring;
      spring.a = off + sp.a;
      spring.b = off + sp.b;
      spring.garment = gi;
      spring.startGap = static_cast<float>((sg.initialPositions[sp.a] - sg.initialPositions[sp.b]).norm());
      next.seams.push_back(spring);
      if (k + 1 >= sg.mesh.seamPairs.size() || sg.mesh.seamPairs[k + 1].seam != sp.seam) continue;
      const auto& nx = sg.mesh.seamPairs[k + 1];
      const auto tA = triangleWithEdge(sp.a, nx.a);
      const auto tB = triangleWithEdge(sp.b, nx.b);
      if (tA < 0 || tB < 0) continue;
      const auto ta = static_cast<std::uint32_t>(tA), tb = static_cast<std::uint32_t>(tB);
      const auto pieceA = sg.mesh.trianglePiece[ta], pieceB = sg.mesh.trianglePiece[tb];
      const Vec2d dA = sg.mesh.rest[nx.a] - sg.mesh.rest[sp.a];
      const Vec2d dB = sg.mesh.rest[nx.b] - sg.mesh.rest[sp.b];
      HingeTerm hg;
      hg.v = {off + opposite(ta, sp.a, nx.a), off + opposite(tb, sp.b, nx.b), off + sp.a, off + nx.a};
      next.hinges.push_back(hg);
      next.hingeSides.push_back({Impl::HingeSide{g.fabricOffset + pieceA, dA, sg.mesh.pieceGrain[pieceA]},
                                 Impl::HingeSide{g.fabricOffset + pieceB, dB, sg.mesh.pieceGrain[pieceB]}});
      next.hingeGeom.push_back(dA.squaredNorm() / (triArea(ta) + triArea(tb)));
    }
    next.garments.push_back(g);
  }
  const std::size_t n = next.x.size();
  next.v.assign(n, Vec3f::Zero());
  next.fExt.assign(n, Vec3d::Zero());
  next.pinned.assign(n, 0);
  next.mass.assign(n, 0.0);
  for (const auto& pin : scene.pins) {
    if (pin.particle >= n) throw std::out_of_range("pin particle out of range");
    next.pinned[pin.particle] = 1;
    next.pins.push_back({pin.particle, pin.position.cast<float>()});
    next.x[pin.particle] = pin.position.cast<float>();
  }
  for (const auto& f : scene.forces) {
    if (f.particle >= n) throw std::out_of_range("force particle out of range");
    next.fExt[f.particle] += f.force;
  }
  for (std::uint32_t i = 0; i < n; ++i) next.updateMass(i);
  for (std::size_t hgi = 0; hgi < next.hinges.size(); ++hgi) next.hinges[hgi].K = next.hingeStiffness(hgi);
  next.buildStructure();
  *impl_ = std::move(next);
}

void CpuSolver::updateFabric(const GarmentId& id, const std::vector<FabricPhysical>& pieceFabric) {
  auto& g = impl_->garments[impl_->garmentIndex(id)];
  if (pieceFabric.size() != g.pieceCount) throw std::invalid_argument("one fabric per pattern piece required");
  std::copy(pieceFabric.begin(), pieceFabric.end(), impl_->fabrics.begin() + g.fabricOffset);
  for (std::uint32_t i = g.range.offset; i < g.range.offset + g.range.count; ++i) impl_->updateMass(i);
  for (std::size_t hgi = 0; hgi < impl_->hinges.size(); ++hgi) {
    const auto fi = impl_->hingeSides[hgi][0].fabric;
    if (fi >= g.fabricOffset && fi < g.fabricOffset + g.pieceCount) impl_->hinges[hgi].K = impl_->hingeStiffness(hgi);
  }
}

void CpuSolver::setPhase(const GarmentId& id, SimPhase phase) {
  const auto gi = impl_->garmentIndex(id);
  auto& g = impl_->garments[gi];
  g.phase = phase;
  if (phase == SimPhase::Assembling) {
    g.assemblyTime = 0;
    for (auto& sp : impl_->seams) {
      if (sp.garment == gi) sp.startGap = (impl_->x[sp.a] - impl_->x[sp.b]).norm();
    }
  }
}

void CpuSolver::step(const StepInput& in) {
  const int substeps = std::max(1, in.substeps);
  const double h = in.dt / substeps;
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
    if (s.pinned[i]) continue;
    const double v2 = s.v[i].cast<double>().squaredNorm();
    st.kineticEnergy += 0.5 * s.mass[i] * v2;
    sumV2 += v2;
    ++moving;
  }
  st.rmsSpeed = moving ? std::sqrt(sumV2 / static_cast<double>(moving)) : 0.0;
  for (const auto& sp : s.seams)
    st.maxSeamGap = std::max(st.maxSeamGap, static_cast<double>((s.x[sp.a] - s.x[sp.b]).norm()));
  if (s.body) {
    for (const auto& p : s.x) st.maxBodyPenetration = std::max(st.maxBodyPenetration, -s.body->sample(p.cast<double>()));
  }
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
  for (const auto& sp : impl_->seams) snap.seamStartGap.push_back(sp.startGap);
  return snap;
}

void CpuSolver::restore(const Snapshot& snap) {
  auto& s = *impl_;
  if (snap.positions.size() != s.x.size() || snap.phases.size() != s.garments.size()) {
    throw std::invalid_argument("snapshot does not match this scene");
  }
  s.x = snap.positions;
  s.v = snap.velocities;
  for (std::size_t i = 0; i < s.garments.size(); ++i) {
    s.garments[i].phase = snap.phases[i];
    s.garments[i].assemblyTime = snap.assemblyTime[i];
  }
  if (snap.seamStartGap.size() == s.seams.size()) {
    for (std::size_t k = 0; k < s.seams.size(); ++k) s.seams[k].startGap = snap.seamStartGap[k];
  }
}

ParticleRange CpuSolver::range(const GarmentId& id) const { return impl_->garments[impl_->garmentIndex(id)].range; }

std::size_t CpuSolver::particleCount() const { return impl_->x.size(); }

}  // namespace drape::sim
