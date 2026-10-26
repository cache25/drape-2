#include "drape/sim/cpu_solver.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

#include "bending.hpp"
#include "collision.hpp"
#include "energies.hpp"
#include "seams.hpp"
#include "sim_model.hpp"

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
    case SimQuality::Draft: return 2;
    case SimQuality::Standard: return 2;
    case SimQuality::Fine: return 2;
  }
  return 2;
}

struct CpuSolver::Impl {
  SimModel m;

  // Body contact inside the solve (spec 5.5), refreshed every substep. A particle near the body gets the
  // tangent plane at its substep-start position. While active, the particle is held on that plane: Newton
  // steps are filtered to the plane (Baraff & Witkin 1998) and the normal force is read from the residual.
  // Friction is a smoothed Coulomb energy in the same solve, using the latest known normal force (previous
  // Newton iteration, or previous substep for the first one). The normal forces live in m.contactForce.
  std::vector<V3> contactNormal;
  std::vector<double> contactOffset, frictionLambda;  // offset r - phi(xStart); forces in newtons
  double substepH = 1.0 / 60.0;
  std::vector<std::uint8_t> contactNear, contactActive;

  // Solver work arrays (double).
  std::vector<V3> xd, xStart, y, grad, step, r, z, p, ap, trial, apFull, resid;
  std::vector<M3> blocks, precond;

  // Sizes the work arrays for the current model.
  void allocate() {
    const std::size_t n = m.x.size();
    blocks.assign(m.colIndex.size(), M3::Zero());
    precond.assign(n, M3::Identity());
    for (auto* vec : {&xd, &xStart, &y, &grad, &step, &r, &z, &p, &ap, &trial, &apFull, &resid})
      vec->assign(n, V3::Zero());
    contactNormal.assign(n, V3::Zero());
    contactOffset.assign(n, 0.0);
    frictionLambda.assign(n, 0.0);
    contactNear.assign(n, 0);
    contactActive.assign(n, 0);
  }

  // > 0 when `pos` lies below particle i's contact plane.
  double contactGap(std::size_t i, const V3& pos) const {
    return contactOffset[i] - contactNormal[i].dot(pos - xStart[i]);
  }
  // Removes the normal component at particles held on their contact plane.
  void filter(std::vector<V3>& vec) const {
    for (std::size_t i = 0; i < vec.size(); ++i)
      if (contactActive[i]) vec[i] -= contactNormal[i] * contactNormal[i].dot(vec[i]);
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
      if (m.pinned[i]) continue;
      const double k = m.mass[i] * invH2;
      const V3 d = pos[i] - y[i];
      energy += 0.5 * k * d.squaredNorm();
      if (withDerivatives) {
        grad[i] += k * d;
        blocks[m.diagSlot[i]] += k * M3::Identity();
      }
    }
    const double limitScale = m.settings.strainLimitStiffnessScale;
    for (std::size_t k = 0; k < m.stretch.size(); ++k) {
      const auto& t = m.stretch[k];
      addTerm<3>(t.v, m.stretchSlots[k], pos, withDerivatives, energy, [&](const auto& xs, auto& te) {
        evalStretchTerm(t, xs, m.fabrics[t.fabric], limitScale, te);
      });
    }
    for (std::size_t k = 0; k < m.hinges.size(); ++k) {
      const auto& h = m.hinges[k];
      addTerm<4>(h.v, m.hingeSlots[k], pos, withDerivatives, energy,
                 [&](const auto& xs, auto& te) { evalHingeTerm(h, xs, te); });
    }
    for (std::size_t k = 0; k < m.seams.size(); ++k) {
      const auto& s = m.seams[k];
      const double target = m.seamTarget(s);
      addTerm<2>(std::array<std::uint32_t, 2>{s.a, s.b}, m.seamSlots[k], pos, withDerivatives, energy,
                 [&](const auto& xs, auto& te) { evalSeamTerm(xs, target, m.settings.seamStiffness, te); });
    }
    for (std::size_t i = 0; i < n; ++i) {
      if (frictionLambda[i] <= 0) continue;
      TermEval<1> te;
      te.clear();
      evalFrictionTerm(pos[i], xStart[i], contactNormal[i], m.fabrics[m.pFabric[i]].friction * frictionLambda[i],
                       m.settings.frictionSlipSpeed * substepH, te);
      energy += te.energy;
      if (withDerivatives) {
        grad[i] += te.grad[0];
        blocks[m.diagSlot[i]] += te.hess[0][0];
      }
    }
    if (withDerivatives) {
      for (std::size_t i = 0; i < n; ++i)
        if (m.pinned[i]) grad[i].setZero();
    }
    return energy;
  }

  // out = A in, over free particles only (pinned rows and columns are excluded).
  void multiply(const std::vector<V3>& in, std::vector<V3>& out) const {
    const std::size_t n = in.size();
    for (std::size_t i = 0; i < n; ++i) {
      V3 acc = V3::Zero();
      if (!m.pinned[i]) {
        for (std::uint32_t s = m.rowStart[i]; s < m.rowStart[i + 1]; ++s) {
          const auto j = m.colIndex[s];
          if (!m.pinned[j]) acc += blocks[s] * in[j];
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

  // Solves A step = -grad with block-Jacobi preconditioned conjugate gradients, restricted to the contact
  // planes of active particles; then reads each active particle's normal force from the unfiltered residual.
  void solveLinear() {
    const std::size_t n = grad.size();
    for (std::size_t i = 0; i < n; ++i) {
      if (m.pinned[i]) {
        precond[i].setZero();
        continue;
      }
      Eigen::LDLT<M3> ldlt(blocks[m.diagSlot[i]]);
      precond[i] = ldlt.info() == Eigen::Success ? M3(ldlt.solve(M3::Identity())) : M3::Identity();
    }
    for (std::size_t i = 0; i < n; ++i) {
      step[i].setZero();
      resid[i] = -grad[i];
      r[i] = resid[i];
    }
    filter(r);
    for (std::size_t i = 0; i < n; ++i) z[i] = precond[i] * r[i];
    filter(z);
    p = z;
    double rz = dot(r, z);
    const double r0 = std::sqrt(dot(r, r));
    for (int it = 0; r0 > 0 && it < m.settings.maxCgIterations; ++it) {
      multiply(p, apFull);
      ap = apFull;
      filter(ap);
      const double pAp = dot(p, ap);
      if (!(pAp > 0)) break;
      const double alpha = rz / pAp;
      for (std::size_t i = 0; i < n; ++i) {
        step[i] += alpha * p[i];
        r[i] -= alpha * ap[i];
        resid[i] -= alpha * apFull[i];
      }
      if (std::sqrt(dot(r, r)) <= m.settings.cgTolerance * r0) break;
      for (std::size_t i = 0; i < n; ++i) z[i] = precond[i] * r[i];
      filter(z);
      const double rzNew = dot(r, z);
      const double beta = rzNew / rz;
      rz = rzNew;
      for (std::size_t i = 0; i < n; ++i) p[i] = z[i] + beta * p[i];
    }
    // resid is the net force left in the linear model; the body supplies whatever pushes inward.
    for (std::size_t i = 0; i < n; ++i)
      m.contactForce[i] = contactActive[i] ? std::max(0.0, -resid[i].dot(contactNormal[i])) : 0.0;
  }

  // Holds near particles on their contact planes for this Newton iteration (projecting any that are below
  // it), evaluates, then frees the ones the other forces pull off the body.
  double evaluateWithContact(double invH2) {
    for (std::size_t i = 0; i < xd.size(); ++i) {
      if (!contactNear[i]) continue;
      const double c = contactGap(i, xd[i]);
      if (c > -kContactSnap) contactActive[i] = 1;
      if (contactActive[i] && c > 0) xd[i] += c * contactNormal[i];
    }
    const double e = evaluate(xd, invH2, true);
    for (std::size_t i = 0; i < xd.size(); ++i)
      if (contactActive[i] && grad[i].dot(contactNormal[i]) < 0) contactActive[i] = 0;
    return e;
  }

  void substep(double h, int newtonIterations) {
    const double invH2 = 1.0 / (h * h);
    const Vec3d gravity = m.settings.gravity;
    const std::size_t n = m.x.size();
    for (std::size_t i = 0; i < n; ++i) {
      xStart[i] = m.x[i].cast<double>();
      const V3 vel = m.v[i].cast<double>();
      if (m.pinned[i]) {
        y[i] = xStart[i];
        xd[i] = xStart[i];
        continue;
      }
      const V3 g = m.garments[m.pGarment[i]].phase == SimPhase::Settling ? gravity : V3::Zero();
      y[i] = xStart[i] + h * vel + h * h * (g + m.fExt[i] / m.mass[i]);
      xd[i] = xStart[i] + h * vel;
    }
    substepH = h;
    for (std::size_t i = 0; i < n; ++i) {
      contactNear[i] = 0;
      contactActive[i] = 0;
      frictionLambda[i] = 0;
      const double lastForce = m.contactForce[i];
      m.contactForce[i] = 0;
      if (!m.body || m.pinned[i]) continue;
      const double radius = m.contactRadius(i);
      const double phi = m.body->sample(xStart[i]);
      if (phi >= radius + kContactReach) continue;
      const V3 normal = m.body->gradient(xStart[i]);
      if (normal.isZero()) continue;
      contactNear[i] = 1;
      contactNormal[i] = normal;
      contactOffset[i] = radius - phi;
      frictionLambda[i] = lastForce;
    }
    for (int it = 0; it < newtonIterations; ++it) {
      const double e0 = evaluateWithContact(invH2);
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
      // The next Newton iteration's friction uses this solve's normal forces, so a particle landing this substep
      // feels friction from its impact force rather than one substep late.
      for (std::size_t i = 0; i < n; ++i) frictionLambda[i] = contactActive[i] ? m.contactForce[i] : 0.0;
      if (maxMove < 1e-9) break;
    }
    for (std::size_t i = 0; i < n; ++i) m.x[i] = xd[i].cast<float>();
    if (m.body) {
      // Safety net only. The solve keeps contacts on their planes, and projecting every particle afterwards would
      // kick resting cloth by the SDF's interpolation error / h each substep. A particle that still ends deeper
      // than the snap tolerance (it reached the body during the last Newton step) is pushed out here.
      for (std::size_t i = 0; i < n; ++i) {
        if (m.pinned[i]) continue;
        const double radius = m.contactRadius(i);
        if (m.body->sample(m.x[i].cast<double>()) >= radius - kContactSnap) continue;
        if (m.body->gradient(m.x[i].cast<double>()).isZero()) {
          // Past the SDF band the field is flat and gives no direction: stop the particle where its path met the
          // body this substep. It is near the body next substep, so its contact plane takes over.
          m.x[i] = landOnBody(*m.body, static_cast<float>(radius), xStart[i].cast<float>(), m.x[i]);
          continue;
        }
        collideBody(*m.body, static_cast<float>(radius), static_cast<float>(m.fabrics[m.pFabric[i]].friction),
                    xStart[i].cast<float>(), m.x[i]);
      }
    }
    for (const auto& [i, pos] : m.pins) m.x[i] = pos;
    for (std::size_t i = 0; i < n; ++i) {
      m.v[i] = ((m.x[i].cast<double>() - xStart[i]) / h).cast<float>();
      double c = m.settings.dampingScale * m.fabrics[m.pFabric[i]].damping;
      if (m.garments[m.pGarment[i]].phase == SimPhase::Assembling) c += m.settings.assemblyExtraDamping;
      m.v[i] *= static_cast<float>(std::max(0.0, 1.0 - c * h));
    }
    for (auto& g : m.garments) {
      if (g.phase == SimPhase::Assembling) g.assemblyTime += h;
    }
  }
};

CpuSolver::CpuSolver() : impl_(std::make_unique<Impl>()) {}
CpuSolver::~CpuSolver() = default;

void CpuSolver::build(const SimScene& scene) {
  Impl next;
  next.m = SimModel::build(scene);
  next.allocate();
  *impl_ = std::move(next);
}

void CpuSolver::updateFabric(const GarmentId& id, const std::vector<FabricPhysical>& pieceFabric) {
  impl_->m.updateFabric(id, pieceFabric);
}

void CpuSolver::setPhase(const GarmentId& id, SimPhase phase) { impl_->m.setPhase(id, phase); }

void CpuSolver::step(const StepInput& in) {
  const int substeps = std::max(1, in.substeps);
  const double h = in.dt / substeps;
  const int iterations = std::max(1, in.iterations);
  for (int s = 0; s < substeps; ++s) impl_->substep(h, iterations);
}

void CpuSolver::readPositions(std::span<Vec3f> out) const {
  if (out.size() != impl_->m.x.size()) throw std::invalid_argument("readPositions: span size must equal particleCount()");
  std::copy(impl_->m.x.begin(), impl_->m.x.end(), out.begin());
}

SimStats CpuSolver::stats() const { return impl_->m.stats(); }

Snapshot CpuSolver::snapshot() const { return impl_->m.snapshot(); }

void CpuSolver::restore(const Snapshot& snap) { impl_->m.restore(snap); }

ParticleRange CpuSolver::range(const GarmentId& id) const { return impl_->m.range(id); }

std::size_t CpuSolver::particleCount() const { return impl_->m.x.size(); }

}  // namespace drape::sim
