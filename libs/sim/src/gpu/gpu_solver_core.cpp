#include "gpu/gpu_solver_core.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "gpu_layout.h"
#include "sim_model.hpp"

namespace drape::sim {

using dgpu::uint;

namespace {

constexpr int kCgChunk = 10;  // CG iterations encoded per submission

dgpu::float4 f4(const Vec3f& v) { return {v.x(), v.y(), v.z(), 0.0f}; }
dgpu::float4 f4(const Vec3d& v) {
  return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()), 0.0f};
}
Vec3f v3(const dgpu::float4& v) { return {v.x, v.y, v.z}; }

dgpu::GpuFabric gpuFabric(const FabricPhysical& f) {
  dgpu::GpuFabric g{};
  g.stretchWarp = static_cast<float>(f.stretchWarp);
  g.stretchWeft = static_cast<float>(f.stretchWeft);
  g.stretchBias = static_cast<float>(f.stretchBias);
  g.strainLimitWarp = static_cast<float>(f.strainLimitWarp);
  g.strainLimitWeft = static_cast<float>(f.strainLimitWeft);
  g.weight = static_cast<float>(f.weight);
  g.thickness = static_cast<float>(f.thickness);
  g.friction = static_cast<float>(f.friction);
  g.damping = static_cast<float>(f.damping);
  return g;
}

}  // namespace

struct GpuSolverCore::Impl {
  std::unique_ptr<ComputeDevice> dev;
  SimModel m;
  uint n = 0, nStretch = 0, nHinge = 0, nSeam = 0, nBlocks = 0, nEnergies = 0;

  // Static scene data.
  BufferId particles, garments, fabrics, fExt, pinPos, stretch, hinges, seams, rowStart, colIndex, diagSlot;
  BufferId gradStart, gradIdx, blockStart, blockIdx, fieldValues, fieldInfo;
  // Per-step state and work.
  BufferId paramsDeriv, paramsEnergy, x, v, xStart, yRel, u, trialU, grad, step, r, z, p, ap, apFull, resid;
  BufferId precond, blocks, contact, energies, gradContrib, hessContrib, scalars, partials, prodA, prodB, moved;
  BufferId specRz, specR0, specPAp, specRr, specRzNew;

  explicit Impl(std::unique_ptr<ComputeDevice> d) : dev(std::move(d)) {}

  template <class T>
  T* view(BufferId b) const {
    return static_cast<T*>(dev->contents(b));
  }

  template <class T>
  BufferId upload(const std::vector<T>& data) {
    const BufferId b = dev->create(sizeof(T) * std::max<std::size_t>(data.size(), 1));
    if (!data.empty()) std::memcpy(dev->contents(b), data.data(), sizeof(T) * data.size());
    return b;
  }

  template <class T>
  void write(BufferId b, const std::vector<T>& data) {
    if (!data.empty()) std::memcpy(dev->contents(b), data.data(), sizeof(T) * data.size());
  }

  template <class T>
  BufferId array(std::size_t count) {
    return dev->create(sizeof(T) * std::max<std::size_t>(count, 1));
  }

  void dispatch(Kernel k, uint threads, std::initializer_list<BufferId> buffers) {
    dev->dispatch(k, threads, std::span<const BufferId>(buffers.begin(), buffers.size()));
  }

  // ---- Model <-> device ----

  std::vector<dgpu::GpuParticle> particleData() const {
    std::vector<dgpu::GpuParticle> out(n);
    for (uint i = 0; i < n; ++i)
      out[i] = {static_cast<float>(m.mass[i]), m.pFabric[i], m.pGarment[i], m.pinned[i] ? 1u : 0u};
    return out;
  }
  std::vector<dgpu::GpuFabric> fabricData() const {
    std::vector<dgpu::GpuFabric> out;
    for (const auto& f : m.fabrics) out.push_back(gpuFabric(f));
    return out;
  }
  std::vector<dgpu::GpuGarment> garmentData() const {
    std::vector<dgpu::GpuGarment> out;
    for (const auto& g : m.garments)
      out.push_back({g.phase == SimPhase::Settling ? 1u : 0u, static_cast<float>(g.assemblyTime), {0, 0}});
    return out;
  }
  std::vector<dgpu::GpuHinge> hingeData() const {
    std::vector<dgpu::GpuHinge> out;
    for (const auto& h : m.hinges)
      out.push_back({{h.v[0], h.v[1], h.v[2], h.v[3]}, static_cast<float>(h.K), static_cast<float>(h.restAngle), {0, 0}});
    return out;
  }
  std::vector<dgpu::GpuSeam> seamData() const {
    std::vector<dgpu::GpuSeam> out;
    for (const auto& s : m.seams) out.push_back({s.a, s.b, s.garment, s.startGap});
    return out;
  }

  // Device -> model: positions, velocities and contact forces (the state the device advances).
  void syncToModel() {
    const auto* xs = view<dgpu::float4>(x);
    const auto* vs = view<dgpu::float4>(v);
    const auto* cs = view<dgpu::GpuContact>(contact);
    for (uint i = 0; i < n; ++i) {
      m.x[i] = v3(xs[i]);
      m.v[i] = v3(vs[i]);
      m.contactForce[i] = cs[i].force;
    }
  }

  // Model -> device: everything the model may have changed (after build, restore, setPhase, updateFabric).
  void syncFromModel() {
    auto* xs = view<dgpu::float4>(x);
    auto* vs = view<dgpu::float4>(v);
    auto* cs = view<dgpu::GpuContact>(contact);
    for (uint i = 0; i < n; ++i) {
      xs[i] = f4(m.x[i]);
      vs[i] = f4(m.v[i]);
      cs[i].force = static_cast<float>(m.contactForce[i]);
    }
    write(particles, particleData());
    write(fabrics, fabricData());
    write(garments, garmentData());
    write(hinges, hingeData());
    write(seams, seamData());
  }

  void build(const SimScene& scene) {
    m = SimModel::build(scene);
    n = static_cast<uint>(m.x.size());
    nStretch = static_cast<uint>(m.stretch.size());
    nHinge = static_cast<uint>(m.hinges.size());
    nSeam = static_cast<uint>(m.seams.size());
    nBlocks = static_cast<uint>(m.colIndex.size());
    nEnergies = n + nStretch + nHinge + nSeam;
    const uint nGrad = n + 3 * nStretch + 4 * nHinge + 2 * nSeam;
    const uint nHess = n + 9 * nStretch + 16 * nHinge + 4 * nSeam;

    particles = upload(particleData());
    fabrics = upload(fabricData());
    garments = upload(garmentData());
    hinges = upload(hingeData());
    seams = upload(seamData());
    {
      std::vector<dgpu::float4> fe(n), pins(n);
      for (uint i = 0; i < n; ++i) fe[i] = f4(m.fExt[i]);
      for (const auto& [i, pos] : m.pins) pins[i] = f4(pos);
      fExt = upload(fe);
      pinPos = upload(pins);
    }
    {
      std::vector<dgpu::GpuStretch> st;
      for (const auto& t : m.stretch) {
        dgpu::GpuStretch g{};
        g.v[0] = t.v[0];
        g.v[1] = t.v[1];
        g.v[2] = t.v[2];
        g.dmInv = {static_cast<float>(t.dmInv(0, 0)), static_cast<float>(t.dmInv(1, 0)),
                   static_cast<float>(t.dmInv(0, 1)), static_cast<float>(t.dmInv(1, 1))};
        g.area = static_cast<float>(t.area);
        g.fabric = t.fabric;
        st.push_back(g);
      }
      stretch = upload(st);
    }
    rowStart = upload(m.rowStart);
    colIndex = upload(m.colIndex);
    diagSlot = upload(m.diagSlot);

    // Gather lists in a fixed order: the particle's own term, then stretch, hinge and seam terms in term order.
    {
      std::vector<std::vector<uint>> g(n), b(nBlocks);
      for (uint i = 0; i < n; ++i) {
        g[i].push_back(i);
        b[m.diagSlot[i]].push_back(i);
      }
      for (uint t = 0; t < nStretch; ++t)
        for (uint a = 0; a < 3; ++a) {
          g[m.stretch[t].v[a]].push_back(n + 3 * t + a);
          for (uint c = 0; c < 3; ++c) b[m.stretchSlots[t][3 * a + c]].push_back(n + 9 * t + 3 * a + c);
        }
      for (uint h = 0; h < nHinge; ++h)
        for (uint a = 0; a < 4; ++a) {
          g[m.hinges[h].v[a]].push_back(n + 3 * nStretch + 4 * h + a);
          for (uint c = 0; c < 4; ++c)
            b[m.hingeSlots[h][4 * a + c]].push_back(n + 9 * nStretch + 16 * h + 4 * a + c);
        }
      for (uint k = 0; k < nSeam; ++k) {
        const uint vs[2] = {m.seams[k].a, m.seams[k].b};
        for (uint a = 0; a < 2; ++a) {
          g[vs[a]].push_back(n + 3 * nStretch + 4 * nHinge + 2 * k + a);
          for (uint c = 0; c < 2; ++c)
            b[m.seamSlots[k][2 * a + c]].push_back(n + 9 * nStretch + 16 * nHinge + 4 * k + 2 * a + c);
        }
      }
      auto flatten = [](const std::vector<std::vector<uint>>& lists, std::vector<uint>& start, std::vector<uint>& idx) {
        start.assign(1, 0);
        for (const auto& l : lists) {
          idx.insert(idx.end(), l.begin(), l.end());
          start.push_back(static_cast<uint>(idx.size()));
        }
      };
      std::vector<uint> gs, gi, bs, bi;
      flatten(g, gs, gi);
      flatten(b, bs, bi);
      gradStart = upload(gs);
      gradIdx = upload(gi);
      blockStart = upload(bs);
      blockIdx = upload(bi);
    }
    {
      dgpu::FieldInfo info{};
      if (m.body && !m.body->values.empty()) {
        info.origin = f4(m.body->origin);
        for (int a = 0; a < 3; ++a) info.dims[a] = static_cast<uint>(m.body->dims[a]);
        info.cellSize = static_cast<float>(m.body->cellSize);
        info.band = static_cast<float>(m.body->band);
        fieldValues = upload(m.body->values);
      } else {
        fieldValues = array<float>(1);
      }
      fieldInfo = upload(std::vector<dgpu::FieldInfo>{info});
    }

    dgpu::StepParams sp{};
    sp.gravity = f4(m.settings.gravity);
    sp.limitScale = static_cast<float>(m.settings.strainLimitStiffnessScale);
    sp.seamStiffness = static_cast<float>(m.settings.seamStiffness);
    sp.closingSpeed = static_cast<float>(m.settings.assemblyClosingSpeed);
    sp.frictionSlip = static_cast<float>(m.settings.frictionSlipSpeed);
    sp.cgTolerance = static_cast<float>(m.settings.cgTolerance);
    sp.collisionOffset = static_cast<float>(m.settings.collisionOffset);
    sp.dampingScale = static_cast<float>(m.settings.dampingScale);
    sp.assemblyExtraDamping = static_cast<float>(m.settings.assemblyExtraDamping);
    sp.contactReach = static_cast<float>(kContactReach);
    sp.contactSnap = static_cast<float>(kContactSnap);
    sp.n = n;
    sp.nStretch = nStretch;
    sp.nHinge = nHinge;
    sp.nSeam = nSeam;
    sp.nBlocks = nBlocks;
    sp.maxCgIterations = static_cast<uint>(m.settings.maxCgIterations);
    sp.withDerivatives = 1;
    paramsDeriv = upload(std::vector<dgpu::StepParams>{sp});
    sp.withDerivatives = 0;
    paramsEnergy = upload(std::vector<dgpu::StepParams>{sp});

    x = array<dgpu::float4>(n);
    v = array<dgpu::float4>(n);
    for (BufferId* b : {&xStart, &yRel, &u, &trialU, &grad, &step, &r, &z, &p, &ap, &apFull, &resid})
      *b = array<dgpu::float4>(n);
    precond = array<dgpu::Block3>(n);
    blocks = array<dgpu::Block3>(nBlocks);
    contact = array<dgpu::GpuContact>(n);
    energies = array<float>(nEnergies);
    gradContrib = array<dgpu::float4>(nGrad);
    hessContrib = array<dgpu::Block3>(nHess);
    scalars = array<dgpu::CgScalars>(1);
    partials = array<float>(n / dgpu::kReduceBlock + 1);
    prodA = array<float>(n);
    prodB = array<float>(n);
    moved = array<float>(n);
    auto spec = [&](uint selector) { return upload(std::vector<dgpu::ReduceSpec>{{n, selector, {0, 0}}}); };
    specRz = spec(dgpu::kReduceRz);
    specR0 = spec(dgpu::kReduceR0);
    specPAp = spec(dgpu::kReducePAp);
    specRr = spec(dgpu::kReduceRr);
    specRzNew = spec(dgpu::kReduceRzNew);
    syncFromModel();
  }

  // ---- One substep ----

  void reduce(BufferId spec, BufferId input) {
    dispatch(Kernel::ReducePartial, n / dgpu::kReduceBlock + 1, {spec, input, partials});
    dispatch(Kernel::ReduceFinal, 1, {spec, partials, scalars});
  }

  // Term kernels at `positions` (u, or trialU in the line search); the params choose whether derivatives are written.
  void evalTerms(BufferId params, BufferId positions) {
    dispatch(Kernel::InertiaFriction, n,
             {params, particles, fabrics, positions, yRel, contact, energies, gradContrib, hessContrib});
    dispatch(Kernel::StretchTerms, nStretch,
             {params, stretch, fabrics, xStart, positions, energies, gradContrib, hessContrib});
    dispatch(Kernel::HingeTerms, nHinge, {params, hinges, xStart, positions, energies, gradContrib, hessContrib});
    dispatch(Kernel::SeamTerms, nSeam,
             {params, seams, garments, xStart, positions, energies, gradContrib, hessContrib});
  }

  // Total energy, summed in double in a fixed order (Phase B spec 5.2 rule 2).
  double energySum() const {
    const float* e = view<float>(energies);
    double s = 0;
    for (uint k = 0; k < nEnergies; ++k) s += e[k];
    return s;
  }

  void cgIteration() {
    dispatch(Kernel::Spmv, n, {paramsDeriv, particles, rowStart, colIndex, blocks, contact, scalars, p, apFull, ap, prodA});
    reduce(specPAp, prodA);
    dispatch(Kernel::CgAlpha, 1, {scalars});
    dispatch(Kernel::CgUpdate, n, {paramsDeriv, scalars, p, ap, apFull, step, r, resid, prodB});
    reduce(specRr, prodB);
    dispatch(Kernel::CgCheck, 1, {paramsDeriv, scalars});
    dispatch(Kernel::CgPrecond, n, {paramsDeriv, contact, scalars, precond, r, z, prodA});
    reduce(specRzNew, prodA);
    dispatch(Kernel::CgBeta, 1, {scalars});
    dispatch(Kernel::CgDirection, n, {paramsDeriv, scalars, z, p});
  }

  void substep(double h, int iterations) {
    for (BufferId b : {paramsDeriv, paramsEnergy}) {
      auto* sp = view<dgpu::StepParams>(b);
      sp->h = static_cast<float>(h);
      sp->invH2 = static_cast<float>(1.0 / (h * h));
    }
    auto* lineParams = view<dgpu::StepParams>(paramsEnergy);

    dev->begin();
    dispatch(Kernel::Predict, n, {paramsDeriv, particles, garments, x, v, fExt, xStart, yRel, u});
    dispatch(Kernel::ContactSetup, n, {paramsDeriv, particles, fabrics, xStart, fieldValues, fieldInfo, contact});
    for (int it = 0; it < iterations; ++it) {
      if (it > 0) dev->begin();
      dispatch(Kernel::ContactActivate, n, {paramsDeriv, contact, u});
      evalTerms(paramsDeriv, u);
      dispatch(Kernel::GatherGrad, n, {paramsDeriv, particles, gradStart, gradIdx, gradContrib, grad});
      dispatch(Kernel::GatherBlocks, nBlocks, {paramsDeriv, blockStart, blockIdx, hessContrib, blocks});
      dispatch(Kernel::ContactRelease, n, {paramsDeriv, contact, grad});
      dev->submitAndWait();
      const double e0 = energySum();

      // Linear solve, encoded kCgChunk iterations per submission until converged or maxCgIterations.
      dev->begin();
      dispatch(Kernel::Precondition, n, {paramsDeriv, particles, diagSlot, blocks, precond});
      dispatch(Kernel::CgInit, n, {paramsDeriv, contact, precond, grad, step, resid, r, z, p, prodA, prodB});
      reduce(specRz, prodA);
      reduce(specR0, prodB);
      int encoded = 0;
      const int maxCg = m.settings.maxCgIterations;
      while (true) {
        const int chunk = std::min(kCgChunk, maxCg - encoded);
        for (int c = 0; c < chunk; ++c) cgIteration();
        encoded += chunk;
        if (encoded >= maxCg) dispatch(Kernel::ContactForce, n, {paramsDeriv, contact, resid});
        dev->submitAndWait();
        if (encoded >= maxCg) break;
        if (view<dgpu::CgScalars>(scalars)->converged != 0u) {
          dev->begin();
          dispatch(Kernel::ContactForce, n, {paramsDeriv, contact, resid});
          dev->submitAndWait();
          break;
        }
        dev->begin();
      }

      // Newton slope grad . step, in double.
      double slope = 0;
      {
        const auto* g = view<dgpu::float4>(grad);
        const auto* s = view<dgpu::float4>(step);
        for (uint i = 0; i < n; ++i)
          slope += static_cast<double>(g[i].x) * s[i].x + static_cast<double>(g[i].y) * s[i].y +
                   static_cast<double>(g[i].z) * s[i].z;
      }
      if (!(slope < 0)) break;

      double alpha = 1.0;
      for (int ls = 0; ls < 20; ++ls) {
        lineParams->alpha = static_cast<float>(alpha);
        dev->begin();
        dispatch(Kernel::TrialPositions, n, {paramsEnergy, u, step, trialU});
        evalTerms(paramsEnergy, trialU);
        dev->submitAndWait();
        if (energySum() <= e0 + 1e-4 * alpha * slope) break;
        alpha *= 0.5;
      }
      lineParams->alpha = static_cast<float>(alpha);
      dev->begin();
      dispatch(Kernel::ApplyStep, n, {paramsEnergy, contact, u, step, moved});
      dev->submitAndWait();
      const float* mv = view<float>(moved);
      double maxMove = 0;
      for (uint i = 0; i < n; ++i) maxMove = std::max(maxMove, static_cast<double>(mv[i]));
      if (maxMove < 1e-9) break;
    }
    dev->begin();
    dispatch(Kernel::FinishSubstep, n,
             {paramsDeriv, particles, fabrics, garments, xStart, u, pinPos, fieldValues, fieldInfo, x, v});
    dev->submitAndWait();
    for (auto& g : m.garments)
      if (g.phase == SimPhase::Assembling) g.assemblyTime += h;
    write(garments, garmentData());
  }
};

GpuSolverCore::GpuSolverCore(std::unique_ptr<ComputeDevice> device) : impl_(std::make_unique<Impl>(std::move(device))) {
  if (!impl_->dev) throw std::invalid_argument("GpuSolverCore: no compute device");
}

GpuSolverCore::~GpuSolverCore() = default;

void GpuSolverCore::build(const SimScene& scene) { impl_->build(scene); }

void GpuSolverCore::updateFabric(const GarmentId& g, const std::vector<FabricPhysical>& pieceFabric) {
  impl_->syncToModel();
  impl_->m.updateFabric(g, pieceFabric);
  impl_->syncFromModel();
}

void GpuSolverCore::setPhase(const GarmentId& g, SimPhase phase) {
  impl_->syncToModel();
  impl_->m.setPhase(g, phase);
  impl_->syncFromModel();
}

void GpuSolverCore::step(const StepInput& in) {
  if (impl_->n == 0) return;
  const int substeps = std::max(1, in.substeps);
  const double h = in.dt / substeps;
  const int iterations = std::max(1, in.iterations);
  for (int s = 0; s < substeps; ++s) impl_->substep(h, iterations);
}

void GpuSolverCore::readPositions(std::span<Vec3f> out) const {
  if (out.size() != impl_->n) throw std::invalid_argument("readPositions: span size must equal particleCount()");
  const auto* xs = impl_->view<dgpu::float4>(impl_->x);
  for (uint i = 0; i < impl_->n; ++i) out[i] = v3(xs[i]);
}

SimStats GpuSolverCore::stats() const {
  impl_->syncToModel();
  return impl_->m.stats();
}

Snapshot GpuSolverCore::snapshot() const {
  impl_->syncToModel();
  return impl_->m.snapshot();
}

void GpuSolverCore::restore(const Snapshot& s) {
  impl_->syncToModel();
  impl_->m.restore(s);
  impl_->syncFromModel();
}

ParticleRange GpuSolverCore::range(const GarmentId& g) const { return impl_->m.range(g); }

std::size_t GpuSolverCore::particleCount() const { return impl_->n; }

}  // namespace drape::sim
