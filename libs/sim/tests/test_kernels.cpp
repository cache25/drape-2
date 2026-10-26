#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <random>
#include <set>
#include <vector>

#include <Eigen/Dense>

#include "gpu/emulated_device.hpp"
#include "gpu_layout.h"

using namespace drape::sim;
using dgpu::uint;

namespace {

template <class T>
T* view(ComputeDevice& d, BufferId b) {
  return static_cast<T*>(d.contents(b));
}

template <class T>
BufferId upload(ComputeDevice& d, const std::vector<T>& data) {
  const BufferId b = d.create(sizeof(T) * std::max<std::size_t>(data.size(), 1));
  if (!data.empty()) std::memcpy(d.contents(b), data.data(), sizeof(T) * data.size());
  return b;
}

dgpu::float4 f4(const Eigen::Vector3d& v) {
  return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()), 0};
}
Eigen::Vector3d d3(const dgpu::float4& v) { return {v.x, v.y, v.z}; }

// A random block-CSR SPD system on n particles: each particle couples to itself and a few random neighbours.
struct Rig {
  static constexpr uint n = 40;
  EmulatedDevice dev;
  Eigen::MatrixXd A;  // dense 3n x 3n, pinned rows and columns zeroed
  std::vector<uint> rowStart, colIndex, diagSlot;
  std::vector<dgpu::Block3> blocks;
  std::vector<dgpu::GpuParticle> particles;
  BufferId params, pinfo, rows, cols, diag, blockBuf, contact, scalars, partials;
  BufferId grad, step, r, z, p, ap, apFull, resid, precond, prodA, prodB;
  BufferId specRz, specR0, specPAp, specRr, specRzNew;

  explicit Rig(std::set<uint> pinned = {3, 17, 29}) {
    std::mt19937 rng(77);
    std::uniform_real_distribution<double> unit(-1, 1);
    std::uniform_int_distribution<uint> pick(0, n - 1);
    std::vector<std::set<uint>> nb(n);
    for (uint i = 0; i < n; ++i) {
      nb[i].insert(i);
      for (int k = 0; k < 3; ++k) {
        const uint j = pick(rng);
        nb[i].insert(j);
        nb[j].insert(i);
      }
    }
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(3 * n, 3 * n);
    for (uint i = 0; i < n; ++i)
      for (uint j : nb[i])
        if (j > i) {
          Eigen::Matrix3d B;
          for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) B(a, b) = 0.3 * unit(rng);
          M.block<3, 3>(3 * i, 3 * j) = B;
          M.block<3, 3>(3 * j, 3 * i) = B.transpose();
        }
    for (uint i = 0; i < n; ++i) M.block<3, 3>(3 * i, 3 * i) = (6.0 + unit(rng)) * Eigen::Matrix3d::Identity();
    rowStart.push_back(0);
    for (uint i = 0; i < n; ++i) {
      for (uint j : nb[i]) {
        if (j == i) diagSlot.push_back(static_cast<uint>(colIndex.size()));
        colIndex.push_back(j);
        dgpu::Block3 b{};
        b.c0 = f4(M.block<3, 1>(3 * i, 3 * j + 0));
        b.c1 = f4(M.block<3, 1>(3 * i, 3 * j + 1));
        b.c2 = f4(M.block<3, 1>(3 * i, 3 * j + 2));
        blocks.push_back(b);
      }
      rowStart.push_back(static_cast<uint>(colIndex.size()));
    }
    particles.resize(n);
    for (uint i = 0; i < n; ++i) particles[i] = {1.0f, 0, 0, pinned.count(i) ? 1u : 0u};
    A = M;
    for (uint i : pinned) {
      A.block(3 * i, 0, 3, 3 * n).setZero();
      A.block(0, 3 * i, 3 * n, 3).setZero();
    }
    dgpu::StepParams sp{};
    sp.n = n;
    sp.nBlocks = static_cast<uint>(blocks.size());
    sp.cgTolerance = 1e-2f;
    sp.maxCgIterations = 50;
    params = upload(dev, std::vector<dgpu::StepParams>{sp});
    pinfo = upload(dev, particles);
    rows = upload(dev, rowStart);
    cols = upload(dev, colIndex);
    diag = upload(dev, diagSlot);
    blockBuf = upload(dev, blocks);
    contact = dev.create(sizeof(dgpu::GpuContact) * n);
    scalars = dev.create(sizeof(dgpu::CgScalars));
    partials = dev.create(sizeof(float) * (n / 256 + 1));
    for (BufferId* b : {&grad, &step, &r, &z, &p, &ap, &apFull, &resid}) *b = dev.create(sizeof(dgpu::float4) * n);
    precond = dev.create(sizeof(dgpu::Block3) * n);
    prodA = dev.create(sizeof(float) * n);
    prodB = dev.create(sizeof(float) * n);
    auto spec = [&](uint selector) { return upload(dev, std::vector<dgpu::ReduceSpec>{{n, selector, {0, 0}}}); };
    specRz = spec(dgpu::kReduceRz);
    specR0 = spec(dgpu::kReduceR0);
    specPAp = spec(dgpu::kReducePAp);
    specRr = spec(dgpu::kReduceRr);
    specRzNew = spec(dgpu::kReduceRzNew);
  }

  void reduce(BufferId spec, BufferId input) {
    const BufferId a[] = {spec, input, partials};
    dev.dispatch(Kernel::ReducePartial, n / 256 + 1, a);
    const BufferId b[] = {spec, partials, scalars};
    dev.dispatch(Kernel::ReduceFinal, 1, b);
  }

  void init() {
    const BufferId pre[] = {params, pinfo, diag, blockBuf, precond};
    dev.dispatch(Kernel::Precondition, n, pre);
    const BufferId ini[] = {params, contact, precond, grad, step, resid, r, z, p, prodA, prodB};
    dev.dispatch(Kernel::CgInit, n, ini);
    reduce(specRz, prodA);
    reduce(specR0, prodB);
  }

  void iteration() {
    const BufferId mv[] = {params, pinfo, rows, cols, blockBuf, contact, scalars, p, apFull, ap, prodA};
    dev.dispatch(Kernel::Spmv, n, mv);
    reduce(specPAp, prodA);
    const BufferId al[] = {scalars};
    dev.dispatch(Kernel::CgAlpha, 1, al);
    const BufferId up[] = {params, scalars, p, ap, apFull, step, r, resid, prodB};
    dev.dispatch(Kernel::CgUpdate, n, up);
    reduce(specRr, prodB);
    const BufferId ch[] = {params, scalars};
    dev.dispatch(Kernel::CgCheck, 1, ch);
    const BufferId pc[] = {params, contact, scalars, precond, r, z, prodA};
    dev.dispatch(Kernel::CgPrecond, n, pc);
    reduce(specRzNew, prodA);
    dev.dispatch(Kernel::CgBeta, 1, al);
    const BufferId dir[] = {params, scalars, z, p};
    dev.dispatch(Kernel::CgDirection, n, dir);
  }
};

}  // namespace

TEST(Kernels, LayoutSizes) {
  EXPECT_EQ(sizeof(dgpu::GpuParticle), 16u);
  EXPECT_EQ(sizeof(dgpu::GpuContact), 32u);
  EXPECT_EQ(sizeof(dgpu::ReduceSpec), 16u);
}

TEST(Kernels, ReduceIsDeterministicAndAccurate) {
  const uint count = 100003;
  std::mt19937 rng(1);
  std::uniform_real_distribution<float> val(-1.0f, 2.0f);
  std::vector<float> data(count);
  double exact = 0;
  for (auto& v : data) {
    v = val(rng);
    exact += v;
  }
  float results[2];
  for (float& res : results) {
    EmulatedDevice dev;
    const BufferId input = upload(dev, data);
    const BufferId spec = upload(dev, std::vector<dgpu::ReduceSpec>{{count, dgpu::kReduceRr, {0, 0}}});
    const BufferId partials = dev.create(sizeof(float) * (count / 256 + 1));
    const BufferId scalars = dev.create(sizeof(dgpu::CgScalars));
    dev.begin();
    const BufferId a[] = {spec, input, partials};
    dev.dispatch(Kernel::ReducePartial, count / 256 + 1, a);
    const BufferId b[] = {spec, partials, scalars};
    dev.dispatch(Kernel::ReduceFinal, 1, b);
    dev.submitAndWait();
    res = view<dgpu::CgScalars>(dev, scalars)->rr;
  }
  EXPECT_EQ(results[0], results[1]);
  EXPECT_NEAR(results[0], exact, 1e-5 * std::abs(exact));
}

TEST(Kernels, SpmvMatchesDense) {
  Rig rig;
  std::mt19937 rng(2);
  std::uniform_real_distribution<double> unit(-1, 1);
  Eigen::VectorXd pv(3 * Rig::n);
  for (uint i = 0; i < Rig::n; ++i) {
    const Eigen::Vector3d v(unit(rng), unit(rng), unit(rng));
    pv.segment<3>(3 * i) = v;
    view<dgpu::float4>(rig.dev, rig.p)[i] = f4(v);
  }
  rig.dev.begin();
  const BufferId mv[] = {rig.params, rig.pinfo, rig.rows, rig.cols, rig.blockBuf, rig.contact, rig.scalars,
                         rig.p, rig.apFull, rig.ap, rig.prodA};
  rig.dev.dispatch(Kernel::Spmv, Rig::n, mv);
  rig.dev.submitAndWait();
  const Eigen::VectorXd expected = rig.A * pv;
  Eigen::VectorXd got(3 * Rig::n);
  for (uint i = 0; i < Rig::n; ++i) got.segment<3>(3 * i) = d3(view<dgpu::float4>(rig.dev, rig.apFull)[i]);
  EXPECT_LE((got - expected).norm(), 1e-5 * expected.norm());
}

TEST(Kernels, CgSolvesSpdSystem) {
  Rig rig;
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> unit(-1, 1);
  Eigen::VectorXd b(3 * Rig::n);
  for (uint i = 0; i < Rig::n; ++i) {
    const bool pinned = rig.particles[i].pinned != 0;
    const Eigen::Vector3d v = pinned ? Eigen::Vector3d::Zero() : Eigen::Vector3d(unit(rng), unit(rng), unit(rng));
    b.segment<3>(3 * i) = v;
    view<dgpu::float4>(rig.dev, rig.grad)[i] = f4(-v);  // A step = -grad = b
  }
  rig.dev.begin();
  rig.init();
  for (int it = 0; it < 50; ++it) rig.iteration();
  rig.dev.submitAndWait();
  const auto* sc = view<dgpu::CgScalars>(rig.dev, rig.scalars);
  EXPECT_EQ(sc->converged, 1u);
  EXPECT_LE(sc->iteration, 50u);
  EXPECT_GT(sc->iteration, 0u);
  Eigen::VectorXd x(3 * Rig::n);
  for (uint i = 0; i < Rig::n; ++i) x.segment<3>(3 * i) = d3(view<dgpu::float4>(rig.dev, rig.step)[i]);
  EXPECT_LE((rig.A * x - b).norm(), 1e-2 * b.norm() * 1.0001);
}

TEST(Kernels, ContactFilterRemovesNormal) {
  Rig rig;
  const Eigen::Vector3d n = Eigen::Vector3d(0.3, 0.9, -0.2).normalized();
  auto* c = view<dgpu::GpuContact>(rig.dev, rig.contact);
  c[0].normal = f4(n);
  c[0].nearBody = 1;
  c[0].active = 1;
  for (uint i = 0; i < Rig::n; ++i) view<dgpu::float4>(rig.dev, rig.grad)[i] = {0.5f, -1.0f, 0.25f, 0};
  rig.dev.begin();
  rig.init();
  rig.iteration();
  rig.dev.submitAndWait();
  EXPECT_NEAR(d3(view<dgpu::float4>(rig.dev, rig.step)[0]).dot(n), 0.0, 1e-7);
  EXPECT_GT(d3(view<dgpu::float4>(rig.dev, rig.step)[0]).norm(), 1e-3);
}
