#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "collision.hpp"
#include "drape/avatar/collision_bake.hpp"
#include "drape/avatar/primitives.hpp"
#include "drape/core/test_fabrics.hpp"
#include "energies.hpp"
#include "gpu_layout.h"
#include "term_math.h"

using namespace drape;
using namespace drape::sim;

// Float ports (kernels/term_math.h) against the double reference (energies.cpp, collision.cpp, collision_field.cpp).
// Bounds: relative to the double value, plus a floor of 1e-6 of the term's natural scale (its value at unit
// strain or unit angle), below which 32-bit cancellation in C = |f| - 1 or theta - theta0 dominates.

namespace {

dgpu::float3 f3(const V3& v) { return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())}; }
V3 d3(const dgpu::float3& v) { return {v.x, v.y, v.z}; }
M3 dm(const dgpu::Mat3& m) {
  M3 r;
  r.col(0) = d3(m.c0);
  r.col(1) = d3(m.c1);
  r.col(2) = d3(m.c2);
  return r;
}

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

// Compares a float evaluation against the double one; sE, sG, sH are the natural scales of energy, gradient and
// Hessian blocks.
template <int N>
void expectClose(const dgpu::TermEvalF<N>& f, const TermEval<static_cast<std::size_t>(N)>& d, double sE, double sG,
                 double sH, int sample) {
  EXPECT_LE(std::abs(f.energy - d.energy), 1e-4 * std::abs(d.energy) + 1e-6 * sE + 1e-12) << "sample " << sample;
  double gErr = 0, gNorm = 0, hErr = 0, hNorm = 0;
  for (int a = 0; a < N; ++a) {
    gErr += (d3(f.grad[a]) - d.grad[a]).squaredNorm();
    gNorm += d.grad[a].squaredNorm();
    for (int b = 0; b < N; ++b) {
      hErr += (dm(f.hess[a][b]) - d.hess[a][b]).squaredNorm();
      hNorm += d.hess[a][b].squaredNorm();
    }
  }
  EXPECT_LE(std::sqrt(gErr), 1e-3 * std::sqrt(gNorm) + 1e-6 * sG + 1e-9) << "sample " << sample;
  EXPECT_LE(std::sqrt(hErr), 1e-3 * std::sqrt(hNorm) + 1e-6 * sH + 1e-9) << "sample " << sample;
}

// A random rest triangle with edges of 2-20 mm, deformed by warp/weft strains of -20%..+30%, a shear, a rotation
// and a little out-of-plane noise. Positions are relative to vertex 0, as the kernels pass them.
struct StretchSample {
  StretchTerm term;
  std::array<V3, 3> x;
  double minEdge = 0;
};

StretchSample randomStretch(std::mt19937& rng) {
  std::uniform_real_distribution<double> len(0.002, 0.02), ang(0.6, 2.5), strain(-0.2, 0.3), shear(-0.2, 0.2),
      unit(-1, 1), noise(-0.1, 0.1);
  const double a = len(rng), b = len(rng), t = ang(rng);
  const Vec2d r0(0, 0), r1(a, 0), r2(b * std::cos(t), b * std::sin(t));
  Eigen::Matrix2d dmat;
  dmat << r1.x() - r0.x(), r2.x() - r0.x(), r1.y() - r0.y(), r2.y() - r0.y();
  StretchSample s;
  s.term.v = {0, 1, 2};
  s.term.dmInv = dmat.inverse();
  s.term.area = 0.5 * std::abs(dmat.determinant());
  s.minEdge = std::min({a, b, (r2 - r1).norm()});
  Eigen::Matrix2d F;
  F << 1 + strain(rng), shear(rng), shear(rng), 1 + strain(rng);
  const M3 R = Eigen::AngleAxisd(3 * unit(rng), V3(unit(rng), unit(rng), unit(rng)).normalized()).toRotationMatrix();
  const Vec2d rr[3] = {r0, r1, r2};
  for (int k = 0; k < 3; ++k) {
    const Vec2d p = F * rr[k];
    s.x[k] = R * V3(p.x(), p.y(), noise(rng) * s.minEdge);
  }
  const V3 origin = s.x[0];
  for (auto& p : s.x) p -= origin;
  return s;
}

const CollisionField& sphereField() {
  static const CollisionField f = [] {
    std::vector<TriangleMesh> c{avatar::icosphere({0, 0, 0}, 0.15, 4)};
    return avatar::bakeCollisionField(c);
  }();
  return f;
}

dgpu::FieldInfo fieldInfo(const CollisionField& f) {
  dgpu::FieldInfo info{};
  info.origin = {static_cast<float>(f.origin.x()), static_cast<float>(f.origin.y()), static_cast<float>(f.origin.z()), 0};
  info.cellSize = static_cast<float>(f.cellSize);
  info.band = static_cast<float>(f.band);
  for (int a = 0; a < 3; ++a) info.dims[a] = static_cast<std::uint32_t>(f.dims[a]);
  return info;
}

}  // namespace

TEST(TermMath, StretchMatchesDouble) {
  std::mt19937 rng(1234);
  const FabricPhysical fabrics[3] = {testdata::testJersey(), testdata::testStiff(), testdata::testAniso()};
  const double limitScale = 100.0;
  for (int i = 0; i < 200; ++i) {
    const StretchSample s = randomStretch(rng);
    const FabricPhysical& fab = fabrics[i % 3];
    TermEval<3> d;
    d.clear();
    evalStretchTerm(s.term, s.x, fab, limitScale, d);
    dgpu::GpuStretch g{};
    g.dmInv = {static_cast<float>(s.term.dmInv(0, 0)), static_cast<float>(s.term.dmInv(1, 0)),
               static_cast<float>(s.term.dmInv(0, 1)), static_cast<float>(s.term.dmInv(1, 1))};
    g.area = static_cast<float>(s.term.area);
    dgpu::float3 xs[3] = {f3(s.x[0]), f3(s.x[1]), f3(s.x[2])};
    dgpu::TermEvalF<3> f;
    dgpu::clearTerm(f);
    dgpu::evalStretchF(g, xs, gpuFabric(fab), static_cast<float>(limitScale), f);
    const double kMax = std::max({fab.stretchWarp, fab.stretchWeft, fab.stretchBias}) * s.term.area * (1 + limitScale);
    expectClose(f, d, kMax, kMax / s.minEdge, kMax / (s.minEdge * s.minEdge), i);
  }
}

TEST(TermMath, HingeMatchesDouble) {
  std::mt19937 rng(99);
  std::uniform_real_distribution<double> len(0.002, 0.02), fold(-2.5, 2.5), rest(-0.5, 0.5), unit(-1, 1);
  for (int i = 0; i < 200; ++i) {
    // Hinge edge x3 -> x4 along X; wings at random heights, folded by a random angle.
    const double e = len(rng), h1 = len(rng), h2 = len(rng), phi = fold(rng);
    std::array<V3, 4> x{V3(0.3 * e, h1, 0), V3(0.6 * e, -h2 * std::cos(phi), h2 * std::sin(phi)), V3(0, 0, 0),
                        V3(e, 0, 0)};
    const M3 R = Eigen::AngleAxisd(3 * unit(rng), V3(unit(rng), unit(rng), unit(rng)).normalized()).toRotationMatrix();
    for (auto& p : x) p = R * p;
    const V3 origin = x[0];
    for (auto& p : x) p -= origin;
    HingeTerm t;
    t.K = 1e-5 * (1 + 10 * (unit(rng) + 1));
    t.restAngle = rest(rng);
    TermEval<4> d;
    d.clear();
    evalHingeTerm(t, x, d);
    dgpu::GpuHinge g{};
    g.K = static_cast<float>(t.K);
    g.restAngle = static_cast<float>(t.restAngle);
    dgpu::float3 xs[4] = {f3(x[0]), f3(x[1]), f3(x[2]), f3(x[3])};
    dgpu::TermEvalF<4> f;
    dgpu::clearTerm(f);
    dgpu::evalHingeF(g, xs, f);
    const double l = std::min({e, h1, h2});
    expectClose(f, d, t.K, t.K / l, t.K / (l * l), i);
  }
}

TEST(TermMath, SeamMatchesDouble) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> gap(0.0, 0.03), unit(-1, 1);
  const double K = 1e4;
  for (int i = 0; i < 200; ++i) {
    const std::array<V3, 2> x{V3(0, 0, 0), gap(rng) * V3(unit(rng), unit(rng), unit(rng)).normalized()};
    const double target = i % 3 == 0 ? 0.0 : gap(rng);
    TermEval<2> d;
    d.clear();
    evalSeamTerm(x, target, K, d);
    dgpu::float3 xs[2] = {f3(x[0]), f3(x[1])};
    dgpu::TermEvalF<2> f;
    dgpu::clearTerm(f);
    dgpu::evalSeamF(xs, static_cast<float>(target), static_cast<float>(K), f);
    expectClose(f, d, K * 1e-4, K * 1e-2, K, i);
  }
}

TEST(TermMath, FrictionMatchesDouble) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> unit(-1, 1), slip(0, 3), lambda(0.001, 0.5);
  const double eps = 3e-4 / 60.0;
  for (int i = 0; i < 200; ++i) {
    const V3 n = V3(unit(rng), unit(rng), unit(rng)).normalized();
    const V3 dx = slip(rng) * eps * V3(unit(rng), unit(rng), unit(rng)).normalized();
    const double muLambda = lambda(rng);
    TermEval<1> d;
    d.clear();
    evalFrictionTerm(dx, V3::Zero(), n, muLambda, eps, d);
    dgpu::TermEvalF<1> f;
    dgpu::clearTerm(f);
    dgpu::evalFrictionF(f3(dx), f3(n), static_cast<float>(muLambda), static_cast<float>(eps), f);
    expectClose(f, d, muLambda * eps, muLambda, muLambda / eps, i);
  }
}

TEST(TermMath, FieldMatchesCollisionField) {
  const CollisionField& field = sphereField();
  const dgpu::FieldInfo info = fieldInfo(field);
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> unit(-1, 1), radius(0.12, 0.19);
  for (int i = 0; i < 1000; ++i) {
    const V3 p = radius(rng) * V3(unit(rng), unit(rng), unit(rng)).normalized();
    const double sd = field.sample(p);
    const float sf = dgpu::fieldSample(field.values.data(), info, f3(p));
    EXPECT_NEAR(sf, sd, 1e-6) << i;
    const V3 gd = field.gradient(p);
    const V3 gf = d3(dgpu::fieldGradient(field.values.data(), info, f3(p)));
    // Raw (unnormalized) slope: ~1 inside the band, ~0 where the field is clamped flat. Directions are compared where
    // the slope is meaningful; near the clamped edge (|g| < 1e-2) float noise (~1e-6) dominates the direction.
    V3 raw;
    for (int a = 0; a < 3; ++a) {
      V3 d = V3::Zero();
      d[a] = field.cellSize;
      raw[a] = (field.sample(p + d) - field.sample(p - d)) / (2 * field.cellSize);
    }
    if (gd.isZero()) {
      EXPECT_TRUE(gf.isZero()) << i;
    }
    if (raw.norm() >= 1e-2) {
      ASSERT_FALSE(gf.isZero()) << i;
      EXPECT_LE(std::acos(std::clamp(gd.dot(gf.normalized()), -1.0, 1.0)), 1e-3) << i;
    }
  }
  EXPECT_FLOAT_EQ(dgpu::fieldSample(field.values.data(), info, {5, 5, 5}), static_cast<float>(field.band));
}

TEST(TermMath, CollideMatchesDouble) {
  const CollisionField& field = sphereField();
  const dgpu::FieldInfo info = fieldInfo(field);
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> unit(-1, 1), depth(0.13, 0.153), step(0.0, 0.004);
  const float radius = 0.0033f, friction = 0.3f;
  for (int i = 0; i < 300; ++i) {
    const V3 dir = V3(unit(rng), unit(rng), unit(rng)).normalized();
    const Eigen::Vector3f xPrev = (0.16 * dir).cast<float>();
    Eigen::Vector3f xd = (depth(rng) * dir + step(rng) * V3(unit(rng), unit(rng), unit(rng))).cast<float>();
    dgpu::float3 xf = {xd.x(), xd.y(), xd.z()};
    collideBody(field, radius, friction, xPrev, xd);
    dgpu::collideBodyF(field.values.data(), info, radius, friction, {xPrev.x(), xPrev.y(), xPrev.z()}, xf);
    EXPECT_LE((d3(xf) - xd.cast<double>()).norm(), 1e-6) << i;

    const Eigen::Vector3f from = (0.2 * dir).cast<float>(), to = (0.05 * dir).cast<float>();
    const Eigen::Vector3f landed = landOnBody(field, radius, from, to);
    const dgpu::float3 lf = dgpu::landOnBodyF(field.values.data(), info, radius, {from.x(), from.y(), from.z()},
                                              {to.x(), to.y(), to.z()});
    EXPECT_LE((d3(lf) - landed.cast<double>()).norm(), 1e-6) << i;
  }
}

TEST(TermMath, InvertSpd3) {
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> unit(-1, 1);
  for (int i = 0; i < 200; ++i) {
    M3 B;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) B(r, c) = unit(rng);
    const M3 A = B * B.transpose() + 0.1 * M3::Identity();
    dgpu::Mat3 Af{f3(A.col(0)), f3(A.col(1)), f3(A.col(2))};
    bool ok = false;
    const M3 inv = dm(dgpu::invertSpd3(Af, ok));
    ASSERT_TRUE(ok) << i;
    const V3 x(unit(rng), unit(rng), unit(rng));
    EXPECT_LE((inv * (A * x) - x).norm(), 1e-4 * x.norm()) << i;
  }
  M3 neg = M3::Identity();
  neg(1, 1) = -1;
  bool ok = true;
  dgpu::invertSpd3(dgpu::Mat3{f3(neg.col(0)), f3(neg.col(1)), f3(neg.col(2))}, ok);
  EXPECT_FALSE(ok);
}

TEST(GpuLayout, Sizes) {
  EXPECT_EQ(sizeof(dgpu::GpuStretch), 48u);
  EXPECT_EQ(sizeof(dgpu::GpuHinge), 32u);
  EXPECT_EQ(sizeof(dgpu::GpuSeam), 16u);
  EXPECT_EQ(sizeof(dgpu::GpuFabric), 48u);
  EXPECT_EQ(sizeof(dgpu::GpuGarment), 16u);
  EXPECT_EQ(sizeof(dgpu::Block3), 48u);
  EXPECT_EQ(sizeof(dgpu::FieldInfo), 48u);
  EXPECT_EQ(sizeof(dgpu::StepParams), 96u);
  EXPECT_EQ(sizeof(dgpu::CgScalars), 32u);
}
