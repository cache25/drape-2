#pragma once

#include <array>
#include <cstdint>

#include <Eigen/Dense>

#include "drape/core/fabric.hpp"

namespace drape::sim {

using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;

// Energy, gradient and positive semi-definite Hessian of one term over its N vertices.
template <std::size_t N>
struct TermEval {
  double energy = 0;
  std::array<V3, N> grad;
  std::array<std::array<M3, N>, N> hess;
  void clear() {
    energy = 0;
    for (auto& g : grad) g.setZero();
    for (auto& row : hess)
      for (auto& b : row) b.setZero();
  }
};

struct StretchTerm {
  std::array<std::uint32_t, 3> v{};
  Eigen::Matrix2d dmInv = Eigen::Matrix2d::Identity();
  double area = 0;
  std::uint32_t fabric = 0;
};

// Warp/weft/bias energies 1/2 K C^2 (K = k * area) plus one-sided strain-limit energies (K = limitScale * k * area).
// Hessian: Gauss-Newton plus the tension (geometric) stiffness of warp/weft when stretched (C > 0).
void evalStretchTerm(const StretchTerm& t, const std::array<V3, 3>& x, const FabricPhysical& f, double limitScale,
                     TermEval<3>& out);

struct HingeTerm {
  std::array<std::uint32_t, 4> v{};  // opposite A, opposite B, edge start, edge end
  double K = 0;                      // B |e|^2 / (A1 + A2)
  double restAngle = 0;
};

// 1/2 K (theta - theta0)^2 with a Gauss-Newton Hessian.
void evalHingeTerm(const HingeTerm& t, const std::array<V3, 4>& x, TermEval<4>& out);

// One-sided spring 1/2 K (d - target)^2 for d > target between x[0] and x[1].
void evalSeamTerm(const std::array<V3, 2>& x, double target, double K, TermEval<2>& out);

// Smoothed Coulomb friction for one particle in contact (Li et al. 2020, IPC): D = muLambda * f0(|u|) with
// u = (I - n n^T)(x - x0). f0 is C1: quadratic-cubic below the slip length eps, |u| above it, so the force grows
// to muLambda over eps and stays there while sliding. The Hessian is the exact one, which is PSD.
void evalFrictionTerm(const V3& x, const V3& x0, const V3& n, double muLambda, double eps, TermEval<1>& out);

}  // namespace drape::sim
