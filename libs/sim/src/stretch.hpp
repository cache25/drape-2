#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "drape/core/fabric.hpp"

namespace drape::sim {

// Anisotropic in-plane strain of one triangle against its grain-aligned rest shape (spec 5.4).
// which: 0 warp (|f_u| - 1), 1 weft (|f_v| - 1), 2 bias (n_u . n_v). F = Ds * dmInv.
template <class S>
void evalStretch(const std::array<Eigen::Matrix<S, 3, 1>, 3>& x, const Eigen::Matrix<S, 2, 2>& dmInv, int which,
                 S& C, std::array<Eigen::Matrix<S, 3, 1>, 3>& grad) {
  using V = Eigen::Matrix<S, 3, 1>;
  const V e1 = x[1] - x[0];
  const V e2 = x[2] - x[0];
  const V fu = e1 * dmInv(0, 0) + e2 * dmInv(1, 0);
  const V fv = e1 * dmInv(0, 1) + e2 * dmInv(1, 1);
  const S lu = fu.norm();
  const S lv = fv.norm();
  const S tiny = S(1e-12);
  V dfu = V::Zero(), dfv = V::Zero();
  C = S(0);
  if (which == 0) {
    if (lu > tiny) {
      C = lu - S(1);
      dfu = fu / lu;
    }
  } else if (which == 1) {
    if (lv > tiny) {
      C = lv - S(1);
      dfv = fv / lv;
    }
  } else if (lu > tiny && lv > tiny) {
    const V nu = fu / lu;
    const V nv = fv / lv;
    const S c = nu.dot(nv);
    C = c;
    dfu = (nv - c * nu) / lu;
    dfv = (nu - c * nv) / lv;
  }
  grad[1] = dfu * dmInv(0, 0) + dfv * dmInv(0, 1);
  grad[2] = dfu * dmInv(1, 0) + dfv * dmInv(1, 1);
  grad[0] = -(grad[1] + grad[2]);
}

struct StretchElement {
  std::array<std::uint32_t, 3> v{};
  Eigen::Matrix2f dmInv = Eigen::Matrix2f::Identity();
  float area = 0;
  std::uint32_t fabric = 0;  // index into the solver's fabric table
};

// Adds, for vertex `local` (0..2) of the element, the force -dE/dx and the Gauss-Newton Hessian of the
// element's stretch energies: 1/2 K C^2 for warp, weft and bias (K = k * area), plus the one-sided
// strain-limit energies (K = limitScale * k * area) when warp or weft stretch exceeds the fabric limit.
void accumulateStretch(const StretchElement& e, int local, const std::vector<Eigen::Vector3f>& x,
                       const FabricPhysical& f, float limitScale, Eigen::Vector3f& force, Eigen::Matrix3f& hessian);

}  // namespace drape::sim
