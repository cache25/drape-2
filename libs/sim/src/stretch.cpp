#include "stretch.hpp"

namespace drape::sim {

namespace {

// Coefficient of vertex `local` in f_u (column 0) or f_v (column 1): f = sum_k d_k x_k.
float vertexCoefficient(const Eigen::Matrix2f& dmInv, int column, int local) {
  if (local == 1) return dmInv(0, column);
  if (local == 2) return dmInv(1, column);
  return -(dmInv(0, column) + dmInv(1, column));
}

// Geometric stiffness of 1/2 K C^2 with C = |f| - c0, for C > 0: K C d^2 (I - n n^T) / |f|.
// Dropping it (pure Gauss-Newton) leaves taut cloth with no out-of-plane stiffness and the Newton step blows up.
void addTension(Eigen::Matrix3f& hessian, float K, float C, float d, const Eigen::Vector3f& f) {
  if (C <= 0) return;
  const float len = f.norm();
  if (len < 1e-12f) return;
  const Eigen::Vector3f n = f / len;
  hessian += (K * C * d * d / len) * (Eigen::Matrix3f::Identity() - n * n.transpose());
}

}  // namespace

void accumulateStretch(const StretchElement& e, int local, const std::vector<Eigen::Vector3f>& x,
                       const FabricPhysical& f, float limitScale, Eigen::Vector3f& force, Eigen::Matrix3f& hessian) {
  const std::array<Eigen::Vector3f, 3> xs{x[e.v[0]], x[e.v[1]], x[e.v[2]]};
  const double k[3] = {f.stretchWarp, f.stretchWeft, f.stretchBias};
  const double limit[2] = {f.strainLimitWarp, f.strainLimitWeft};
  const Eigen::Vector3f e1 = xs[1] - xs[0], e2 = xs[2] - xs[0];
  const Eigen::Vector3f fcol[2] = {e1 * e.dmInv(0, 0) + e2 * e.dmInv(1, 0), e1 * e.dmInv(0, 1) + e2 * e.dmInv(1, 1)};
  std::array<Eigen::Vector3f, 3> g;
  float C = 0;
  for (int which = 0; which < 3; ++which) {
    if (k[which] <= 0) continue;
    evalStretch<float>(xs, e.dmInv, which, C, g);
    const float K = static_cast<float>(k[which]) * e.area;
    force -= K * C * g[local];
    hessian += K * g[local] * g[local].transpose();
    if (which < 2) {
      const float d = vertexCoefficient(e.dmInv, which, local);
      addTension(hessian, K, C, d, fcol[which]);
      if (limit[which] > 0) {
        const float excess = C - static_cast<float>(limit[which]);
        if (excess > 0) {
          const float KL = limitScale * K;
          force -= KL * excess * g[local];
          hessian += KL * g[local] * g[local].transpose();
          addTension(hessian, KL, excess, d, fcol[which]);
        }
      }
    }
  }
}

}  // namespace drape::sim
