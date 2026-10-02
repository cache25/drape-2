#include "stretch.hpp"

namespace drape::sim {

void accumulateStretch(const StretchElement& e, int local, const std::vector<Eigen::Vector3f>& x,
                       const FabricPhysical& f, float limitScale, Eigen::Vector3f& force, Eigen::Matrix3f& hessian) {
  const std::array<Eigen::Vector3f, 3> xs{x[e.v[0]], x[e.v[1]], x[e.v[2]]};
  const double k[3] = {f.stretchWarp, f.stretchWeft, f.stretchBias};
  const double limit[2] = {f.strainLimitWarp, f.strainLimitWeft};
  std::array<Eigen::Vector3f, 3> g;
  float C = 0;
  for (int which = 0; which < 3; ++which) {
    if (k[which] <= 0) continue;
    evalStretch<float>(xs, e.dmInv, which, C, g);
    const float K = static_cast<float>(k[which]) * e.area;
    force -= K * C * g[local];
    hessian += K * g[local] * g[local].transpose();
    if (which < 2 && limit[which] > 0) {
      const float excess = C - static_cast<float>(limit[which]);
      if (excess > 0) {
        const float KL = limitScale * K;
        force -= KL * excess * g[local];
        hessian += KL * g[local] * g[local].transpose();
      }
    }
  }
}

}  // namespace drape::sim
