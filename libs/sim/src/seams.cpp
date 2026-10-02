#include "seams.hpp"

namespace drape::sim {

void accumulateSeam(const SeamSpring& s, int side, float target, float K, const std::vector<Eigen::Vector3f>& x,
                    Eigen::Vector3f& force, Eigen::Matrix3f& hessian) {
  const Eigen::Vector3f diff = x[s.a] - x[s.b];
  const float d = diff.norm();
  if (d <= target) return;
  if (d < 1e-9f) {  // target is 0 here: the energy is 1/2 K d^2, Hessian K I
    hessian += K * Eigen::Matrix3f::Identity();
    return;
  }
  const Eigen::Vector3f n = diff / d;
  const float sign = side == 0 ? 1.0f : -1.0f;
  force -= sign * K * (d - target) * n;
  const Eigen::Matrix3f nn = n * n.transpose();
  hessian += K * (nn + ((d - target) / d) * (Eigen::Matrix3f::Identity() - nn));
}

}  // namespace drape::sim
