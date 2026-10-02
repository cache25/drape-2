#include "bending.hpp"

#include <algorithm>

namespace drape::sim {

BendCoefficients bendingCoefficients(double bendWarp, double bendWeft) {
  const double floor = 0.05 * std::min(bendWarp, bendWeft);
  return {std::max((3.0 * bendWarp - bendWeft) / 2.0, floor), std::max(bendWeft, floor)};
}

double edgeBendStiffness(const BendCoefficients& c, const Vec2d& restEdgeDir, const Vec2d& grain) {
  const Vec2d d = restEdgeDir.normalized();
  const Vec2d u = grain.normalized();
  const double cosPhi = d.dot(u);
  const double sinPhi = d.x() * u.y() - d.y() * u.x();
  return c.p * sinPhi * sinPhi + c.q * cosPhi * cosPhi;
}

void accumulateHinge(const Hinge& h, int local, const std::vector<Eigen::Vector3f>& x, Eigen::Vector3f& force,
                     Eigen::Matrix3f& hessian) {
  const std::array<Eigen::Vector3f, 4> xs{x[h.v[0]], x[h.v[1]], x[h.v[2]], x[h.v[3]]};
  float theta = 0;
  std::array<Eigen::Vector3f, 4> g;
  evalDihedral<float>(xs, theta, g);
  force -= h.K * (theta - h.restAngle) * g[local];
  hessian += h.K * g[local] * g[local].transpose();
}

}  // namespace drape::sim
