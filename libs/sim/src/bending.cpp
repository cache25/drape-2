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

}  // namespace drape::sim
