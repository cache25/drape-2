#include "collision.hpp"

namespace drape::sim {

void collideBody(const CollisionField& body, float radius, float friction, const Eigen::Vector3f& xPrev,
                 Eigen::Vector3f& x) {
  const Eigen::Vector3d p = x.cast<double>();
  const double phi = body.sample(p);
  if (phi >= radius) return;
  const Eigen::Vector3f n = body.gradient(p).cast<float>();
  if (n.isZero()) return;
  const float depth = static_cast<float>(radius - phi);
  x += depth * n;
  const Eigen::Vector3f delta = x - xPrev;
  const Eigen::Vector3f tangential = delta - delta.dot(n) * n;
  const float t = tangential.norm();
  if (t <= friction * depth) {
    x -= tangential;
  } else if (t > 0) {
    x -= tangential * (friction * depth / t);
  }
}

}  // namespace drape::sim
