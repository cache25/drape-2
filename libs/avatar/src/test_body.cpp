#include "drape/avatar/test_body.hpp"

#include <cmath>

#include "drape/avatar/primitives.hpp"

namespace drape::avatar {

TestBody makeTestBody() {
  const double s = std::sqrt(0.5);  // sin/cos 45°
  const Vec3d dL(s, -s, 0), dR(-s, -s, 0);
  const Vec3d shoulderL(0.19, 1.37, 0), shoulderR(-0.19, 1.37, 0);

  TestBody b;
  b.components.push_back(ellipticCapsule(0.85, 1.38, 0.17, 0.11, 0.08));
  b.components.push_back(capsule({0, 1.40, 0}, {0, 1.58, 0}, 0.055));
  b.components.push_back(icosphere({0, 1.68, 0}, 0.10, 3));
  b.components.push_back(capsule(shoulderL, shoulderL + 0.55 * dL, 0.045));
  b.components.push_back(capsule(shoulderR, shoulderR + 0.55 * dR, 0.045));

  b.regions.cylinders[BodyRegion::Torso] = {{0, 1.47, 0}, {0, 1, 0}, {0, 0, 1}, 0.17};
  b.regions.cylinders[BodyRegion::Neck] = {{0, 1.47, 0}, {0, 1, 0}, {0, 0, 1}, 0.055};
  b.regions.cylinders[BodyRegion::ArmLeft] = {shoulderL, dL, {s, s, 0}, 0.045};
  b.regions.cylinders[BodyRegion::ArmRight] = {shoulderR, dR, {-s, s, 0}, 0.045};
  return b;
}

}  // namespace drape::avatar
