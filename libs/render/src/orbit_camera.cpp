#include "drape/render/orbit_camera.hpp"

#include <algorithm>
#include <cmath>

namespace drape::render {

namespace {
Vec3d offsetDir(double yaw, double pitch) {
  return {std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw)};
}
}  // namespace

void OrbitCamera::frame(const Vec3d& boundsMin, const Vec3d& boundsMax, double verticalFovRad, double aspect) {
  target = 0.5 * (boundsMin + boundsMax);
  const double radius = 0.5 * (boundsMax - boundsMin).norm() * 1.1;
  const double hfov = 2.0 * std::atan(std::tan(verticalFovRad / 2) * aspect);
  const double fov = std::min(verticalFovRad, hfov);
  distance = std::clamp(radius / std::sin(fov / 2), kMinDistance, kMaxDistance);
  yaw = 0;
  pitch = 0;
}

void OrbitCamera::orbit(double dxPixels, double dyPixels) {
  yaw -= dxPixels * kRadiansPerPixel;
  pitch = std::clamp(pitch + dyPixels * kRadiansPerPixel, -kMaxPitch, kMaxPitch);
}

void OrbitCamera::pan(double dxPixels, double dyPixels, double viewportHeightPx, double verticalFovRad) {
  const double metresPerPixel = 2.0 * distance * std::tan(verticalFovRad / 2) / std::max(viewportHeightPx, 1.0);
  const Vec3d forward = -offsetDir(yaw, pitch);
  const Vec3d right = forward.cross(Vec3d(0, 1, 0)).normalized();
  const Vec3d camUp = right.cross(forward);
  target += (-dxPixels * right + dyPixels * camUp) * metresPerPixel;
}

void OrbitCamera::zoom(double steps) { distance = std::clamp(distance * std::pow(0.9, steps), kMinDistance, kMaxDistance); }

Vec3d OrbitCamera::eye() const { return target + distance * offsetDir(yaw, pitch); }

Vec3d OrbitCamera::up() const {
  const Vec3d forward = -offsetDir(yaw, pitch);
  const Vec3d right = forward.cross(Vec3d(0, 1, 0)).normalized();
  return right.cross(forward);
}

}  // namespace drape::render
