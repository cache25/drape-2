#pragma once

#include "drape/core/math.hpp"

namespace drape::render {

// Orbit camera around a target. yaw = pitch = 0 puts the eye on +Z of the target looking toward -Z, at the
// mannequin's face (it faces +Z). Drag to orbit, right-drag/two-finger drag to pan, scroll/pinch to zoom.
class OrbitCamera {
 public:
  Vec3d target{0, 0, 0};
  double distance = 3.0;  // m
  double yaw = 0.0;       // rad, about +Y
  double pitch = 0.0;     // rad, clamped to +-kMaxPitch

  static constexpr double kRadiansPerPixel = 0.005;
  static constexpr double kMaxPitch = 1.5;
  static constexpr double kMinDistance = 0.2, kMaxDistance = 20.0;

  // Centers on the box and backs off until its bounding sphere, plus 10%, fits both fields of view; resets the
  // view to the front.
  void frame(const Vec3d& boundsMin, const Vec3d& boundsMax, double verticalFovRad, double aspect);
  void orbit(double dxPixels, double dyPixels);
  // Moves the target in the view plane so the scene follows the cursor.
  void pan(double dxPixels, double dyPixels, double viewportHeightPx, double verticalFovRad);
  void zoom(double steps);  // distance *= 0.9^steps, clamped

  Vec3d eye() const;
  Vec3d up() const;  // the camera's orthonormal up vector
};

}  // namespace drape::render
