#pragma once

#include <array>
#include <vector>

#include "drape/core/math.hpp"

namespace drape {

// Signed distance to the body on a regular grid (negative inside), clamped to [-band, band].
struct CollisionField {
  Vec3d origin{0, 0, 0};
  double cellSize = 0.004;
  std::array<int, 3> dims{0, 0, 0};
  double band = 0.02;
  std::vector<float> values;  // samples at origin + (i,j,k) * cellSize, x fastest

  double sample(const Vec3d& p) const;   // trilinear; any p outside the sampled box -> +band
  Vec3d gradient(const Vec3d& p) const;  // central differences with step cellSize, normalized; zero if |g| < 1e-9
};

}  // namespace drape
