#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Dense>

namespace drape::sim {

struct SeamSpring {
  std::uint32_t a = 0, b = 0;  // global particle indices
  std::uint32_t garment = 0;
  float startGap = 0;          // gap when the garment last entered Assembling
};

// Adds the force and Hessian of the one-sided spring 1/2 K (d - target)^2 (only when d > target)
// for side 0 (particle a) or side 1 (particle b).
void accumulateSeam(const SeamSpring& s, int side, float target, float K, const std::vector<Eigen::Vector3f>& x,
                    Eigen::Vector3f& force, Eigen::Matrix3f& hessian);

}  // namespace drape::sim
