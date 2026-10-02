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

}  // namespace drape::sim
