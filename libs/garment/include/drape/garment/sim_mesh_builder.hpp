#pragma once

#include <stdexcept>

#include "drape/core/pattern.hpp"
#include "drape/core/project.hpp"
#include "drape/core/sim_mesh.hpp"

namespace drape::garment {

double particleDistance(SimQuality q);  // Draft 0.020, Standard 0.010, Fine 0.005

class MeshError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Pattern -> grain-aligned triangle mesh with 1:1 seam vertex pairs (spec 5.3).
// Throws MeshError listing validation rule ids when the pattern is invalid.
SimMesh buildSimMesh(const Pattern& pattern, double particleDistance);

}  // namespace drape::garment
