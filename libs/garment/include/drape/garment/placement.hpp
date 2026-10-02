#pragma once

#include <stdexcept>
#include <vector>

#include "drape/core/body_regions.hpp"
#include "drape/core/pattern.hpp"
#include "drape/core/sim_mesh.hpp"

namespace drape::garment {

class PlacementError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Wraps every piece onto its body-region cylinder (spec 7.3); one world position per sim vertex.
// Throws PlacementError naming the region when a piece's region is missing.
std::vector<Vec3d> placePattern(const Pattern& pattern, const SimMesh& mesh, const BodyRegions& regions);

}  // namespace drape::garment
