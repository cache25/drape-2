#pragma once

#include <span>

#include "drape/core/collision_field.hpp"
#include "drape/core/triangle_mesh.hpp"

namespace drape::avatar {

struct BakeOptions {
  double cellSize = 0.004;
  double padding = 0.05;
  double band = 0.02;
};

// Signed distance field of the union of closed components (min of their fields).
// Throws std::invalid_argument if a component is not a closed manifold.
CollisionField bakeCollisionField(std::span<const TriangleMesh> components, const BakeOptions& options = {});

}  // namespace drape::avatar
