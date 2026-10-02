#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "drape/core/math.hpp"

namespace drape {

// Closed surface mesh; triangles are CCW seen from outside.
struct TriangleMesh {
  std::vector<Vec3d> vertices;
  std::vector<std::array<std::uint32_t, 3>> triangles;
};

double signedVolume(const TriangleMesh& m);
// Every undirected edge is used by exactly two triangles, in opposite directions.
bool isClosedManifold(const TriangleMesh& m);

}  // namespace drape
