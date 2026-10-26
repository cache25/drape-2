#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "drape/core/math.hpp"

namespace drape::render {

// Per-vertex normals for shading (spec 9): area-weighted (the triangles' unnormalized cross products summed per
// vertex), then each weld pair (a seam's two coincident particles) gets the normalized sum of both sides, so seams
// shade smoothly. A vertex with no area gets (0, 1, 0).
std::vector<Vec3f> vertexNormals(std::span<const Vec3f> positions,
                                 std::span<const std::array<std::uint32_t, 3>> triangles,
                                 std::span<const std::pair<std::uint32_t, std::uint32_t>> weldPairs);

}  // namespace drape::render
