#pragma once

#include <filesystem>

#include "drape/core/triangle_mesh.hpp"

namespace drape::avatar {

// Reads the `v x y z` and `f a b c` lines of a Wavefront OBJ (1-based indices, "a/b/c" tokens use
// the vertex index only); every other line is ignored. Throws std::runtime_error:
//   "<path>: cannot open"
//   "<path>:<line>: malformed vertex" | "malformed face index" | "face is not a triangle" |
//   "vertex index out of range"
TriangleMesh readObjTriangles(const std::filesystem::path& path);

}  // namespace drape::avatar
