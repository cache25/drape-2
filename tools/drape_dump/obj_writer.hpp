#pragma once

#include <filesystem>
#include <span>
#include <string>
#include <utility>

#include "drape/core/triangle_mesh.hpp"

namespace drape::tools {

// Writes one `o <name>` block per object to a Wavefront OBJ file; face indices are 1-based and global across
// objects. Throws std::runtime_error if the file cannot be written.
void writeObj(const std::filesystem::path& path, std::span<const std::pair<std::string, TriangleMesh>> objects);

}  // namespace drape::tools
