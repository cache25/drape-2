#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <string_view>

#include "drape/core/body_regions.hpp"
#include "drape/core/math.hpp"
#include "drape/core/triangle_mesh.hpp"

namespace drape::avatar {

enum class BodyType { Male, Female };

std::string_view bodyName(BodyType type);  // "male" | "female"

struct Landmarks {
  std::map<std::string, Vec3d> points;
};

// A committed mannequin from assets/mannequins (built by tools/mannequin/build_mannequins.py):
// metres, Y up, facing +Z, lowest vertex at y = 0, the character's left at +x.
struct Mannequin {
  TriangleMesh mesh;                     // closed manifold (checked on load); one component in the committed assets
  BodyRegions regions;                   // all eight regions
  Landmarks landmarks;
  std::map<std::string, double> girths;  // chest, waist, hip, neck (m)
  double height = 0;
};

// Reads <mannequinDir>/<body>.obj and <body>.json. The first overload reads DRAPE_ASSET_DIR/mannequins.
// A missing or malformed file throws std::runtime_error whose message starts with the file's absolute path
// (std::filesystem::absolute, symlinks not resolved; and the line number, for OBJ syntax errors). An OBJ that
// is empty, truncated or otherwise not a closed manifold throws "<obj path>: mesh is not a closed manifold".
// The JSON must have an integer "format" of 1, all eight regions, every landmark the builder writes and the
// four girths; a missing one throws "<json path>: missing landmark 'chest'" (or region, girth).
Mannequin loadMannequin(BodyType type);
Mannequin loadMannequin(BodyType type, const std::filesystem::path& mannequinDir);

}  // namespace drape::avatar
