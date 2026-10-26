#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "drape/avatar/mannequin.hpp"
#include "drape/core/collision_field.hpp"
#include "drape/core/pattern.hpp"
#include "drape/core/sim_mesh.hpp"
#include "drape/garment/generator.hpp"
#include "drape/sim/solver.hpp"

namespace drape::session {

// The Phase A test tee on a mannequin, ready to build into a Solver. Not copyable or movable once built:
// scene.body points at field.
struct TeeScene {
  avatar::BodyType bodyType = avatar::BodyType::Male;
  SimQuality quality = SimQuality::Standard;
  avatar::Mannequin mannequin;
  CollisionField field;
  Pattern pattern;
  SimMesh mesh;
  sim::SimScene scene;  // one garment, "tee"

  TeeScene() = default;
  TeeScene(const TeeScene&) = delete;
  TeeScene& operator=(const TeeScene&) = delete;
};

// The tee each body wears: the men's test tee on the male, the women's cut on the female.
const garment::GarmentGenerator& teeGenerator(avatar::BodyType body);

// Loads the mannequin (DRAPE_ASSET_DIR/mannequins unless mannequinDir is given), bakes its collision field, generates
// the tee at `size`, meshes it at the quality's particle distance and places it around the body. Fabrics: "body"
// testJersey, "rib" testRib. Loader errors propagate (std::runtime_error naming the absolute path).
std::unique_ptr<const TeeScene> makeTeeScene(avatar::BodyType body, SimQuality quality, const std::string& size = "M",
                                             const std::optional<std::filesystem::path>& mannequinDir = {});

}  // namespace drape::session
