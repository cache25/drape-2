#pragma once

#include <string>

#include "drape/core/pattern.hpp"

namespace drape {

// Physical fabric properties in SI units (spec 6.1).
struct FabricPhysical {
  double stretchWarp = 0, stretchWeft = 0, stretchBias = 0;  // N/m
  double strainLimitWarp = 0, strainLimitWeft = 0;           // fraction (0.05 = 5%)
  double bendWarp = 0, bendWeft = 0;                         // N·m
  double weight = 0;                                         // kg/m²
  double thickness = 0;                                      // m
  double friction = 0, damping = 0;                          // 0..1
  bool operator==(const FabricPhysical&) const = default;
};

struct FabricVisual {
  std::string baseColorTexture, normalTexture, roughnessTexture;
  double textureScale = 0.01;  // meters per tile
  Color sheenColor;
  double sheenRoughness = 0.5;
  Color subsurfaceColor;
  double insideDarkening = 0.2;
  bool operator==(const FabricVisual&) const = default;
};

struct Fabric {
  Id id;
  std::string name;
  FabricPhysical physical;
  FabricVisual visual;
  bool operator==(const Fabric&) const = default;
};

}  // namespace drape
