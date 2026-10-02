#pragma once

#include <string>

#include "drape/core/pattern.hpp"

namespace drape::garment {

// Rectangle CCW from (0,0) with segments "bottom", "right", "top", "left" and a default Placement.
PatternPiece rectanglePiece(const Id& id, double width, double height, const Vec2d& grainline = {0, 1},
                            const std::string& fabricSlot = "body");

}  // namespace drape::garment
