#include "drape/garment/shapes.hpp"

namespace drape::garment {

PatternPiece rectanglePiece(const Id& id, double width, double height, const Vec2d& grainline,
                            const std::string& fabricSlot) {
  PatternPiece p;
  p.id = id;
  p.name = id;
  p.outline.add("bottom", CubicBezier::line({0, 0}, {width, 0}));
  p.outline.add("right", CubicBezier::line({width, 0}, {width, height}));
  p.outline.add("top", CubicBezier::line({width, height}, {0, height}));
  p.outline.add("left", CubicBezier::line({0, height}, {0, 0}));
  p.grainline = grainline;
  p.fabricSlot = fabricSlot;
  return p;
}

}  // namespace drape::garment
