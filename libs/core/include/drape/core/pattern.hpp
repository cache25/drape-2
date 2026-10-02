#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "drape/core/body_regions.hpp"
#include "drape/core/outline.hpp"

namespace drape {

using Id = std::string;

// How a piece is wrapped onto a body-region cylinder before assembly (spec 7.3).
struct Placement {
  BodyRegion region = BodyRegion::Torso;
  double centerAngle = 0;    // radians, angle of pattern x = 0
  double axialAtOrigin = 0;  // meters along the axis for pattern y = 0
  int axialSign = 1;         // +1: pattern +y runs along +axis
  double offset = 0.06;      // radial distance outside the region radius (m)
  std::optional<double> wrapRadius;  // absolute wrap radius, overrides radius + offset
  bool operator==(const Placement&) const = default;
};

struct LayerRelation {
  Id basePiece;
  int order = 1;
  bool operator==(const LayerRelation&) const = default;
};

struct ElasticSpec {
  double factor = 1;
  Vec2d direction{1, 0};
  bool operator==(const ElasticSpec&) const = default;
};

struct PatternPiece {
  Id id;
  std::string name;
  Outline outline;  // CCW, seen from outside the garment
  Vec2d grainline{0, 1};
  std::string fabricSlot;
  Placement placement;
  std::optional<LayerRelation> layer;
  std::optional<ElasticSpec> elastic;
  bool mirrored = false;
  bool operator==(const PatternPiece&) const = default;
};

// Arc-length range [start, end] on a piece outline; `reversed` means the edge is traversed end -> start.
struct EdgeRef {
  Id piece;
  double start = 0;
  double end = 0;
  bool reversed = false;
  bool operator==(const EdgeRef&) const = default;
};

enum class SeamKind { Normal, ClosedZipper, ClosedFly };

struct Seam {
  Id id;
  EdgeRef a, b;
  double ease = 0;
  SeamKind kind = SeamKind::Normal;
  bool operator==(const Seam&) const = default;
};

enum class InternalLineKind { Fold, Quilting, Topstitch };

struct Color {
  float r = 1, g = 1, b = 1;  // sRGB, 0..1
  bool operator==(const Color&) const = default;
};

struct InternalLine {
  Id piece;
  std::vector<Vec2d> polyline;
  InternalLineKind kind = InternalLineKind::Fold;
  double foldAngle = 0;
  double stiffnessMultiplier = 1;
  double topstitchOffset = 0;
  double stitchesPerInch = 0;
  Color threadColor;
  double threadThickness = 0;
  bool operator==(const InternalLine&) const = default;
};

enum class TrimKind { Rivet, Button, ZipperStrip };

struct Trim {
  TrimKind kind = TrimKind::Rivet;
  Id piece;
  Vec2d position{0, 0};
  Id seam;
  double size = 0;
  std::string material;
  bool operator==(const Trim&) const = default;
};

struct Pattern {
  std::vector<PatternPiece> pieces;
  std::vector<Seam> seams;
  std::vector<InternalLine> internalLines;
  std::vector<Trim> trims;

  const PatternPiece* findPiece(std::string_view id) const;
  bool operator==(const Pattern&) const = default;
};

// Edge covering the contiguous named segments [first..last] of `piece`.
// Throws std::invalid_argument for unknown names or when last comes before first.
EdgeRef segmentEdge(const PatternPiece& piece, std::string_view first, std::string_view last, bool reversed);

}  // namespace drape
