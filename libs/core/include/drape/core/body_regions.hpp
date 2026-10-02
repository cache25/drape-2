#pragma once

#include <map>

#include "drape/core/math.hpp"

namespace drape {

enum class BodyRegion { Torso, Neck, Head, ArmLeft, ArmRight, LegLeft, LegRight, Waist };

// A cylinder pieces are wrapped onto for initial placement. Origins are anchors:
// Torso and Neck sit on the body axis at neck-base height, arms at the shoulder joints.
struct RegionCylinder {
  Vec3d origin{0, 0, 0};
  Vec3d axis{0, 1, 0};
  Vec3d e0{0, 0, 1};  // angle-zero direction, perpendicular to axis
  double radius = 0;

  Vec3d e1() const { return axis.cross(e0); }
  bool operator==(const RegionCylinder&) const = default;
};

struct BodyRegions {
  std::map<BodyRegion, RegionCylinder> cylinders;
  bool operator==(const BodyRegions&) const = default;
};

}  // namespace drape
