#include "drape/garment/placement.hpp"

#include <cmath>
#include <string>

namespace drape::garment {

namespace {
const char* regionName(BodyRegion r) {
  switch (r) {
    case BodyRegion::Torso: return "Torso";
    case BodyRegion::Neck: return "Neck";
    case BodyRegion::Head: return "Head";
    case BodyRegion::ArmLeft: return "ArmLeft";
    case BodyRegion::ArmRight: return "ArmRight";
    case BodyRegion::LegLeft: return "LegLeft";
    case BodyRegion::LegRight: return "LegRight";
    case BodyRegion::Waist: return "Waist";
  }
  return "Unknown";
}
}  // namespace

std::vector<Vec3d> placePattern(const Pattern& pattern, const SimMesh& mesh, const BodyRegions& regions) {
  std::vector<const RegionCylinder*> cylinders;
  for (const auto& piece : pattern.pieces) {
    auto it = regions.cylinders.find(piece.placement.region);
    if (it == regions.cylinders.end()) {
      throw PlacementError("piece '" + piece.id + "' needs body region " + regionName(piece.placement.region) +
                           ", which this body does not provide");
    }
    cylinders.push_back(&it->second);
  }
  std::vector<Vec3d> out(mesh.rest.size());
  for (std::size_t v = 0; v < mesh.rest.size(); ++v) {
    const auto& piece = pattern.pieces[mesh.vertexPiece[v]];
    const Placement& pl = piece.placement;
    const RegionCylinder& c = *cylinders[mesh.vertexPiece[v]];
    const double R = pl.wrapRadius ? *pl.wrapRadius : c.radius + pl.offset;
    const Vec2d& r = mesh.rest[v];
    const double theta = pl.centerAngle + pl.axialSign * r.x() / R;
    const double axial = pl.axialAtOrigin + pl.axialSign * r.y();
    out[v] = c.origin + axial * c.axis + R * (std::cos(theta) * c.e0 + std::sin(theta) * c.e1());
  }
  return out;
}

}  // namespace drape::garment
