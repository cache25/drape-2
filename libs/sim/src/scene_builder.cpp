#include "drape/sim/scene_builder.hpp"

#include <stdexcept>

namespace drape::sim {

SimGarment makeSimGarment(const GarmentId& id, const Pattern& pattern, SimMesh mesh, std::vector<Vec3d> positions,
                          const std::map<std::string, FabricPhysical>& fabricBySlot) {
  SimGarment g;
  g.id = id;
  for (const auto& piece : pattern.pieces) {
    auto it = fabricBySlot.find(piece.fabricSlot);
    if (it == fabricBySlot.end()) {
      throw std::invalid_argument("garment '" + id + "': no fabric for slot '" + piece.fabricSlot + "' (piece '" +
                                  piece.id + "')");
    }
    g.pieceFabric.push_back(it->second);
  }
  g.mesh = std::move(mesh);
  g.initialPositions = std::move(positions);
  return g;
}

}  // namespace drape::sim
