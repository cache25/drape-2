#pragma once

#include <map>
#include <string>
#include <vector>

#include "drape/core/pattern.hpp"
#include "drape/sim/solver.hpp"

namespace drape::sim {

// pieceFabric[i] = fabricBySlot.at(pattern.pieces[i].fabricSlot); a missing slot -> std::invalid_argument naming it.
SimGarment makeSimGarment(const GarmentId& id, const Pattern& pattern, SimMesh mesh, std::vector<Vec3d> positions,
                          const std::map<std::string, FabricPhysical>& fabricBySlot);

}  // namespace drape::sim
