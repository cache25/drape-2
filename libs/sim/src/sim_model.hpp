#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "drape/sim/solver.hpp"
#include "energies.hpp"
#include "seams.hpp"

namespace drape::sim {

inline constexpr double kContactReach = 0.01;   // m: particles closer than r + this get a contact plane each substep
inline constexpr double kContactSnap = 0.0005;  // m: a particle this close above its plane is held on it

// Scene setup and per-particle state shared by every Solver backend: particles, energy terms, the block-sparse
// Hessian structure, phases and seam assembly bookkeeping, plus stats and snapshots. Solvers add their own
// work arrays on top.
struct SimModel {
  struct Garment {
    GarmentId id;
    ParticleRange range;
    SimPhase phase = SimPhase::Settling;
    double assemblyTime = 0;
    std::uint32_t fabricOffset = 0;
    std::uint32_t pieceCount = 0;
  };
  struct HingeSide {
    std::uint32_t fabric = 0;
    Vec2d edgeDir{1, 0}, grain{0, 1};
  };

  SimSettings settings;
  const CollisionField* body = nullptr;
  std::vector<Garment> garments;
  std::vector<FabricPhysical> fabrics;  // garment.fabricOffset + piece index

  // Particle state.
  std::vector<Vec3f> x, v;
  std::vector<double> mass, vertexArea;
  std::vector<Vec3d> fExt;
  std::vector<std::uint32_t> pFabric, pGarment;
  std::vector<std::uint8_t> pinned;
  std::vector<std::pair<std::uint32_t, Vec3f>> pins;

  // Energy terms.
  std::vector<StretchTerm> stretch;
  std::vector<HingeTerm> hinges;
  std::vector<std::array<HingeSide, 2>> hingeSides;
  std::vector<double> hingeGeom;  // |e|^2 / (A1 + A2)
  std::vector<SeamSpring> seams;

  // Block-sparse Hessian structure (CSR over 3x3 blocks) and each term's block slots.
  std::vector<std::uint32_t> rowStart, colIndex, diagSlot;
  std::vector<std::array<std::uint32_t, 9>> stretchSlots;
  std::vector<std::array<std::uint32_t, 16>> hingeSlots;
  std::vector<std::array<std::uint32_t, 4>> seamSlots;

  // Body normal force per particle from the last solve (newtons); the next substep's friction uses it.
  std::vector<double> contactForce;

  // Throws std::invalid_argument / std::out_of_range on inconsistent scenes (as CpuSolver::build always has).
  static SimModel build(const SimScene& scene);

  void updateFabric(const GarmentId& g, const std::vector<FabricPhysical>& pieceFabric);
  void setPhase(const GarmentId& g, SimPhase phase);
  std::size_t garmentIndex(const GarmentId& id) const;  // throws std::out_of_range for unknown ids
  ParticleRange range(const GarmentId& id) const;

  double contactRadius(std::size_t i) const { return settings.collisionOffset + 0.5 * fabrics[pFabric[i]].thickness; }
  double seamTarget(const SeamSpring& sp) const;
  double hingeStiffness(std::size_t hgi) const;
  void updateMass(std::uint32_t i) { mass[i] = std::max(vertexArea[i] * fabrics[pFabric[i]].weight, 1e-12); }
  std::uint32_t slot(std::uint32_t row, std::uint32_t col) const;

  SimStats stats() const;
  Snapshot snapshot() const;
  void restore(const Snapshot& s);

 private:
  void buildStructure();
};

}  // namespace drape::sim
