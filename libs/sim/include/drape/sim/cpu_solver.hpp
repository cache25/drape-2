#pragma once

#include <memory>

#include "drape/sim/solver.hpp"

namespace drape::sim {

// Reference Vertex Block Descent solver (spec 5): per-vertex Newton steps on the implicit-Euler energy,
// processed by vertex color; float particle state. Deterministic: same inputs, bitwise-identical results.
class CpuSolver final : public Solver {
 public:
  CpuSolver();
  ~CpuSolver() override;
  CpuSolver(const CpuSolver&) = delete;
  CpuSolver& operator=(const CpuSolver&) = delete;

  void build(const SimScene& scene) override;
  void updateFabric(const GarmentId& g, const std::vector<FabricPhysical>& pieceFabric) override;
  void setPhase(const GarmentId& g, SimPhase phase) override;
  void step(const StepInput& in) override;
  void readPositions(std::span<Vec3f> out) const override;
  SimStats stats() const override;
  Snapshot snapshot() const override;
  void restore(const Snapshot& s) override;
  ParticleRange range(const GarmentId& g) const override;
  std::size_t particleCount() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace drape::sim
