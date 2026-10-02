#pragma once

#include <memory>

#include "drape/sim/solver.hpp"

namespace drape::sim {

// Reference solver (spec 5): implicit Euler, each substep minimized by Newton's method with a
// block-Jacobi preconditioned conjugate-gradient linear solve and a backtracking line search.
// Particle state is float; the solve runs in double. Deterministic: same inputs, bitwise-identical results.
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
