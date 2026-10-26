#pragma once

#include <memory>

#include "drape/sim/solver.hpp"
#include "gpu/compute_device.hpp"

namespace drape::sim {

// The GPU solver (spec 5.1, 5.5; Phase B spec 5): CpuSolver's algorithm with every per-particle and per-term step in
// the kernels.h kernels on a ComputeDevice (Metal, or the CPU emulation), and the accept/reject sums of the line search
// in double on the CPU. Scene setup, stats and snapshots are SimModel's, shared with CpuSolver; the device holds the
// live positions, velocities and contact forces.
class GpuSolverCore final : public Solver {
 public:
  explicit GpuSolverCore(std::unique_ptr<ComputeDevice> device);
  ~GpuSolverCore() override;
  GpuSolverCore(const GpuSolverCore&) = delete;
  GpuSolverCore& operator=(const GpuSolverCore&) = delete;

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
