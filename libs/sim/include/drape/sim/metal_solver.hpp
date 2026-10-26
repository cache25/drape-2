#pragma once

#include <filesystem>
#include <memory>

#include "drape/sim/solver.hpp"

namespace drape::sim {

// The GPU solver on Apple GPUs (Phase B spec 5): GpuSolverCore over a MetalDevice. Mac only.
//
// The kernels ship as one Metal source (drape_kernels.metal, combined at build time) that the system Metal framework
// compiles at construction, so no Metal toolchain is needed on the machine.
class MetalSolver final : public Solver {
 public:
  // Throws std::runtime_error: "metal: no Metal device", or "metal: cannot load <absolute path>: <reason>".
  explicit MetalSolver(const std::filesystem::path& kernelSource = defaultKernelSourcePath());
  ~MetalSolver() override;

  // drape_kernels.metal next to the executable, else in <exe>/../Resources (the app bundle), else the build output.
  static std::filesystem::path defaultKernelSourcePath();

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
  std::unique_ptr<Solver> core_;
};

}  // namespace drape::sim
