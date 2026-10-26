#include "drape/sim/metal_solver.hpp"

#include <mach-o/dyld.h>

#include <cstdint>
#include <string>
#include <vector>

#include "gpu/gpu_solver_core.hpp"
#include "gpu/metal_device.hpp"

namespace drape::sim {

std::filesystem::path MetalSolver::defaultKernelSourcePath() {
  const char* name = "drape_kernels.metal";
  std::uint32_t size = 0;
  _NSGetExecutablePath(nullptr, &size);
  std::vector<char> buf(size + 1, '\0');
  if (_NSGetExecutablePath(buf.data(), &size) == 0) {
    const std::filesystem::path exeDir = std::filesystem::path(buf.data()).parent_path();
    for (const auto& candidate : {exeDir / name, exeDir / ".." / "Resources" / name}) {
      std::error_code ec;
      if (std::filesystem::exists(candidate, ec)) return candidate;
    }
  }
  return DRAPE_KERNEL_SOURCE_PATH;
}

MetalSolver::MetalSolver(const std::filesystem::path& kernelSource)
    : core_(std::make_unique<GpuSolverCore>(std::make_unique<MetalDevice>(kernelSource))) {}

MetalSolver::~MetalSolver() = default;

void MetalSolver::build(const SimScene& scene) { core_->build(scene); }
void MetalSolver::updateFabric(const GarmentId& g, const std::vector<FabricPhysical>& f) { core_->updateFabric(g, f); }
void MetalSolver::setPhase(const GarmentId& g, SimPhase phase) { core_->setPhase(g, phase); }
void MetalSolver::step(const StepInput& in) { core_->step(in); }
void MetalSolver::readPositions(std::span<Vec3f> out) const { core_->readPositions(out); }
SimStats MetalSolver::stats() const { return core_->stats(); }
Snapshot MetalSolver::snapshot() const { return core_->snapshot(); }
void MetalSolver::restore(const Snapshot& s) { core_->restore(s); }
ParticleRange MetalSolver::range(const GarmentId& g) const { return core_->range(g); }
std::size_t MetalSolver::particleCount() const { return core_->particleCount(); }

}  // namespace drape::sim
