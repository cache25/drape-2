#include "drape/sim/solver_factory.hpp"

#include <stdexcept>
#include <string>

#include "drape/sim/cpu_solver.hpp"
#include "gpu/emulated_device.hpp"
#include "gpu/gpu_solver_core.hpp"

#if defined(DRAPE_HAVE_METAL)
#include "drape/sim/metal_solver.hpp"
#endif

namespace drape::sim {

std::string_view backendName(SolverBackend b) {
  switch (b) {
    case SolverBackend::Cpu: return "cpu";
    case SolverBackend::GpuEmulated: return "gpu-emulated";
    case SolverBackend::Metal: return "metal";
  }
  return "unknown";
}

bool isAvailable(SolverBackend b) {
  switch (b) {
    case SolverBackend::Cpu: return true;
    case SolverBackend::GpuEmulated: return true;
    case SolverBackend::Metal: {
#if defined(DRAPE_HAVE_METAL)
      // Checked once: a Metal device exists and the kernels compile.
      static const bool ok = [] {
        try {
          MetalSolver probe;
          return true;
        } catch (const std::exception&) {
          return false;
        }
      }();
      return ok;
#else
      return false;
#endif
    }
  }
  return false;
}

std::unique_ptr<Solver> makeSolver(SolverBackend b) {
  switch (b) {
    case SolverBackend::Cpu: return std::make_unique<CpuSolver>();
    case SolverBackend::GpuEmulated: return std::make_unique<GpuSolverCore>(std::make_unique<EmulatedDevice>());
    case SolverBackend::Metal:
#if defined(DRAPE_HAVE_METAL)
      return std::make_unique<MetalSolver>();
#else
      throw std::runtime_error("metal: not built on this platform");
#endif
  }
  throw std::runtime_error("unknown solver backend");
}

}  // namespace drape::sim
