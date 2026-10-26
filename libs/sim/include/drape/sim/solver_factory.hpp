#pragma once

#include <memory>
#include <string_view>

#include "drape/sim/solver.hpp"

namespace drape::sim {

// Cpu: the double-precision reference solver. GpuEmulated: the GPU algorithm run on the CPU (32-bit, for tests and
// for platforms without Metal). Metal: the GPU algorithm on an Apple GPU.
enum class SolverBackend { Cpu, GpuEmulated, Metal };

std::string_view backendName(SolverBackend b);  // "cpu" | "gpu-emulated" | "metal"

// Metal is available only in a Mac build, with a Metal device and the kernel library present.
bool isAvailable(SolverBackend b);

// Throws std::runtime_error naming the backend and the cause when it is unavailable.
std::unique_ptr<Solver> makeSolver(SolverBackend b);

}  // namespace drape::sim
