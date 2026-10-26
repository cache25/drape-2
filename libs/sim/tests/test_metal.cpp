#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>
#include <string>

#include "drape/sim/metal_solver.hpp"
#include "drape/sim/solver_factory.hpp"
#include "gpu_parity_helpers.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

TEST(MetalSolver, MissingKernelSourceThrowsWithPath) {
  const std::filesystem::path missing = "/nonexistent/drape_kernels.metal";
  try {
    MetalSolver s(missing);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find(missing.string()), std::string::npos) << e.what();
  }
}

TEST(MetalSolver, Available) { EXPECT_TRUE(isAvailable(SolverBackend::Metal)); }

TEST(MetalSolver, MatchesEmulatedOneFrame) {
  const SimScene scene = hangingSheet().scene;
  MetalSolver metal;
  const auto emulated = makeSolver(SolverBackend::GpuEmulated);
  metal.build(scene);
  emulated->build(scene);
  metal.step({1.0 / 60.0, 1, 2});
  emulated->step({1.0 / 60.0, 1, 2});
  EXPECT_LT(maxDistance(positions(metal), positions(*emulated)), 1e-5);
}
