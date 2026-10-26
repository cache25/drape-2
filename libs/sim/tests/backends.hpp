#pragma once

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "drape/sim/solver_factory.hpp"

namespace drape::sim::testing {

// Every backend this build and machine can run: cpu everywhere, gpu-emulated everywhere, metal on Macs with a GPU.
inline std::vector<SolverBackend> testBackends() {
  std::vector<SolverBackend> out;
  for (auto b : {SolverBackend::Cpu, SolverBackend::GpuEmulated, SolverBackend::Metal})
    if (isAvailable(b)) out.push_back(b);
  return out;
}

// The available backends other than the CPU reference.
inline std::vector<SolverBackend> gpuBackends() {
  std::vector<SolverBackend> out;
  for (auto b : testBackends())
    if (b != SolverBackend::Cpu) out.push_back(b);
  return out;
}

// gtest parameter names: "cpu", "gpu_emulated", "metal".
inline std::string backendTestName(const ::testing::TestParamInfo<SolverBackend>& info) {
  std::string name(backendName(info.param));
  for (auto& c : name)
    if (c == '-') c = '_';
  return name;
}

class BackendTest : public ::testing::TestWithParam<SolverBackend> {
 protected:
  std::unique_ptr<Solver> make() const { return makeSolver(GetParam()); }
};

}  // namespace drape::sim::testing

// Runs the TEST_P tests of `suite` (a BackendTest alias) on every available backend.
#define DRAPE_INSTANTIATE_BACKENDS(suite)                                                                   \
  INSTANTIATE_TEST_SUITE_P(Backends, suite, ::testing::ValuesIn(::drape::sim::testing::testBackends()), \
                           ::drape::sim::testing::backendTestName)
