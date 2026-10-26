#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "drape/sim/solver_factory.hpp"

using namespace drape::sim;

TEST(SolverFactory, CpuAlwaysAvailable) {
  EXPECT_TRUE(isAvailable(SolverBackend::Cpu));
  const auto s = makeSolver(SolverBackend::Cpu);
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->particleCount(), 0u);
}

TEST(SolverFactory, BackendNames) {
  EXPECT_EQ(backendName(SolverBackend::Cpu), "cpu");
  EXPECT_EQ(backendName(SolverBackend::GpuEmulated), "gpu-emulated");
  EXPECT_EQ(backendName(SolverBackend::Metal), "metal");
}

#if !defined(__APPLE__)
TEST(SolverFactory, UnavailableThrowsNamingBackend) {
  EXPECT_FALSE(isAvailable(SolverBackend::Metal));
  try {
    makeSolver(SolverBackend::Metal);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("metal"), std::string::npos) << e.what();
  }
}
#endif
