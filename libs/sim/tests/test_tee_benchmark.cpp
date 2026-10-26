#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <map>
#include <string>

#include "drape/session/tee_scene.hpp"
#include "drape/sim/drape_runner.hpp"
#include "drape/sim/solver_factory.hpp"

using namespace drape;
using namespace drape::sim;

// Phase B spec 7.3: the tee draped to rest on the male, on the CPU and on Metal, at every quality. Prints one bench
// line per run (recorded in the Phase B spec's as-built notes) and gates Metal faster than CPU at Standard.
TEST(TeeBenchmark, GpuFasterThanCpuAtStandard) {
  ASSERT_TRUE(isAvailable(SolverBackend::Metal));
  std::map<std::pair<int, int>, double> wall;
  const char* qualityName[] = {"draft", "standard", "fine"};
  for (SimQuality q : {SimQuality::Draft, SimQuality::Standard, SimQuality::Fine}) {
    const auto ts = session::makeTeeScene(avatar::BodyType::Male, q);
    for (SolverBackend b : {SolverBackend::Cpu, SolverBackend::Metal}) {
      const auto solver = makeSolver(b);
      solver->build(ts->scene);
      DrapeOptions o;
      o.substeps = substepsFor(q);
      o.iterations = iterationsFor(q);
      const auto start = std::chrono::steady_clock::now();
      const DrapeResult r = drapeToRest(*solver, {"tee"}, o);
      const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      wall[{static_cast<int>(q), static_cast<int>(b)}] = seconds;
      std::printf("bench backend=%s quality=%s wall_s=%.2f sim_s=%.2f frames=%d particles=%zu settled=%d\n",
                  std::string(backendName(b)).c_str(), qualityName[static_cast<int>(q)], seconds, r.simulatedSeconds,
                  r.framesRun, solver->particleCount(), r.settled ? 1 : 0);
      std::fflush(stdout);
    }
  }
  const int standard = static_cast<int>(SimQuality::Standard);
  EXPECT_LT(wall[{standard, static_cast<int>(SolverBackend::Metal)}], wall[{standard, static_cast<int>(SolverBackend::Cpu)}]);
}
