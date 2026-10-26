#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <limits>

#include "drape/core/test_fabrics.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "drape/sim/drape_runner.hpp"
#include "backends.hpp"
#include "scripted_solver.hpp"
#include "sim_test_utils.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;


TEST(Runner, SettlesAfterStillFrames) {
  ScriptedSolver s;
  s.script = [](int, int) { return calm(); };
  const auto r = drapeToRest(s, {"g"});
  EXPECT_TRUE(r.settled);
  EXPECT_FALSE(r.failed);
  EXPECT_EQ(r.framesRun, 61);
  EXPECT_EQ(s.phases.at(1), SimPhase::Assembling);
  EXPECT_EQ(s.phases.at(2), SimPhase::Settling);
}

TEST(Runner, AssemblyEndsAtTimeout) {
  ScriptedSolver s;
  s.script = [](int, int) { return calm(0.01); };
  const auto r = drapeToRest(s, {"g"});
  EXPECT_TRUE(r.settled);
  EXPECT_DOUBLE_EQ(r.assemblyEndSeamGap, 0.01);
  EXPECT_EQ(r.framesRun, 180 + 60);
}

TEST(Runner, ReportsWhenAssemblyEnded) {
  ScriptedSolver s;
  s.script = [](int frame, int) { return calm(frame < 30 ? 0.01 : 0.0); };  // the seams close on frame 30
  const auto r = drapeToRest(s, {"g"});
  EXPECT_TRUE(r.settled);
  EXPECT_NEAR(r.assemblySeconds, 30.0 / 60.0, 1e-12);
  EXPECT_NEAR(r.simulatedSeconds, 90.0 / 60.0, 1e-12);

  ScriptedSolver slow;
  slow.script = [](int, int) { return calm(0.01); };  // never closes: assembly times out at 3 s
  EXPECT_NEAR(drapeToRest(slow, {"g"}).assemblySeconds, 3.0, 1e-9);
}

TEST(Runner, RetriesOnNonFinite) {
  ScriptedSolver s;
  s.script = [](int frame, int restores) {
    SimStats st = calm();
    if (frame == 40 && restores == 0) st.hasNonFinite = true;
    return st;
  };
  DrapeOptions o;
  o.substeps = 2;
  const auto r = drapeToRest(s, {"g"}, o);
  EXPECT_TRUE(r.settled);
  EXPECT_EQ(r.retries, 1);
  ASSERT_EQ(s.restoredFrames.size(), 1u);
  EXPECT_EQ(s.restoredFrames[0], 30);
  EXPECT_EQ(s.stepSubsteps.back(), 4);
}

TEST(Runner, FailsAfterThreeRetries) {
  ScriptedSolver s;
  s.script = [](int frame, int) {
    SimStats st = calm(0.0, 1.0);
    if (frame >= 10) st.hasNonFinite = true;
    return st;
  };
  const auto r = drapeToRest(s, {"g"});
  EXPECT_TRUE(r.failed);
  EXPECT_FALSE(r.settled);
  EXPECT_EQ(r.retries, 3);
  EXPECT_EQ(r.message, kDrapeFailureMessage);
}

TEST(Runner, RisingEnergyIsFailure) {
  ScriptedSolver s;
  s.script = [](int frame, int restores) {
    SimStats st = calm(0.0, 1.0);
    st.kineticEnergy = restores == 0 ? static_cast<double>(frame) : 1.0;
    if (restores > 0) st.rmsSpeed = 0.0;
    return st;
  };
  const auto r = drapeToRest(s, {"g"});
  EXPECT_EQ(r.retries, 1);
  EXPECT_TRUE(r.settled);
}

TEST(Runner, StopsAtMaxSimSeconds) {
  ScriptedSolver s;
  s.script = [](int, int) { return calm(0.0, 1.0); };
  const auto r = drapeToRest(s, {"g"});
  EXPECT_FALSE(r.settled);
  EXPECT_FALSE(r.failed);
  EXPECT_NEAR(r.simulatedSeconds, 20.0, 1.0 / 60.0);
}

using PhysicsStability = BackendTest;
DRAPE_INSTANTIATE_BACKENDS(PhysicsStability);

TEST_P(PhysicsStability, HangingSheetSettlesSymmetric) {
  const HangingSheet hs = hangingSheet();
  const SimScene& scene = hs.scene;
  const auto bl = hs.bottomLeft, br = hs.bottomRight;
  const auto sp = make();
  Solver& s = *sp;
  s.build(scene);
  DrapeOptions o;
  o.maxSimSeconds = 15.0;
  const auto r = drapeToRest(s, {"sheet"}, o);
  ASSERT_TRUE(r.settled) << r.message;
  const auto x = positions(s);
  double comX = 0;
  for (const auto& p : x) comX += p.x();
  comX /= static_cast<double>(x.size());
  EXPECT_NEAR(comX, 0.25, 0.002);
  EXPECT_NEAR(x[bl].y(), x[br].y(), 0.002);
}
