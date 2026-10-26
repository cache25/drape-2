#include <gtest/gtest.h>

#include "drape/sim/cpu_solver.hpp"
#include "drape/sim/drape_runner.hpp"
#include "drape/sim/drape_session.hpp"
#include "scripted_solver.hpp"
#include "sim_test_utils.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

TEST(DrapeSession, StartsAssembling) {
  ScriptedSolver s;
  s.script = [](int, int) { return calm(); };
  DrapeSession session(s, {"g"});
  EXPECT_EQ(session.state(), DrapeState::Assembling);
  EXPECT_EQ(session.result().framesRun, 0);
  EXPECT_FALSE(session.finished());
  ASSERT_EQ(s.phases.size(), 2u);
  EXPECT_EQ(s.phases.back(), SimPhase::Assembling);
}

TEST(DrapeSession, SequenceToSettled) {
  ScriptedSolver s;
  s.script = [](int, int) { return calm(); };  // seams closed and still from frame 1
  DrapeSession session(s, {"g"});
  EXPECT_EQ(session.advance(), DrapeState::Settling);
  for (int f = 2; f <= 60; ++f) EXPECT_EQ(session.advance(), DrapeState::Settling) << "frame " << f;
  EXPECT_EQ(session.advance(), DrapeState::Settled);
  EXPECT_TRUE(session.finished());
  EXPECT_TRUE(session.result().settled);
  EXPECT_EQ(session.result().framesRun, 61);
}

TEST(DrapeSession, AdvanceAfterFinishIsNoOp) {
  ScriptedSolver s;
  s.script = [](int, int) { return calm(); };
  DrapeSession session(s, {"g"});
  while (!session.finished()) session.advance();
  const int frames = session.result().framesRun;
  EXPECT_EQ(session.advance(), DrapeState::Settled);
  EXPECT_EQ(session.result().framesRun, frames);
  EXPECT_EQ(s.frame, frames);
}

TEST(DrapeSession, FailureMessage) {
  ScriptedSolver s;
  s.script = [](int, int) {
    SimStats st = calm(0.0, 1.0);
    st.hasNonFinite = true;
    return st;
  };
  DrapeSession session(s, {"g"});
  while (!session.finished()) session.advance();
  EXPECT_EQ(session.state(), DrapeState::Failed);
  EXPECT_TRUE(session.result().failed);
  EXPECT_EQ(session.result().message, kDrapeFailureMessage);
  EXPECT_EQ(session.result().retries, 3);
}

TEST(DrapeSession, TimedOut) {
  ScriptedSolver s;
  s.script = [](int, int) { return calm(0.0, 1.0); };  // never still
  DrapeOptions o;
  o.maxSimSeconds = 1.0;
  DrapeSession session(s, {"g"}, o);
  while (!session.finished()) session.advance();
  EXPECT_EQ(session.state(), DrapeState::TimedOut);
  EXPECT_FALSE(session.result().settled);
  EXPECT_FALSE(session.result().failed);
  EXPECT_EQ(session.result().framesRun, 60);
  EXPECT_NEAR(session.simulatedSeconds(), 1.0, 1e-9);
}

TEST(DrapeSession, MatchesDrapeToRestOnHangingSheet) {
  const HangingSheet hs = hangingSheet();
  DrapeOptions o;
  o.maxSimSeconds = 15.0;
  CpuSolver a, b;
  a.build(hs.scene);
  b.build(hs.scene);
  const DrapeResult ra = drapeToRest(a, {"sheet"}, o);
  DrapeSession session(b, {"sheet"}, o);
  while (!session.finished()) session.advance();
  const DrapeResult& rb = session.result();
  EXPECT_EQ(ra.settled, rb.settled);
  EXPECT_EQ(ra.failed, rb.failed);
  EXPECT_EQ(ra.framesRun, rb.framesRun);
  EXPECT_EQ(ra.retries, rb.retries);
  EXPECT_EQ(ra.simulatedSeconds, rb.simulatedSeconds);
  EXPECT_EQ(ra.assemblySeconds, rb.assemblySeconds);
  EXPECT_EQ(ra.assemblyEndSeamGap, rb.assemblyEndSeamGap);
  EXPECT_EQ(positions(a), positions(b));
}
