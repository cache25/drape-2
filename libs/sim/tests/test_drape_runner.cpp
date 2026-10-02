#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <limits>

#include "drape/core/test_fabrics.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "drape/sim/drape_runner.hpp"
#include "sim_test_utils.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

namespace {

// Solver double whose stats come from a script indexed by the solver's current frame. Snapshots carry the
// frame index in positions[0].x(), so restore() rewinds the frame counter.
class ScriptedSolver final : public Solver {
 public:
  std::function<SimStats(int frame, int restores)> script;
  int frame = 0;
  int restores = 0;
  std::vector<int> stepSubsteps;
  std::vector<int> restoredFrames;
  std::vector<SimPhase> phases{SimPhase::Settling};

  void build(const SimScene&) override {}
  void updateFabric(const GarmentId&, const std::vector<FabricPhysical>&) override {}
  void setPhase(const GarmentId&, SimPhase p) override { phases.push_back(p); }
  void step(const StepInput& in) override {
    ++frame;
    stepSubsteps.push_back(in.substeps);
  }
  void readPositions(std::span<Vec3f>) const override {}
  SimStats stats() const override { return script(frame, restores); }
  Snapshot snapshot() const override {
    Snapshot s;
    s.positions = {Vec3f(static_cast<float>(frame), 0, 0)};
    return s;
  }
  void restore(const Snapshot& s) override {
    frame = static_cast<int>(s.positions[0].x());
    restoredFrames.push_back(frame);
    ++restores;
  }
  ParticleRange range(const GarmentId&) const override { return {0, 1}; }
  std::size_t particleCount() const override { return 1; }
};

SimStats calm(double gap = 0.0, double rms = 0.0) {
  SimStats s;
  s.maxSeamGap = gap;
  s.rmsSpeed = rms;
  return s;
}

}  // namespace

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

TEST(Runner, HangingSheetSettlesSymmetric) {
  SimScene scene;
  SimGarment g;
  {
    Pattern p;
    p.pieces.push_back(garment::rectanglePiece("sheet", 0.5, 0.5));
    g.id = "sheet";
    g.mesh = garment::buildSimMesh(p, 0.01);
    for (const auto& r : g.mesh.rest) g.initialPositions.push_back(Vec3d(r.x(), 0.5 + r.y(), 0));
    g.pieceFabric = {testdata::testJersey()};
  }
  const SimMesh& m = g.mesh;
  std::uint32_t tl = 0, tr = 0, bl = 0, br = 0;
  for (std::uint32_t v = 0; v < m.rest.size(); ++v) {
    const Vec2d r = m.rest[v];
    if ((r - Vec2d(0, 0.5)).norm() < 1e-9) tl = v;
    if ((r - Vec2d(0.5, 0.5)).norm() < 1e-9) tr = v;
    if ((r - Vec2d(0, 0)).norm() < 1e-9) bl = v;
    if ((r - Vec2d(0.5, 0)).norm() < 1e-9) br = v;
  }
  scene.pins.push_back({tl, g.initialPositions[tl]});
  scene.pins.push_back({tr, g.initialPositions[tr]});
  scene.garments.push_back(g);
  CpuSolver s;
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
