#pragma once

#include <functional>
#include <span>
#include <vector>

#include "drape/sim/solver.hpp"

namespace drape::sim::testing {

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

inline SimStats calm(double gap = 0.0, double rms = 0.0) {
  SimStats s;
  s.maxSeamGap = gap;
  s.rmsSpeed = rms;
  return s;
}


}  // namespace drape::sim::testing
