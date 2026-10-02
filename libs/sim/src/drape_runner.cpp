#include "drape/sim/drape_runner.hpp"

#include <limits>

namespace drape::sim {

namespace {
constexpr double kTimeEps = 1e-9;  // keeps accumulated 1/60 steps from shifting a transition by a frame
}

DrapeResult drapeToRest(Solver& solver, const std::vector<GarmentId>& garments, const DrapeOptions& o) {
  struct Saved {
    Snapshot snapshot;
    double time = 0;
    SimPhase phase = SimPhase::Assembling;
    double phaseTime = 0;
  };
  auto setAll = [&](SimPhase p) {
    for (const auto& g : garments) solver.setPhase(g, p);
  };

  DrapeResult result;
  setAll(SimPhase::Assembling);
  SimPhase phase = SimPhase::Assembling;
  double time = 0, phaseTime = 0;
  int substeps = o.substeps;
  int stillFrames = 0, riseFrames = 0, keptFrames = 0;
  double lastEnergy = std::numeric_limits<double>::infinity();
  Saved saved{solver.snapshot(), 0, phase, 0};

  auto resetCounters = [&] {
    stillFrames = 0;
    riseFrames = 0;
    lastEnergy = std::numeric_limits<double>::infinity();
  };

  while (true) {
    if (time >= o.maxSimSeconds - kTimeEps) break;
    solver.step({o.frameDt, substeps, o.iterations});
    ++result.framesRun;
    ++keptFrames;
    time += o.frameDt;
    phaseTime += o.frameDt;
    const SimStats st = solver.stats();

    riseFrames = st.kineticEnergy > lastEnergy ? riseFrames + 1 : 0;
    lastEnergy = st.kineticEnergy;
    if (st.hasNonFinite || riseFrames >= o.energyRiseFrames) {
      ++result.retries;
      if (result.retries > o.maxRetries) {
        result.retries = o.maxRetries;
        result.failed = true;
        result.message = std::string(kDrapeFailureMessage);
        result.simulatedSeconds = time;
        return result;
      }
      solver.restore(saved.snapshot);
      time = saved.time;
      phase = saved.phase;
      phaseTime = saved.phaseTime;
      keptFrames = static_cast<int>(saved.time / o.frameDt + 0.5);
      substeps *= 2;
      resetCounters();
      continue;
    }

    if (phase == SimPhase::Assembling) {
      if (st.maxSeamGap < o.seamGapDone || phaseTime >= o.maxAssemblySeconds - kTimeEps) {
        result.assemblyEndSeamGap = st.maxSeamGap;
        phase = SimPhase::Settling;
        setAll(phase);
        phaseTime = 0;
        resetCounters();
      }
    } else {
      stillFrames = st.rmsSpeed < o.settleRmsSpeed ? stillFrames + 1 : 0;
      if (stillFrames >= o.settleHoldFrames) {
        result.settled = true;
        result.simulatedSeconds = time;
        return result;
      }
    }
    if (keptFrames % o.snapshotEvery == 0) saved = {solver.snapshot(), time, phase, phaseTime};
  }
  result.simulatedSeconds = time;
  return result;
}

}  // namespace drape::sim
