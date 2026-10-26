#pragma once

#include <limits>
#include <vector>

#include "drape/sim/drape_runner.hpp"
#include "drape/sim/solver.hpp"

namespace drape::sim {

enum class DrapeState { Assembling, Settling, Settled, Failed, TimedOut };

// The drape runner (spec 5.7, 5.8) one frame at a time: Assembling -> Settling -> Settled, with snapshots, the
// stability guard and its retries. drapeToRest() is a loop over advance(). The solver must outlive the session.
class DrapeSession {
 public:
  DrapeSession(Solver& solver, std::vector<GarmentId> garments, DrapeOptions options = {});  // sets Assembling

  DrapeState advance();  // runs one frame and returns the new state; a no-op once finished()
  DrapeState state() const { return state_; }
  bool finished() const;  // Settled, Failed or TimedOut
  const DrapeResult& result() const { return result_; }
  double simulatedSeconds() const { return time_; }

 private:
  struct Saved {
    Snapshot snapshot;
    double time = 0;
    SimPhase phase = SimPhase::Assembling;
    double phaseTime = 0;
  };

  void setAll(SimPhase p);
  void resetCounters();

  Solver& solver_;
  std::vector<GarmentId> garments_;
  DrapeOptions o_;
  DrapeResult result_;
  DrapeState state_ = DrapeState::Assembling;
  SimPhase phase_ = SimPhase::Assembling;
  double time_ = 0, phaseTime_ = 0;
  int substeps_ = 1;
  int stillFrames_ = 0, riseFrames_ = 0, keptFrames_ = 0;
  double lastEnergy_ = std::numeric_limits<double>::infinity();
  Saved saved_;
};

}  // namespace drape::sim
