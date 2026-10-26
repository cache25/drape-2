#include "drape/sim/drape_session.hpp"

#include <string>
#include <utility>

namespace drape::sim {

namespace {
constexpr double kTimeEps = 1e-9;  // keeps accumulated 1/60 steps from shifting a transition by a frame
}

DrapeSession::DrapeSession(Solver& solver, std::vector<GarmentId> garments, DrapeOptions options)
    : solver_(solver), garments_(std::move(garments)), o_(options), substeps_(options.substeps) {
  setAll(SimPhase::Assembling);
  saved_ = {solver_.snapshot(), 0, phase_, 0};
}

bool DrapeSession::finished() const {
  return state_ == DrapeState::Settled || state_ == DrapeState::Failed || state_ == DrapeState::TimedOut;
}

void DrapeSession::setAll(SimPhase p) {
  for (const auto& g : garments_) solver_.setPhase(g, p);
}

void DrapeSession::resetCounters() {
  stillFrames_ = 0;
  riseFrames_ = 0;
  lastEnergy_ = std::numeric_limits<double>::infinity();
}

DrapeState DrapeSession::advance() {
  if (finished()) return state_;
  if (time_ >= o_.maxSimSeconds - kTimeEps) {
    result_.simulatedSeconds = time_;
    state_ = DrapeState::TimedOut;
    return state_;
  }
  solver_.step({o_.frameDt, substeps_, o_.iterations});
  ++result_.framesRun;
  ++keptFrames_;
  time_ += o_.frameDt;
  phaseTime_ += o_.frameDt;
  const SimStats st = solver_.stats();

  riseFrames_ = st.kineticEnergy > lastEnergy_ ? riseFrames_ + 1 : 0;
  lastEnergy_ = st.kineticEnergy;
  if (st.hasNonFinite || riseFrames_ >= o_.energyRiseFrames) {
    ++result_.retries;
    if (result_.retries > o_.maxRetries) {
      result_.retries = o_.maxRetries;
      result_.failed = true;
      result_.message = std::string(kDrapeFailureMessage);
      result_.simulatedSeconds = time_;
      state_ = DrapeState::Failed;
      return state_;
    }
    solver_.restore(saved_.snapshot);
    time_ = saved_.time;
    phase_ = saved_.phase;
    phaseTime_ = saved_.phaseTime;
    keptFrames_ = static_cast<int>(saved_.time / o_.frameDt + 0.5);
    substeps_ *= 2;
    resetCounters();
    state_ = phase_ == SimPhase::Assembling ? DrapeState::Assembling : DrapeState::Settling;
    return state_;
  }

  if (phase_ == SimPhase::Assembling) {
    if (st.maxSeamGap < o_.seamGapDone || phaseTime_ >= o_.maxAssemblySeconds - kTimeEps) {
      result_.assemblyEndSeamGap = st.maxSeamGap;
      result_.assemblySeconds = time_;
      phase_ = SimPhase::Settling;
      setAll(phase_);
      phaseTime_ = 0;
      resetCounters();
    }
  } else {
    stillFrames_ = st.rmsSpeed < o_.settleRmsSpeed ? stillFrames_ + 1 : 0;
    if (stillFrames_ >= o_.settleHoldFrames) {
      result_.settled = true;
      result_.simulatedSeconds = time_;
      state_ = DrapeState::Settled;
      return state_;
    }
  }
  if (keptFrames_ % o_.snapshotEvery == 0) saved_ = {solver_.snapshot(), time_, phase_, phaseTime_};
  if (time_ >= o_.maxSimSeconds - kTimeEps) {
    result_.simulatedSeconds = time_;
    state_ = DrapeState::TimedOut;
    return state_;
  }
  state_ = phase_ == SimPhase::Assembling ? DrapeState::Assembling : DrapeState::Settling;
  return state_;
}

}  // namespace drape::sim
