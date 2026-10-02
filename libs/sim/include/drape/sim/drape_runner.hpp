#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "drape/sim/solver.hpp"

namespace drape::sim {

inline constexpr std::string_view kDrapeFailureMessage =
    "Drape couldn't settle this garment. Try Draft quality or a different fabric.";

struct DrapeOptions {
  int substeps = 1;    // Standard quality
  int iterations = 2;  // Newton iterations per substep
  double frameDt = 1.0 / 60.0;
  double maxAssemblySeconds = 3.0;
  double seamGapDone = 0.001;
  double settleRmsSpeed = 0.002;
  int settleHoldFrames = 60;
  double maxSimSeconds = 20.0;
  int snapshotEvery = 30;
  int maxRetries = 3;
  int energyRiseFrames = 60;
};

struct DrapeResult {
  bool settled = false;
  bool failed = false;
  std::string message;
  double simulatedSeconds = 0;
  int retries = 0;
  int framesRun = 0;  // every step() call, including frames later rolled back
  double assemblyEndSeamGap = 0;
};

// Runs Assembling -> Settling -> Settled with the stability guard (spec 5.7, 5.8).
DrapeResult drapeToRest(Solver& solver, const std::vector<GarmentId>& garments, const DrapeOptions& options = {});

}  // namespace drape::sim
