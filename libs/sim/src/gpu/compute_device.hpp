#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace drape::sim {

// One entry per kernel in kernels.h (k_<name>); kernelName() is the Metal function name.
enum class Kernel : std::uint32_t {
  Predict, ContactSetup, ContactActivate, InertiaFriction, StretchTerms, HingeTerms, SeamTerms, GatherGrad,
  GatherBlocks, ContactRelease, Precondition, CgInit, Spmv, CgAlpha, CgUpdate, CgCheck, CgPrecond, CgBeta,
  CgDirection, ReducePartial, ReduceFinal, ContactForce, TrialPositions, ApplyStep, FinishSubstep, Count
};

std::string_view kernelName(Kernel k);  // "predict", "contact_setup", ...

struct BufferId {
  std::uint32_t index = 0;
};

// A GPU (or its CPU emulation) as the solver sees it: shared CPU-visible buffers and an ordered list of kernel
// dispatches per submission. Dispatches in one submission run in order, each seeing the previous ones' writes.
class ComputeDevice {
 public:
  virtual ~ComputeDevice() = default;
  virtual BufferId create(std::size_t bytes) = 0;  // zero-filled, CPU-visible (shared storage)
  virtual void* contents(BufferId b) = 0;
  virtual void begin() = 0;  // start recording a submission
  virtual void dispatch(Kernel k, std::uint32_t threads, std::span<const BufferId> buffers) = 0;
  virtual void submitAndWait() = 0;  // runs the recorded dispatches; throws std::runtime_error on device errors
};

}  // namespace drape::sim
