#include "gpu/emulated_device.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>

#include "kernels.h"

namespace drape::sim {

using namespace dgpu;

namespace {

std::size_t arity(Kernel k) {
  switch (k) {
    case Kernel::Predict: return 9;
    case Kernel::ContactSetup: return 7;
    case Kernel::ContactActivate: return 3;
    case Kernel::InertiaFriction: return 9;
    case Kernel::StretchTerms: return 8;
    case Kernel::HingeTerms: return 7;
    case Kernel::SeamTerms: return 8;
    case Kernel::GatherGrad: return 6;
    case Kernel::GatherBlocks: return 5;
    case Kernel::ContactRelease: return 3;
    case Kernel::Precondition: return 5;
    case Kernel::CgInit: return 11;
    case Kernel::Spmv: return 11;
    case Kernel::CgAlpha: return 1;
    case Kernel::CgUpdate: return 9;
    case Kernel::CgCheck: return 2;
    case Kernel::CgPrecond: return 7;
    case Kernel::CgBeta: return 1;
    case Kernel::CgDirection: return 4;
    case Kernel::ReducePartial: return 3;
    case Kernel::ReduceFinal: return 3;
    case Kernel::ContactForce: return 3;
    case Kernel::TrialPositions: return 4;
    case Kernel::ApplyStep: return 5;
    case Kernel::FinishSubstep: return 11;
    case Kernel::Count: break;
  }
  return 0;
}

}  // namespace

BufferId EmulatedDevice::create(std::size_t bytes) {
  const std::size_t words = (bytes + sizeof(std::max_align_t) - 1) / sizeof(std::max_align_t);
  buffers_.emplace_back(std::max<std::size_t>(words, 1), std::max_align_t{});
  for (auto& w : buffers_.back()) std::memset(&w, 0, sizeof(w));
  return BufferId{static_cast<std::uint32_t>(buffers_.size() - 1)};
}

void* EmulatedDevice::contents(BufferId b) {
  if (b.index >= buffers_.size()) throw std::out_of_range("emulated device: unknown buffer");
  return buffers_[b.index].data();
}

void EmulatedDevice::begin() { recorded_.clear(); }

void EmulatedDevice::dispatch(Kernel k, std::uint32_t threads, std::span<const BufferId> buffers) {
  if (buffers.size() != arity(k)) {
    throw std::invalid_argument("emulated device: " + std::string(kernelName(k)) + " takes " +
                                std::to_string(arity(k)) + " buffers, got " + std::to_string(buffers.size()));
  }
  recorded_.push_back({k, threads, std::vector<BufferId>(buffers.begin(), buffers.end())});
}

void EmulatedDevice::submitAndWait() {
  for (const auto& c : recorded_) run(c);
  recorded_.clear();
}

void EmulatedDevice::run(const Call& c) {
  auto B = [&](std::size_t i) { return contents(c.buffers[i]); };
#define DRAPE_ARG(T, i) static_cast<T*>(B(i))
  for (std::uint32_t tid = 0; tid < c.threads; ++tid) {
    switch (c.kernel) {
      case Kernel::Predict:
        k_predict(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuParticle, 1), DRAPE_ARG(const GpuGarment, 2),
                  DRAPE_ARG(const float4, 3), DRAPE_ARG(const float4, 4), DRAPE_ARG(const float4, 5),
                  DRAPE_ARG(float4, 6), DRAPE_ARG(float4, 7), DRAPE_ARG(float4, 8));
        break;
      case Kernel::ContactSetup:
        k_contact_setup(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuParticle, 1),
                        DRAPE_ARG(const GpuFabric, 2), DRAPE_ARG(const float4, 3), DRAPE_ARG(const float, 4),
                        DRAPE_ARG(const FieldInfo, 5), DRAPE_ARG(GpuContact, 6));
        break;
      case Kernel::ContactActivate:
        k_contact_activate(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(GpuContact, 1), DRAPE_ARG(float4, 2));
        break;
      case Kernel::InertiaFriction:
        k_inertia_friction(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuParticle, 1),
                           DRAPE_ARG(const GpuFabric, 2), DRAPE_ARG(const float4, 3), DRAPE_ARG(const float4, 4),
                           DRAPE_ARG(const GpuContact, 5), DRAPE_ARG(float, 6), DRAPE_ARG(float4, 7),
                           DRAPE_ARG(Block3, 8));
        break;
      case Kernel::StretchTerms:
        k_stretch_terms(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuStretch, 1),
                        DRAPE_ARG(const GpuFabric, 2), DRAPE_ARG(const float4, 3), DRAPE_ARG(const float4, 4),
                        DRAPE_ARG(float, 5), DRAPE_ARG(float4, 6), DRAPE_ARG(Block3, 7));
        break;
      case Kernel::HingeTerms:
        k_hinge_terms(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuHinge, 1), DRAPE_ARG(const float4, 2),
                      DRAPE_ARG(const float4, 3), DRAPE_ARG(float, 4), DRAPE_ARG(float4, 5), DRAPE_ARG(Block3, 6));
        break;
      case Kernel::SeamTerms:
        k_seam_terms(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuSeam, 1), DRAPE_ARG(const GpuGarment, 2),
                     DRAPE_ARG(const float4, 3), DRAPE_ARG(const float4, 4), DRAPE_ARG(float, 5),
                     DRAPE_ARG(float4, 6), DRAPE_ARG(Block3, 7));
        break;
      case Kernel::GatherGrad:
        k_gather_grad(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuParticle, 1), DRAPE_ARG(const uint, 2),
                      DRAPE_ARG(const uint, 3), DRAPE_ARG(const float4, 4), DRAPE_ARG(float4, 5));
        break;
      case Kernel::GatherBlocks:
        k_gather_blocks(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const uint, 1), DRAPE_ARG(const uint, 2),
                        DRAPE_ARG(const Block3, 3), DRAPE_ARG(Block3, 4));
        break;
      case Kernel::ContactRelease:
        k_contact_release(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(GpuContact, 1), DRAPE_ARG(const float4, 2));
        break;
      case Kernel::Precondition:
        k_precondition(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuParticle, 1), DRAPE_ARG(const uint, 2),
                       DRAPE_ARG(const Block3, 3), DRAPE_ARG(Block3, 4));
        break;
      case Kernel::CgInit:
        k_cg_init(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuContact, 1), DRAPE_ARG(const Block3, 2),
                  DRAPE_ARG(const float4, 3), DRAPE_ARG(float4, 4), DRAPE_ARG(float4, 5), DRAPE_ARG(float4, 6),
                  DRAPE_ARG(float4, 7), DRAPE_ARG(float4, 8), DRAPE_ARG(float, 9), DRAPE_ARG(float, 10));
        break;
      case Kernel::Spmv:
        k_spmv(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuParticle, 1), DRAPE_ARG(const uint, 2),
               DRAPE_ARG(const uint, 3), DRAPE_ARG(const Block3, 4), DRAPE_ARG(const GpuContact, 5),
               DRAPE_ARG(const CgScalars, 6), DRAPE_ARG(const float4, 7), DRAPE_ARG(float4, 8), DRAPE_ARG(float4, 9),
               DRAPE_ARG(float, 10));
        break;
      case Kernel::CgAlpha: k_cg_alpha(tid, DRAPE_ARG(CgScalars, 0)); break;
      case Kernel::CgUpdate:
        k_cg_update(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const CgScalars, 1), DRAPE_ARG(const float4, 2),
                    DRAPE_ARG(const float4, 3), DRAPE_ARG(const float4, 4), DRAPE_ARG(float4, 5), DRAPE_ARG(float4, 6),
                    DRAPE_ARG(float4, 7), DRAPE_ARG(float, 8));
        break;
      case Kernel::CgCheck: k_cg_check(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(CgScalars, 1)); break;
      case Kernel::CgPrecond:
        k_cg_precond(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuContact, 1),
                     DRAPE_ARG(const CgScalars, 2), DRAPE_ARG(const Block3, 3), DRAPE_ARG(const float4, 4),
                     DRAPE_ARG(float4, 5), DRAPE_ARG(float, 6));
        break;
      case Kernel::CgBeta: k_cg_beta(tid, DRAPE_ARG(CgScalars, 0)); break;
      case Kernel::CgDirection:
        k_cg_direction(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const CgScalars, 1), DRAPE_ARG(const float4, 2),
                       DRAPE_ARG(float4, 3));
        break;
      case Kernel::ReducePartial:
        k_reduce_partial(tid, DRAPE_ARG(const ReduceSpec, 0), DRAPE_ARG(const float, 1), DRAPE_ARG(float, 2));
        break;
      case Kernel::ReduceFinal:
        k_reduce_final(tid, DRAPE_ARG(const ReduceSpec, 0), DRAPE_ARG(const float, 1), DRAPE_ARG(CgScalars, 2));
        break;
      case Kernel::ContactForce:
        k_contact_force(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(GpuContact, 1), DRAPE_ARG(const float4, 2));
        break;
      case Kernel::TrialPositions:
        k_trial_positions(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const float4, 1), DRAPE_ARG(const float4, 2),
                          DRAPE_ARG(float4, 3));
        break;
      case Kernel::ApplyStep:
        k_apply_step(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(GpuContact, 1), DRAPE_ARG(float4, 2),
                     DRAPE_ARG(const float4, 3), DRAPE_ARG(float, 4));
        break;
      case Kernel::FinishSubstep:
        k_finish_substep(tid, DRAPE_ARG(const StepParams, 0), DRAPE_ARG(const GpuParticle, 1),
                         DRAPE_ARG(const GpuFabric, 2), DRAPE_ARG(const GpuGarment, 3), DRAPE_ARG(const float4, 4),
                         DRAPE_ARG(const float4, 5), DRAPE_ARG(const float4, 6), DRAPE_ARG(const float, 7),
                         DRAPE_ARG(const FieldInfo, 8), DRAPE_ARG(float4, 9), DRAPE_ARG(float4, 10));
        break;
      case Kernel::Count: throw std::invalid_argument("emulated device: Kernel::Count is not a kernel");
    }
  }
#undef DRAPE_ARG
}

}  // namespace drape::sim
