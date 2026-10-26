#include "gpu/compute_device.hpp"

namespace drape::sim {

std::string_view kernelName(Kernel k) {
  switch (k) {
    case Kernel::Predict: return "predict";
    case Kernel::ContactSetup: return "contact_setup";
    case Kernel::ContactActivate: return "contact_activate";
    case Kernel::InertiaFriction: return "inertia_friction";
    case Kernel::StretchTerms: return "stretch_terms";
    case Kernel::HingeTerms: return "hinge_terms";
    case Kernel::SeamTerms: return "seam_terms";
    case Kernel::GatherGrad: return "gather_grad";
    case Kernel::GatherBlocks: return "gather_blocks";
    case Kernel::ContactRelease: return "contact_release";
    case Kernel::Precondition: return "precondition";
    case Kernel::CgInit: return "cg_init";
    case Kernel::Spmv: return "spmv";
    case Kernel::CgAlpha: return "cg_alpha";
    case Kernel::CgUpdate: return "cg_update";
    case Kernel::CgCheck: return "cg_check";
    case Kernel::CgPrecond: return "cg_precond";
    case Kernel::CgBeta: return "cg_beta";
    case Kernel::CgDirection: return "cg_direction";
    case Kernel::ReducePartial: return "reduce_partial";
    case Kernel::ReduceFinal: return "reduce_final";
    case Kernel::ContactForce: return "contact_force";
    case Kernel::TrialPositions: return "trial_positions";
    case Kernel::ApplyStep: return "apply_step";
    case Kernel::FinishSubstep: return "finish_substep";
    case Kernel::Count: break;
  }
  return "unknown";
}

}  // namespace drape::sim
