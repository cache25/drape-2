// Metal entry points for the solver kernels: one [[kernel]] per Kernel (function name = kernelName()), each calling
// the shared body in kernels.h with its buffers in dispatch order. Must match EmulatedDevice::run (same bodies, same
// buffer order). At build time this file and the headers it needs are combined into one source, which MetalDevice
// compiles at runtime (no Metal toolchain needed).
#include "kernels.h"

using namespace dgpu;

kernel void predict(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuParticle* b1 [[buffer(1)]],
    device const GpuGarment* b2 [[buffer(2)]],
    device const float4* b3 [[buffer(3)]],
    device const float4* b4 [[buffer(4)]],
    device const float4* b5 [[buffer(5)]],
    device float4* b6 [[buffer(6)]],
    device float4* b7 [[buffer(7)]],
    device float4* b8 [[buffer(8)]]) {
  k_predict(tid, b0, b1, b2, b3, b4, b5, b6, b7, b8);
}

kernel void contact_setup(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuParticle* b1 [[buffer(1)]],
    device const GpuFabric* b2 [[buffer(2)]],
    device const float4* b3 [[buffer(3)]],
    device const float* b4 [[buffer(4)]],
    device const FieldInfo* b5 [[buffer(5)]],
    device GpuContact* b6 [[buffer(6)]]) {
  k_contact_setup(tid, b0, b1, b2, b3, b4, b5, b6);
}

kernel void contact_activate(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device GpuContact* b1 [[buffer(1)]],
    device float4* b2 [[buffer(2)]]) {
  k_contact_activate(tid, b0, b1, b2);
}

kernel void inertia_friction(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuParticle* b1 [[buffer(1)]],
    device const GpuFabric* b2 [[buffer(2)]],
    device const float4* b3 [[buffer(3)]],
    device const float4* b4 [[buffer(4)]],
    device const GpuContact* b5 [[buffer(5)]],
    device float* b6 [[buffer(6)]],
    device float4* b7 [[buffer(7)]],
    device Block3* b8 [[buffer(8)]]) {
  k_inertia_friction(tid, b0, b1, b2, b3, b4, b5, b6, b7, b8);
}

kernel void stretch_terms(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuStretch* b1 [[buffer(1)]],
    device const GpuFabric* b2 [[buffer(2)]],
    device const float4* b3 [[buffer(3)]],
    device const float4* b4 [[buffer(4)]],
    device float* b5 [[buffer(5)]],
    device float4* b6 [[buffer(6)]],
    device Block3* b7 [[buffer(7)]]) {
  k_stretch_terms(tid, b0, b1, b2, b3, b4, b5, b6, b7);
}

kernel void hinge_terms(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuHinge* b1 [[buffer(1)]],
    device const float4* b2 [[buffer(2)]],
    device const float4* b3 [[buffer(3)]],
    device float* b4 [[buffer(4)]],
    device float4* b5 [[buffer(5)]],
    device Block3* b6 [[buffer(6)]]) {
  k_hinge_terms(tid, b0, b1, b2, b3, b4, b5, b6);
}

kernel void seam_terms(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuSeam* b1 [[buffer(1)]],
    device const GpuGarment* b2 [[buffer(2)]],
    device const float4* b3 [[buffer(3)]],
    device const float4* b4 [[buffer(4)]],
    device float* b5 [[buffer(5)]],
    device float4* b6 [[buffer(6)]],
    device Block3* b7 [[buffer(7)]]) {
  k_seam_terms(tid, b0, b1, b2, b3, b4, b5, b6, b7);
}

kernel void gather_grad(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuParticle* b1 [[buffer(1)]],
    device const uint* b2 [[buffer(2)]],
    device const uint* b3 [[buffer(3)]],
    device const float4* b4 [[buffer(4)]],
    device float4* b5 [[buffer(5)]]) {
  k_gather_grad(tid, b0, b1, b2, b3, b4, b5);
}

kernel void gather_blocks(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const uint* b1 [[buffer(1)]],
    device const uint* b2 [[buffer(2)]],
    device const Block3* b3 [[buffer(3)]],
    device Block3* b4 [[buffer(4)]]) {
  k_gather_blocks(tid, b0, b1, b2, b3, b4);
}

kernel void contact_release(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device GpuContact* b1 [[buffer(1)]],
    device const float4* b2 [[buffer(2)]]) {
  k_contact_release(tid, b0, b1, b2);
}

kernel void precondition(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuParticle* b1 [[buffer(1)]],
    device const uint* b2 [[buffer(2)]],
    device const Block3* b3 [[buffer(3)]],
    device Block3* b4 [[buffer(4)]]) {
  k_precondition(tid, b0, b1, b2, b3, b4);
}

kernel void cg_init(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuContact* b1 [[buffer(1)]],
    device const Block3* b2 [[buffer(2)]],
    device const float4* b3 [[buffer(3)]],
    device float4* b4 [[buffer(4)]],
    device float4* b5 [[buffer(5)]],
    device float4* b6 [[buffer(6)]],
    device float4* b7 [[buffer(7)]],
    device float4* b8 [[buffer(8)]],
    device float* b9 [[buffer(9)]],
    device float* b10 [[buffer(10)]]) {
  k_cg_init(tid, b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10);
}

kernel void spmv(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuParticle* b1 [[buffer(1)]],
    device const uint* b2 [[buffer(2)]],
    device const uint* b3 [[buffer(3)]],
    device const Block3* b4 [[buffer(4)]],
    device const GpuContact* b5 [[buffer(5)]],
    device const CgScalars* b6 [[buffer(6)]],
    device const float4* b7 [[buffer(7)]],
    device float4* b8 [[buffer(8)]],
    device float4* b9 [[buffer(9)]],
    device float* b10 [[buffer(10)]]) {
  k_spmv(tid, b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10);
}

kernel void cg_alpha(
    uint tid [[thread_position_in_grid]],
    device CgScalars* b0 [[buffer(0)]]) {
  k_cg_alpha(tid, b0);
}

kernel void cg_update(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const CgScalars* b1 [[buffer(1)]],
    device const float4* b2 [[buffer(2)]],
    device const float4* b3 [[buffer(3)]],
    device const float4* b4 [[buffer(4)]],
    device float4* b5 [[buffer(5)]],
    device float4* b6 [[buffer(6)]],
    device float4* b7 [[buffer(7)]],
    device float* b8 [[buffer(8)]]) {
  k_cg_update(tid, b0, b1, b2, b3, b4, b5, b6, b7, b8);
}

kernel void cg_check(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device CgScalars* b1 [[buffer(1)]]) {
  k_cg_check(tid, b0, b1);
}

kernel void cg_precond(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuContact* b1 [[buffer(1)]],
    device const CgScalars* b2 [[buffer(2)]],
    device const Block3* b3 [[buffer(3)]],
    device const float4* b4 [[buffer(4)]],
    device float4* b5 [[buffer(5)]],
    device float* b6 [[buffer(6)]]) {
  k_cg_precond(tid, b0, b1, b2, b3, b4, b5, b6);
}

kernel void cg_beta(
    uint tid [[thread_position_in_grid]],
    device CgScalars* b0 [[buffer(0)]]) {
  k_cg_beta(tid, b0);
}

kernel void cg_direction(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const CgScalars* b1 [[buffer(1)]],
    device const float4* b2 [[buffer(2)]],
    device float4* b3 [[buffer(3)]]) {
  k_cg_direction(tid, b0, b1, b2, b3);
}

kernel void reduce_partial(
    uint tid [[thread_position_in_grid]],
    device const ReduceSpec* b0 [[buffer(0)]],
    device const float* b1 [[buffer(1)]],
    device float* b2 [[buffer(2)]]) {
  k_reduce_partial(tid, b0, b1, b2);
}

kernel void reduce_final(
    uint tid [[thread_position_in_grid]],
    device const ReduceSpec* b0 [[buffer(0)]],
    device const float* b1 [[buffer(1)]],
    device CgScalars* b2 [[buffer(2)]]) {
  k_reduce_final(tid, b0, b1, b2);
}

kernel void contact_force(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device GpuContact* b1 [[buffer(1)]],
    device const float4* b2 [[buffer(2)]]) {
  k_contact_force(tid, b0, b1, b2);
}

kernel void trial_positions(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const float4* b1 [[buffer(1)]],
    device const float4* b2 [[buffer(2)]],
    device float4* b3 [[buffer(3)]]) {
  k_trial_positions(tid, b0, b1, b2, b3);
}

kernel void apply_step(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device GpuContact* b1 [[buffer(1)]],
    device float4* b2 [[buffer(2)]],
    device const float4* b3 [[buffer(3)]],
    device float* b4 [[buffer(4)]]) {
  k_apply_step(tid, b0, b1, b2, b3, b4);
}

kernel void finish_substep(
    uint tid [[thread_position_in_grid]],
    device const StepParams* b0 [[buffer(0)]],
    device const GpuParticle* b1 [[buffer(1)]],
    device const GpuFabric* b2 [[buffer(2)]],
    device const GpuGarment* b3 [[buffer(3)]],
    device const float4* b4 [[buffer(4)]],
    device const float4* b5 [[buffer(5)]],
    device const float4* b6 [[buffer(6)]],
    device const float* b7 [[buffer(7)]],
    device const FieldInfo* b8 [[buffer(8)]],
    device float4* b9 [[buffer(9)]],
    device float4* b10 [[buffer(10)]]) {
  k_finish_substep(tid, b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10);
}
