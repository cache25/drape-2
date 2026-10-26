// Per-thread bodies of the GPU solver kernels, compiled as MSL (drape_kernels.metal wraps each in a [[kernel]]) and
// as C++ (EmulatedDevice loops over tid). The buffer order in each signature is the order GpuSolverCore passes to
// ComputeDevice::dispatch. Every kernel returns immediately for tid past its count. No atomics: each thread writes
// only its own slots, and sums run in a fixed order, so results are deterministic.
//
// Term outputs are laid out by term kind (n particles, S stretch, H hinges, K seams):
//   energies:    particle i -> i;  stretch t -> n + t;          hinge h -> n + S + h;            seam k -> n + S + H + k
//   gradContrib: particle i -> i;  stretch t -> n + 3t + a;     hinge h -> n + 3S + 4h + a;      seam k -> n + 3S + 4H + 2k + a
//   hessContrib: particle i -> i;  stretch t -> n + 9t + 3a + b; hinge h -> n + 9S + 16h + 4a + b; seam k -> n + 9S + 16H + 4k + 2a + b
#pragma once

#include "term_math.h"

namespace dgpu {

inline float3 load3(const DRAPE_DEVICE float4* b, uint i) { return xyz(b[i]); }
inline void store3(DRAPE_DEVICE float4* b, uint i, float3 v) { b[i] = make4(v, 0.0f); }

// Removes the normal component at particles held on their contact plane.
inline float3 filterContact(GpuContact c, float3 v) {
  if (c.active == 0u) return v;
  const float3 n = xyz(c.normal);
  return v - n * dot(n, v);
}

inline float contactRadius(DRAPE_DEVICE const StepParams* P, GpuFabric f) {
  return P->collisionOffset + 0.5f * f.thickness;
}

// ---- Substep setup ----

// predict(P, particles, garments, x, v, fExt, xStart, yRel, u): xStart = x; yRel = h v + h^2 (g + fExt/m) with gravity
// only while Settling; u = h v (the initial guess x + h v as a displacement). Pinned: yRel = u = 0.
inline void k_predict(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuParticle* particles,
                      DRAPE_DEVICE const GpuGarment* garments, DRAPE_DEVICE const float4* x,
                      DRAPE_DEVICE const float4* v, DRAPE_DEVICE const float4* fExt, DRAPE_DEVICE float4* xStart,
                      DRAPE_DEVICE float4* yRel, DRAPE_DEVICE float4* u) {
  if (tid >= P->n) return;
  xStart[tid] = x[tid];
  const GpuParticle pa = particles[tid];
  if (pa.pinned != 0u) {
    store3(yRel, tid, zero3());
    store3(u, tid, zero3());
    return;
  }
  const float h = P->h;
  const float3 vel = load3(v, tid);
  const float3 g = garments[pa.garment].phase == 1u ? xyz(P->gravity) : zero3();
  store3(yRel, tid, h * vel + (h * h) * (g + load3(fExt, tid) / pa.mass));
  store3(u, tid, h * vel);
}

// contact_setup(P, particles, fabrics, xStart, fieldValues, fieldInfo, contact): a particle within radius + reach of the
// body gets the tangent plane at xStart, and friction uses last solve's normal force. Everything else is cleared.
inline void k_contact_setup(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuParticle* particles,
                            DRAPE_DEVICE const GpuFabric* fabrics, DRAPE_DEVICE const float4* xStart,
                            DRAPE_DEVICE const float* fieldValues, DRAPE_DEVICE const FieldInfo* fieldInfo,
                            DRAPE_DEVICE GpuContact* contact) {
  if (tid >= P->n) return;
  GpuContact c = contact[tid];
  const float lastForce = c.force;
  c.nearBody = 0u;
  c.active = 0u;
  c.frictionLambda = 0.0f;
  c.force = 0.0f;
  const GpuParticle pa = particles[tid];
  const FieldInfo info = fieldInfo[0];
  if (info.dims[0] != 0u && pa.pinned == 0u) {
    const float radius = contactRadius(P, fabrics[pa.fabric]);
    const float3 xs = load3(xStart, tid);
    const float phi = fieldSample(fieldValues, info, xs);
    if (phi < radius + P->contactReach) {
      const float3 normal = fieldGradient(fieldValues, info, xs);
      if (!isZero3(normal)) {
        c.nearBody = 1u;
        c.normal = make4(normal, radius - phi);
        c.frictionLambda = lastForce;
      }
    }
  }
  contact[tid] = c;
}

// contact_activate(P, contact, u): holds near particles on their plane for this Newton iteration (projecting any below it).
inline void k_contact_activate(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE GpuContact* contact,
                               DRAPE_DEVICE float4* u) {
  if (tid >= P->n) return;
  GpuContact c = contact[tid];
  if (c.nearBody == 0u) return;
  const float3 n = xyz(c.normal);
  float3 ui = load3(u, tid);
  const float gap = c.normal.w - dot(n, ui);
  if (gap > -P->contactSnap) c.active = 1u;
  if (c.active != 0u && gap > 0.0f) {
    ui += gap * n;
    store3(u, tid, ui);
  }
  contact[tid] = c;
}

// ---- Energy terms (positions = the displacement buffer being evaluated: u, or trialU in the line search) ----

// inertia_friction(P, particles, fabrics, positions, yRel, contact, energies, gradContrib, hessContrib)
inline void k_inertia_friction(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuParticle* particles,
                               DRAPE_DEVICE const GpuFabric* fabrics, DRAPE_DEVICE const float4* positions,
                               DRAPE_DEVICE const float4* yRel, DRAPE_DEVICE const GpuContact* contact,
                               DRAPE_DEVICE float* energies, DRAPE_DEVICE float4* gradContrib,
                               DRAPE_DEVICE Block3* hessContrib) {
  if (tid >= P->n) return;
  const GpuParticle pa = particles[tid];
  TermEvalF<1> te;
  clearTerm(te);
  if (pa.pinned == 0u) {
    const float k = pa.mass * P->invH2;
    const float3 d = load3(positions, tid) - load3(yRel, tid);
    te.energy = 0.5f * k * length_squared(d);
    te.grad[0] = k * d;
    te.hess[0][0] = scale(identity3(), k);
  }
  const GpuContact c = contact[tid];
  if (c.frictionLambda > 0.0f) {
    evalFrictionF(load3(positions, tid), xyz(c.normal), fabrics[pa.fabric].friction * c.frictionLambda,
                  P->frictionSlip * P->h, te);
  }
  energies[tid] = te.energy;
  if (P->withDerivatives == 0u) return;
  store3(gradContrib, tid, te.grad[0]);
  hessContrib[tid] = storeBlock(te.hess[0][0]);
}

// Term vertex positions relative to the term's first vertex at substep start: (xStart_k - xStart_v0) + u_k.
inline float3 termPosition(DRAPE_DEVICE const float4* xStart, DRAPE_DEVICE const float4* positions, uint v0, uint vk) {
  return (load3(xStart, vk) - load3(xStart, v0)) + load3(positions, vk);
}

// stretch_terms(P, stretch, fabrics, xStart, positions, energies, gradContrib, hessContrib)
inline void k_stretch_terms(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuStretch* stretch,
                            DRAPE_DEVICE const GpuFabric* fabrics, DRAPE_DEVICE const float4* xStart,
                            DRAPE_DEVICE const float4* positions, DRAPE_DEVICE float* energies,
                            DRAPE_DEVICE float4* gradContrib, DRAPE_DEVICE Block3* hessContrib) {
  if (tid >= P->nStretch) return;
  const GpuStretch t = stretch[tid];
  float3 xs[3];
  for (int a = 0; a < 3; ++a) xs[a] = termPosition(xStart, positions, t.v[0], t.v[a]);
  TermEvalF<3> te;
  clearTerm(te);
  evalStretchF(t, xs, fabrics[t.fabric], P->limitScale, te);
  energies[P->n + tid] = te.energy;
  if (P->withDerivatives == 0u) return;
  for (uint a = 0; a < 3u; ++a) {
    store3(gradContrib, P->n + 3u * tid + a, te.grad[a]);
    for (uint b = 0; b < 3u; ++b) hessContrib[P->n + 9u * tid + 3u * a + b] = storeBlock(te.hess[a][b]);
  }
}

// hinge_terms(P, hinges, xStart, positions, energies, gradContrib, hessContrib)
inline void k_hinge_terms(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuHinge* hinges,
                          DRAPE_DEVICE const float4* xStart, DRAPE_DEVICE const float4* positions,
                          DRAPE_DEVICE float* energies, DRAPE_DEVICE float4* gradContrib,
                          DRAPE_DEVICE Block3* hessContrib) {
  if (tid >= P->nHinge) return;
  const GpuHinge t = hinges[tid];
  float3 xs[4];
  for (int a = 0; a < 4; ++a) xs[a] = termPosition(xStart, positions, t.v[0], t.v[a]);
  TermEvalF<4> te;
  clearTerm(te);
  evalHingeF(t, xs, te);
  energies[P->n + P->nStretch + tid] = te.energy;
  if (P->withDerivatives == 0u) return;
  const uint gBase = P->n + 3u * P->nStretch + 4u * tid;
  const uint hBase = P->n + 9u * P->nStretch + 16u * tid;
  for (uint a = 0; a < 4u; ++a) {
    store3(gradContrib, gBase + a, te.grad[a]);
    for (uint b = 0; b < 4u; ++b) hessContrib[hBase + 4u * a + b] = storeBlock(te.hess[a][b]);
  }
}

// seam_terms(P, seams, garments, xStart, positions, energies, gradContrib, hessContrib): while Assembling the rest
// length shrinks from startGap at closingSpeed; otherwise it is 0.
inline void k_seam_terms(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuSeam* seams,
                         DRAPE_DEVICE const GpuGarment* garments, DRAPE_DEVICE const float4* xStart,
                         DRAPE_DEVICE const float4* positions, DRAPE_DEVICE float* energies,
                         DRAPE_DEVICE float4* gradContrib, DRAPE_DEVICE Block3* hessContrib) {
  if (tid >= P->nSeam) return;
  const GpuSeam s = seams[tid];
  const GpuGarment g = garments[s.garment];
  const float target = g.phase == 0u ? fmax(0.0f, s.startGap - P->closingSpeed * g.assemblyTime) : 0.0f;
  float3 xs[2];
  xs[0] = termPosition(xStart, positions, s.a, s.a);
  xs[1] = termPosition(xStart, positions, s.a, s.b);
  TermEvalF<2> te;
  clearTerm(te);
  evalSeamF(xs, target, P->seamStiffness, te);
  energies[P->n + P->nStretch + P->nHinge + tid] = te.energy;
  if (P->withDerivatives == 0u) return;
  const uint gBase = P->n + 3u * P->nStretch + 4u * P->nHinge + 2u * tid;
  const uint hBase = P->n + 9u * P->nStretch + 16u * P->nHinge + 4u * tid;
  for (uint a = 0; a < 2u; ++a) {
    store3(gradContrib, gBase + a, te.grad[a]);
    for (uint b = 0; b < 2u; ++b) hessContrib[hBase + 2u * a + b] = storeBlock(te.hess[a][b]);
  }
}

// gather_grad(P, particles, gradStart, gradIdx, gradContrib, grad): sums each particle's contributions in list order;
// pinned rows are zero.
inline void k_gather_grad(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuParticle* particles,
                          DRAPE_DEVICE const uint* gradStart, DRAPE_DEVICE const uint* gradIdx,
                          DRAPE_DEVICE const float4* gradContrib, DRAPE_DEVICE float4* grad) {
  if (tid >= P->n) return;
  float3 acc = zero3();
  if (particles[tid].pinned == 0u)
    for (uint k = gradStart[tid]; k < gradStart[tid + 1u]; ++k) acc += load3(gradContrib, gradIdx[k]);
  store3(grad, tid, acc);
}

// gather_blocks(P, blockStart, blockIdx, hessContrib, blocks): sums each CSR block slot's contributions in list order.
inline void k_gather_blocks(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const uint* blockStart,
                            DRAPE_DEVICE const uint* blockIdx, DRAPE_DEVICE const Block3* hessContrib,
                            DRAPE_DEVICE Block3* blocks) {
  if (tid >= P->nBlocks) return;
  Mat3 acc = zeroMat3();
  for (uint k = blockStart[tid]; k < blockStart[tid + 1u]; ++k) acc = add(acc, loadBlock(hessContrib[blockIdx[k]]));
  blocks[tid] = storeBlock(acc);
}

// contact_release(P, contact, grad): frees held particles the other forces pull off the body.
inline void k_contact_release(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE GpuContact* contact,
                              DRAPE_DEVICE const float4* grad) {
  if (tid >= P->n) return;
  if (contact[tid].active != 0u && dot(load3(grad, tid), xyz(contact[tid].normal)) < 0.0f) contact[tid].active = 0u;
}

// ---- Linear solve: block-Jacobi preconditioned CG restricted to contact planes ----

// precondition(P, particles, diagSlot, blocks, precond): inverse of each free particle's diagonal block (identity if
// it is not positive definite); zero for pinned particles.
inline void k_precondition(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuParticle* particles,
                           DRAPE_DEVICE const uint* diagSlot, DRAPE_DEVICE const Block3* blocks,
                           DRAPE_DEVICE Block3* precond) {
  if (tid >= P->n) return;
  if (particles[tid].pinned != 0u) {
    precond[tid] = storeBlock(zeroMat3());
    return;
  }
  bool ok = false;
  const Mat3 inv = invertSpd3(loadBlock(blocks[diagSlot[tid]]), ok);
  precond[tid] = storeBlock(ok ? inv : identity3());
}

// cg_init(P, contact, precond, grad, step, resid, r, z, p, prodA, prodB): step = 0, resid = -grad, r = filter(resid),
// z = filter(M r), p = z; prodA = r.z, prodB = r.r (reduced next into rz and r0).
inline void k_cg_init(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuContact* contact,
                      DRAPE_DEVICE const Block3* precond, DRAPE_DEVICE const float4* grad, DRAPE_DEVICE float4* step,
                      DRAPE_DEVICE float4* resid, DRAPE_DEVICE float4* r, DRAPE_DEVICE float4* z,
                      DRAPE_DEVICE float4* p, DRAPE_DEVICE float* prodA, DRAPE_DEVICE float* prodB) {
  if (tid >= P->n) return;
  const GpuContact c = contact[tid];
  const float3 res = -load3(grad, tid);
  const float3 ri = filterContact(c, res);
  const float3 zi = filterContact(c, mul(loadBlock(precond[tid]), ri));
  store3(step, tid, zero3());
  store3(resid, tid, res);
  store3(r, tid, ri);
  store3(z, tid, zi);
  store3(p, tid, zi);
  prodA[tid] = dot(ri, zi);
  prodB[tid] = dot(ri, ri);
}

// spmv(P, particles, rowStart, colIndex, blocks, contact, scalars, p, apFull, ap, prodA): apFull = A p over free
// particles, ap = filter(apFull), prodA = p.ap. No-op once converged.
inline void k_spmv(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuParticle* particles,
                   DRAPE_DEVICE const uint* rowStart, DRAPE_DEVICE const uint* colIndex,
                   DRAPE_DEVICE const Block3* blocks, DRAPE_DEVICE const GpuContact* contact,
                   DRAPE_DEVICE const CgScalars* scalars, DRAPE_DEVICE const float4* p, DRAPE_DEVICE float4* apFull,
                   DRAPE_DEVICE float4* ap, DRAPE_DEVICE float* prodA) {
  if (tid >= P->n || scalars[0].converged != 0u) return;
  float3 acc = zero3();
  if (particles[tid].pinned == 0u) {
    for (uint s = rowStart[tid]; s < rowStart[tid + 1u]; ++s) {
      const uint j = colIndex[s];
      if (particles[j].pinned == 0u) acc += mul(loadBlock(blocks[s]), load3(p, j));
    }
  }
  const float3 filtered = filterContact(contact[tid], acc);
  store3(apFull, tid, acc);
  store3(ap, tid, filtered);
  prodA[tid] = dot(load3(p, tid), filtered);
}

// cg_alpha(scalars): stops when pAp <= 0 (before the update, as on the CPU); otherwise alpha = rz / pAp.
inline void k_cg_alpha(uint tid, DRAPE_DEVICE CgScalars* scalars) {
  if (tid != 0u || scalars[0].converged != 0u) return;
  if (!(scalars[0].pAp > 0.0f)) {
    scalars[0].converged = 1u;
    return;
  }
  scalars[0].alpha = scalars[0].rz / scalars[0].pAp;
  scalars[0].iteration += 1u;
}

// cg_update(P, scalars, p, ap, apFull, step, r, resid, prodB): step += alpha p, r -= alpha ap, resid -= alpha apFull;
// prodB = r.r.
inline void k_cg_update(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const CgScalars* scalars,
                        DRAPE_DEVICE const float4* p, DRAPE_DEVICE const float4* ap, DRAPE_DEVICE const float4* apFull,
                        DRAPE_DEVICE float4* step, DRAPE_DEVICE float4* r, DRAPE_DEVICE float4* resid,
                        DRAPE_DEVICE float* prodB) {
  if (tid >= P->n || scalars[0].converged != 0u) return;
  const float alpha = scalars[0].alpha;
  store3(step, tid, load3(step, tid) + alpha * load3(p, tid));
  const float3 ri = load3(r, tid) - alpha * load3(ap, tid);
  store3(r, tid, ri);
  store3(resid, tid, load3(resid, tid) - alpha * load3(apFull, tid));
  prodB[tid] = dot(ri, ri);
}

// cg_check(P, scalars): converged once |r| <= tolerance * |r0| (after the update, as on the CPU).
inline void k_cg_check(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE CgScalars* scalars) {
  if (tid != 0u || scalars[0].converged != 0u) return;
  if (sqrt(scalars[0].rr) <= P->cgTolerance * scalars[0].r0) scalars[0].converged = 1u;
}

// cg_precond(P, contact, scalars, precond, r, z, prodA): z = filter(M r); prodA = r.z (reduced into the new rz).
inline void k_cg_precond(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuContact* contact,
                         DRAPE_DEVICE const CgScalars* scalars, DRAPE_DEVICE const Block3* precond,
                         DRAPE_DEVICE const float4* r, DRAPE_DEVICE float4* z, DRAPE_DEVICE float* prodA) {
  if (tid >= P->n || scalars[0].converged != 0u) return;
  const float3 ri = load3(r, tid);
  const float3 zi = filterContact(contact[tid], mul(loadBlock(precond[tid]), ri));
  store3(z, tid, zi);
  prodA[tid] = dot(ri, zi);
}

// cg_beta(scalars): beta = rzNew / rz; rz = rzNew (rzNew arrives in the beta field from the reduction).
inline void k_cg_beta(uint tid, DRAPE_DEVICE CgScalars* scalars) {
  if (tid != 0u || scalars[0].converged != 0u) return;
  const float rzNew = scalars[0].beta;
  scalars[0].beta = rzNew / scalars[0].rz;
  scalars[0].rz = rzNew;
}

// cg_direction(P, scalars, z, p): p = z + beta p.
inline void k_cg_direction(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const CgScalars* scalars,
                           DRAPE_DEVICE const float4* z, DRAPE_DEVICE float4* p) {
  if (tid >= P->n || scalars[0].converged != 0u) return;
  store3(p, tid, load3(z, tid) + scalars[0].beta * load3(p, tid));
}

// ---- Reductions (barrier-free, fixed order) ----

// reduce_partial(spec, input, partials): thread i sums input[256 i, 256 i + 256) in order.
inline void k_reduce_partial(uint tid, DRAPE_DEVICE const ReduceSpec* spec, DRAPE_DEVICE const float* input,
                             DRAPE_DEVICE float* partials) {
  const uint count = spec[0].count;
  const uint begin = tid * kReduceBlock;
  if (begin >= count) return;
  const uint end = begin + kReduceBlock < count ? begin + kReduceBlock : count;
  float s = 0.0f;
  for (uint k = begin; k < end; ++k) s += input[k];
  partials[tid] = s;
}

// reduce_final(spec, partials, scalars): thread 0 sums the partials in order into the spec's field.
inline void k_reduce_final(uint tid, DRAPE_DEVICE const ReduceSpec* spec, DRAPE_DEVICE const float* partials,
                           DRAPE_DEVICE CgScalars* scalars) {
  if (tid != 0u) return;
  const uint parts = (spec[0].count + kReduceBlock - 1u) / kReduceBlock;
  float s = 0.0f;
  for (uint k = 0; k < parts; ++k) s += partials[k];
  const uint sel = spec[0].selector;
  if (sel == kReduceRz) {
    scalars[0].rz = s;
  } else if (sel == kReduceR0) {
    scalars[0].rr = s;
    scalars[0].r0 = sqrt(s);
    scalars[0].converged = scalars[0].r0 > 0.0f ? 0u : 1u;
    scalars[0].iteration = 0u;
  } else if (sel == kReducePAp) {
    scalars[0].pAp = s;
  } else if (sel == kReduceRr) {
    scalars[0].rr = s;
  } else if (sel == kReduceRzNew) {
    scalars[0].beta = s;
  }
}

// ---- After the solve ----

// contact_force(P, contact, resid): each held particle's normal force from the residual (the body supplies whatever
// pushes inward).
inline void k_contact_force(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE GpuContact* contact,
                            DRAPE_DEVICE const float4* resid) {
  if (tid >= P->n) return;
  const GpuContact c = contact[tid];
  contact[tid].force = c.active != 0u ? fmax(0.0f, -dot(load3(resid, tid), xyz(c.normal))) : 0.0f;
}

// trial_positions(P, u, step, trialU): trialU = u + alpha step (alpha from the line-search params).
inline void k_trial_positions(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const float4* u,
                              DRAPE_DEVICE const float4* step, DRAPE_DEVICE float4* trialU) {
  if (tid >= P->n) return;
  store3(trialU, tid, load3(u, tid) + P->alpha * load3(step, tid));
}

// apply_step(P, contact, u, step, moved): u += alpha step; moved = max |component| of the move; friction for the next
// Newton iteration uses this solve's normal forces.
inline void k_apply_step(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE GpuContact* contact,
                         DRAPE_DEVICE float4* u, DRAPE_DEVICE const float4* step, DRAPE_DEVICE float* moved) {
  if (tid >= P->n) return;
  const float3 d = P->alpha * load3(step, tid);
  store3(u, tid, load3(u, tid) + d);
  moved[tid] = fmax(fabs(d.x), fmax(fabs(d.y), fabs(d.z)));
  contact[tid].frictionLambda = contact[tid].active != 0u ? contact[tid].force : 0.0f;
}

// finish_substep(P, particles, fabrics, garments, xStart, u, pinPos, fieldValues, fieldInfo, x, v):
// x = xStart + u, with the post-solve safety push-out for particles still deeper than the snap tolerance (landing on
// the body where the field is flat, else collideBody), pins, then v = (x - xStart)/h damped.
inline void k_finish_substep(uint tid, DRAPE_DEVICE const StepParams* P, DRAPE_DEVICE const GpuParticle* particles,
                             DRAPE_DEVICE const GpuFabric* fabrics, DRAPE_DEVICE const GpuGarment* garments,
                             DRAPE_DEVICE const float4* xStart, DRAPE_DEVICE const float4* u,
                             DRAPE_DEVICE const float4* pinPos, DRAPE_DEVICE const float* fieldValues,
                             DRAPE_DEVICE const FieldInfo* fieldInfo, DRAPE_DEVICE float4* x,
                             DRAPE_DEVICE float4* v) {
  if (tid >= P->n) return;
  const GpuParticle pa = particles[tid];
  const GpuFabric fab = fabrics[pa.fabric];
  const float3 xs = load3(xStart, tid);
  float3 xn = xs + load3(u, tid);
  const FieldInfo info = fieldInfo[0];
  if (pa.pinned != 0u) {
    xn = load3(pinPos, tid);
  } else if (info.dims[0] != 0u) {
    const float radius = contactRadius(P, fab);
    if (fieldSample(fieldValues, info, xn) < radius - P->contactSnap) {
      if (isZero3(fieldGradient(fieldValues, info, xn))) {
        xn = landOnBodyF(fieldValues, info, radius, xs, xn);
      } else {
        collideBodyF(fieldValues, info, radius, fab.friction, xs, xn);
      }
    }
  }
  float c = P->dampingScale * fab.damping;
  if (garments[pa.garment].phase == 0u) c += P->assemblyExtraDamping;
  const float3 vel = ((xn - xs) / P->h) * fmax(0.0f, 1.0f - c * P->h);
  store3(x, tid, xn);
  store3(v, tid, vel);
}

}  // namespace dgpu
