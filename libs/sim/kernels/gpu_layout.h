// Buffer element types shared by the host (GpuSolverCore) and the kernels. Only float, uint and float4 members, laid
// out identically by Apple Clang (host), GCC/Clang (emulated device) and the Metal compiler.
#pragma once

#include "gpu_compat.h"

namespace dgpu {

struct GpuStretch {
  uint v[4];     // three particle indices; v[3] unused
  float4 dmInv;  // (dmInv(0,0), dmInv(1,0), dmInv(0,1), dmInv(1,1)): column-major 2x2
  float area;
  uint fabric;
  uint pad[2];
};

struct GpuHinge {
  uint v[4];  // opposite A, opposite B, edge start, edge end
  float K, restAngle;
  uint pad[2];
};

struct GpuSeam {
  uint a, b, garment;
  float startGap;
};

// The FabricPhysical fields the kernels read, as float.
struct GpuFabric {
  float stretchWarp, stretchWeft, stretchBias;
  float strainLimitWarp, strainLimitWeft;
  float weight, thickness, friction, damping;
  float pad[3];
};

struct GpuGarment {
  uint phase;  // 0 Assembling, 1 Settling (SimPhase order)
  float assemblyTime;
  uint pad[2];
};

// Symmetric or general 3x3 block as three columns (w unused).
struct Block3 {
  float4 c0, c1, c2;
};

struct FieldInfo {
  float4 origin;
  uint dims[4];  // x, y, z; dims[3] unused
  float cellSize, band;
  uint pad[2];
};

struct CgScalars {
  float rz, pAp, alpha, beta, r0, rr;
  uint converged, iteration;
};

struct StepParams {
  float4 gravity;
  float h, invH2, alpha, limitScale, seamStiffness, closingSpeed, frictionSlip, cgTolerance;
  float collisionOffset, dampingScale, assemblyExtraDamping, contactReach, contactSnap;
  uint n, nStretch, nHinge, nSeam, nBlocks, withDerivatives, maxCgIterations;
};

inline Mat3 loadBlock(Block3 b) { return Mat3{xyz(b.c0), xyz(b.c1), xyz(b.c2)}; }
inline Block3 storeBlock(Mat3 m) { return Block3{make4(m.c0, 0.0f), make4(m.c1, 0.0f), make4(m.c2, 0.0f)}; }

}  // namespace dgpu
