// 32-bit ports of the solver's energy terms, body-field sampling, contact helpers and the 3x3 SPD inverse. Each
// function mirrors its double-precision reference expression by expression (same branches, same constants):
//   evalStretchF  <- evalStretchTerm / evalStretch (energies.cpp, stretch.hpp)
//   evalHingeF    <- evalHingeTerm / evalDihedral (energies.cpp, bending.hpp)
//   evalSeamF     <- evalSeamTerm, evalFrictionF <- evalFrictionTerm (energies.cpp)
//   fieldSample / fieldGradient <- CollisionField::sample / gradient (collision_field.cpp)
//   collideBodyF / landOnBodyF  <- collideBody / landOnBody (collision.cpp)
// Term positions are relative to the term's first vertex at the start of the substep; the energies are translation
// invariant, so this only removes the absolute-coordinate rounding.
#pragma once

#include "gpu_layout.h"

namespace dgpu {

template <int N>
struct TermEvalF {
  float energy;
  float3 grad[N];
  Mat3 hess[N][N];
};

template <int N>
inline void clearTerm(DRAPE_THREAD TermEvalF<N>& t) {
  t.energy = 0.0f;
  for (int a = 0; a < N; ++a) {
    t.grad[a] = zero3();
    for (int b = 0; b < N; ++b) t.hess[a][b] = zeroMat3();
  }
}

template <int N>
inline void addQuadratic(DRAPE_THREAD TermEvalF<N>& out, float K, float C, DRAPE_THREAD const float3* g) {
  out.energy += 0.5f * K * C * C;
  for (int i = 0; i < N; ++i) {
    out.grad[i] += K * C * g[i];
    for (int j = 0; j < N; ++j) out.hess[i][j] = add(out.hess[i][j], scale(outer(g[i], g[j]), K));
  }
}

// ---- Stretch ----

// Coefficient of vertex `local` in f_u (column 0) or f_v (column 1): f = sum_k d_k x_k.
inline float vertexCoefficient(float4 d, int column, int local) {
  const float c0 = column == 0 ? d.x : d.z;  // dmInv(0, column)
  const float c1 = column == 0 ? d.y : d.w;  // dmInv(1, column)
  if (local == 1) return c0;
  if (local == 2) return c1;
  return -(c0 + c1);
}

// which: 0 warp (|f_u| - 1), 1 weft (|f_v| - 1), 2 bias (n_u . n_v).
inline void stretchStrain(DRAPE_THREAD const float3* x, float4 d, int which, DRAPE_THREAD float& C,
                          DRAPE_THREAD float3* grad) {
  const float3 e1 = x[1] - x[0];
  const float3 e2 = x[2] - x[0];
  const float3 fu = e1 * d.x + e2 * d.y;
  const float3 fv = e1 * d.z + e2 * d.w;
  const float lu = length(fu);
  const float lv = length(fv);
  const float tiny = 1e-12f;
  float3 dfu = zero3(), dfv = zero3();
  C = 0.0f;
  if (which == 0) {
    if (lu > tiny) {
      C = lu - 1.0f;
      dfu = fu / lu;
    }
  } else if (which == 1) {
    if (lv > tiny) {
      C = lv - 1.0f;
      dfv = fv / lv;
    }
  } else if (lu > tiny && lv > tiny) {
    const float3 nu = fu / lu;
    const float3 nv = fv / lv;
    const float c = dot(nu, nv);
    C = c;
    dfu = (nv - c * nu) / lu;
    dfv = (nu - c * nv) / lv;
  }
  grad[1] = dfu * d.x + dfv * d.z;
  grad[2] = dfu * d.y + dfv * d.w;
  grad[0] = -(grad[1] + grad[2]);
}

// Tension stiffness of 1/2 K C^2 with C = |f| - c0 > 0: K C d_i d_j (I - n n^T) / |f|.
inline void addTension(DRAPE_THREAD TermEvalF<3>& out, float K, float C, float4 d, int column, float3 f) {
  if (C <= 0.0f) return;
  const float len = length(f);
  if (len < 1e-12f) return;
  const float3 n = f / len;
  const Mat3 P = scale(sub(identity3(), outer(n, n)), K * C / len);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out.hess[i][j] = add(out.hess[i][j], scale(P, vertexCoefficient(d, column, i) * vertexCoefficient(d, column, j)));
}

inline void evalStretchF(GpuStretch t, DRAPE_THREAD const float3* x, GpuFabric f, float limitScale,
                         DRAPE_THREAD TermEvalF<3>& out) {
  const float k[3] = {f.stretchWarp, f.stretchWeft, f.stretchBias};
  const float limit[2] = {f.strainLimitWarp, f.strainLimitWeft};
  const float3 e1 = x[1] - x[0], e2 = x[2] - x[0];
  const float3 fcol[2] = {e1 * t.dmInv.x + e2 * t.dmInv.y, e1 * t.dmInv.z + e2 * t.dmInv.w};
  float3 g[3];
  float C = 0.0f;
  for (int which = 0; which < 3; ++which) {
    if (k[which] <= 0.0f) continue;
    stretchStrain(x, t.dmInv, which, C, g);
    const float K = k[which] * t.area;
    addQuadratic<3>(out, K, C, g);
    if (which == 2) continue;
    addTension(out, K, C, t.dmInv, which, fcol[which]);
    if (limit[which] > 0.0f && C > limit[which]) {
      const float KL = limitScale * K;
      const float excess = C - limit[which];
      addQuadratic<3>(out, KL, excess, g);
      addTension(out, KL, excess, t.dmInv, which, fcol[which]);
    }
  }
}

// ---- Hinge ----

// x = {x1, x2, x3, x4}: x1, x2 opposite the hinge; x3, x4 the hinge edge (Bridson, Marino & Fedkiw 2003).
inline void dihedral(DRAPE_THREAD const float3* x, DRAPE_THREAD float& theta, DRAPE_THREAD float3* grad) {
  const float3 n1 = cross(x[0] - x[2], x[0] - x[3]);
  const float3 n2 = cross(x[1] - x[3], x[1] - x[2]);
  const float3 E = x[3] - x[2];
  const float n1sq = length_squared(n1), n2sq = length_squared(n2), elen = length(E);
  if (n1sq < 1e-24f || n2sq < 1e-24f || elen < 1e-12f) {
    theta = 0.0f;
    for (int i = 0; i < 4; ++i) grad[i] = zero3();
    return;
  }
  const float3 e = E / elen;
  const float3 m1 = n1 / sqrt(n1sq), m2 = n2 / sqrt(n2sq);
  theta = atan2(dot(cross(m1, m2), e), dot(m1, m2));
  const float3 a = n1 / n1sq, b = n2 / n2sq;
  grad[0] = -elen * a;
  grad[1] = -elen * b;
  grad[2] = -((dot(x[0] - x[3], E) / elen) * a + (dot(x[1] - x[3], E) / elen) * b);
  grad[3] = (dot(x[0] - x[2], E) / elen) * a + (dot(x[1] - x[2], E) / elen) * b;
}

inline void evalHingeF(GpuHinge t, DRAPE_THREAD const float3* x, DRAPE_THREAD TermEvalF<4>& out) {
  float theta = 0.0f;
  float3 g[4];
  dihedral(x, theta, g);
  addQuadratic<4>(out, t.K, theta - t.restAngle, g);
}

// ---- Seam ----

// One-sided spring 1/2 K (d - target)^2 for d > target between x[0] and x[1].
inline void evalSeamF(DRAPE_THREAD const float3* x, float target, float K, DRAPE_THREAD TermEvalF<2>& out) {
  const float3 diff = x[0] - x[1];
  const float d = length(diff);
  if (d <= target) return;
  Mat3 block;
  if (d < 1e-12f) {
    block = scale(identity3(), K);
  } else {
    const float3 n = diff / d;
    const float ext = d - target;
    out.energy += 0.5f * K * ext * ext;
    out.grad[0] += K * ext * n;
    out.grad[1] -= K * ext * n;
    const Mat3 nn = outer(n, n);
    block = scale(add(nn, scale(sub(identity3(), nn), ext / d)), K);
  }
  out.hess[0][0] = add(out.hess[0][0], block);
  out.hess[1][1] = add(out.hess[1][1], block);
  out.hess[0][1] = sub(out.hess[0][1], block);
  out.hess[1][0] = sub(out.hess[1][0], block);
}

// ---- Friction ----

// Smoothed Coulomb friction (IPC): dx = x - x0, n the contact normal.
inline void evalFrictionF(float3 dx, float3 n, float muLambda, float eps, DRAPE_THREAD TermEvalF<1>& out) {
  if (muLambda <= 0.0f || eps <= 0.0f) return;
  const Mat3 P = sub(identity3(), outer(n, n));
  const float3 u = mul(P, dx);
  const float s = length(u);
  if (s < eps) {
    out.energy += muLambda * (-s * s * s / (3.0f * eps * eps) + s * s / eps + eps / 3.0f);
    const float f1OverS = 2.0f / eps - s / (eps * eps);
    out.grad[0] += muLambda * f1OverS * u;
    out.hess[0][0] = add(out.hess[0][0], scale(P, muLambda * f1OverS));
    if (s > 0.0f) out.hess[0][0] = sub(out.hess[0][0], scale(outer(u, u), muLambda / (eps * eps * s)));
  } else {
    const float3 dir = u / s;
    out.energy += muLambda * s;
    out.grad[0] += muLambda * dir;
    out.hess[0][0] = add(out.hess[0][0], scale(sub(P, outer(dir, dir)), muLambda / s));
  }
}

// ---- Body field ----

// Trilinear signed distance; any point outside the sampled box (or NaN) gives +band.
inline float fieldSample(const DRAPE_DEVICE float* values, FieldInfo info, float3 p) {
  if (info.dims[0] == 0u) return info.band;
  const float3 local = (p - xyz(info.origin)) / info.cellSize;
  const float l[3] = {local.x, local.y, local.z};
  int i[3];
  float t[3];
  for (int a = 0; a < 3; ++a) {
    const float maxCoord = float(info.dims[a]) - 1.0f;
    if (!(l[a] >= 0.0f && l[a] <= maxCoord)) return info.band;
    const int fl = int(floor(l[a]));
    const int top = int(info.dims[a]) - 2;
    i[a] = fl < top ? fl : top;
    t[a] = l[a] - float(i[a]);
  }
  float result = 0.0f;
  for (int dz = 0; dz < 2; ++dz)
    for (int dy = 0; dy < 2; ++dy)
      for (int dx = 0; dx < 2; ++dx) {
        const float w = (dx ? t[0] : 1.0f - t[0]) * (dy ? t[1] : 1.0f - t[1]) * (dz ? t[2] : 1.0f - t[2]);
        const uint idx = (uint(i[2] + dz) * info.dims[1] + uint(i[1] + dy)) * info.dims[0] + uint(i[0] + dx);
        result += w * values[idx];
      }
  return result;
}

// Central differences with step cellSize, normalized; zero where the field is flat. The reference uses |g| < 1e-9 in
// double; in float the trilinear weights of a flat (clamped) region sum to 1 only within ~1e-7, which leaves |g| ~ 1e-6
// of noise, so "flat" here is |g| < 1e-4 (the true slope of a distance field is ~1).
constant_or_static float kFlatGradient = 1e-4f;
inline float3 fieldGradient(const DRAPE_DEVICE float* values, FieldInfo info, float3 p) {
  const float h = info.cellSize;
  const float3 g = float3{
      (fieldSample(values, info, p + float3{h, 0.0f, 0.0f}) - fieldSample(values, info, p - float3{h, 0.0f, 0.0f})) /
          (2.0f * h),
      (fieldSample(values, info, p + float3{0.0f, h, 0.0f}) - fieldSample(values, info, p - float3{0.0f, h, 0.0f})) /
          (2.0f * h),
      (fieldSample(values, info, p + float3{0.0f, 0.0f, h}) - fieldSample(values, info, p - float3{0.0f, 0.0f, h})) /
          (2.0f * h)};
  const float n = length(g);
  return n < kFlatGradient ? zero3() : g / n;
}

inline bool isZero3(float3 v) { return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f; }

// Push out to `radius` along the field gradient, then positional Coulomb friction against this substep's motion.
inline void collideBodyF(const DRAPE_DEVICE float* values, FieldInfo info, float radius, float friction, float3 xPrev,
                         DRAPE_THREAD float3& x) {
  const float phi = fieldSample(values, info, x);
  if (phi >= radius) return;
  const float3 n = fieldGradient(values, info, x);
  if (isZero3(n)) return;
  const float depth = radius - phi;
  x += depth * n;
  const float3 delta = x - xPrev;
  const float3 tangential = delta - dot(delta, n) * n;
  const float t = length(tangential);
  if (t <= friction * depth) {
    x -= tangential;
  } else if (t > 0.0f) {
    x -= tangential * (friction * depth / t);
  }
}

// Last point on from -> to that is at least `radius` from the body (40 bisections), or `from` if it is closer.
inline float3 landOnBodyF(const DRAPE_DEVICE float* values, FieldInfo info, float radius, float3 from, float3 to) {
  if (!(fieldSample(values, info, from) >= radius)) return from;
  float lo = 0.0f, hi = 1.0f;
  for (int i = 0; i < 40; ++i) {
    const float mid = 0.5f * (lo + hi);
    if (fieldSample(values, info, from + mid * (to - from)) >= radius) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return from + lo * (to - from);
}

// ---- 3x3 SPD inverse ----

// Inverse of a symmetric matrix by LDL^T (no pivoting; reads the lower triangle). ok = false, and the identity is
// returned, when a pivot is not positive.
inline Mat3 invertSpd3(Mat3 A, DRAPE_THREAD bool& ok) {
  const float a00 = A.c0.x, a10 = A.c0.y, a20 = A.c0.z, a11 = A.c1.y, a21 = A.c1.z, a22 = A.c2.z;
  ok = false;
  const float d0 = a00;
  if (!(d0 > 0.0f)) return identity3();
  const float l10 = a10 / d0, l20 = a20 / d0;
  const float d1 = a11 - l10 * l10 * d0;
  if (!(d1 > 0.0f)) return identity3();
  const float l21 = (a21 - l20 * l10 * d0) / d1;
  const float d2 = a22 - l20 * l20 * d0 - l21 * l21 * d1;
  if (!(d2 > 0.0f)) return identity3();
  ok = true;
  float3 cols[3];
  for (int k = 0; k < 3; ++k) {
    const float e0 = k == 0 ? 1.0f : 0.0f, e1 = k == 1 ? 1.0f : 0.0f, e2 = k == 2 ? 1.0f : 0.0f;
    // L y = e
    const float y0 = e0;
    const float y1 = e1 - l10 * y0;
    const float y2 = e2 - l20 * y0 - l21 * y1;
    // D z = y; L^T x = z
    const float z0 = y0 / d0, z1 = y1 / d1, z2 = y2 / d2;
    const float x2 = z2;
    const float x1 = z1 - l21 * x2;
    const float x0 = z0 - l10 * x1 - l20 * x2;
    cols[k] = float3{x0, x1, x2};
  }
  return Mat3{cols[0], cols[1], cols[2]};
}

}  // namespace dgpu
