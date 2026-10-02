#include "energies.hpp"

#include "bending.hpp"
#include "stretch.hpp"

namespace drape::sim {

namespace {

// Coefficient of vertex `local` in f_u (column 0) or f_v (column 1): f = sum_k d_k x_k.
double vertexCoefficient(const Eigen::Matrix2d& dmInv, int column, int local) {
  if (local == 1) return dmInv(0, column);
  if (local == 2) return dmInv(1, column);
  return -(dmInv(0, column) + dmInv(1, column));
}

template <std::size_t N>
void addQuadratic(TermEval<N>& out, double K, double C, const std::array<V3, N>& g) {
  out.energy += 0.5 * K * C * C;
  for (std::size_t i = 0; i < N; ++i) {
    out.grad[i] += K * C * g[i];
    for (std::size_t j = 0; j < N; ++j) out.hess[i][j] += K * g[i] * g[j].transpose();
  }
}

// Tension stiffness of 1/2 K C^2 with C = |f| - c0 > 0: K C d_i d_j (I - n n^T) / |f|.
void addTension(TermEval<3>& out, double K, double C, const Eigen::Matrix2d& dmInv, int column, const V3& f) {
  if (C <= 0) return;
  const double len = f.norm();
  if (len < 1e-12) return;
  const V3 n = f / len;
  const M3 P = (M3::Identity() - n * n.transpose()) * (K * C / len);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out.hess[i][j] += vertexCoefficient(dmInv, column, i) * vertexCoefficient(dmInv, column, j) * P;
}

}  // namespace

void evalStretchTerm(const StretchTerm& t, const std::array<V3, 3>& x, const FabricPhysical& f, double limitScale,
                     TermEval<3>& out) {
  const double k[3] = {f.stretchWarp, f.stretchWeft, f.stretchBias};
  const double limit[2] = {f.strainLimitWarp, f.strainLimitWeft};
  const V3 e1 = x[1] - x[0], e2 = x[2] - x[0];
  const V3 fcol[2] = {e1 * t.dmInv(0, 0) + e2 * t.dmInv(1, 0), e1 * t.dmInv(0, 1) + e2 * t.dmInv(1, 1)};
  std::array<V3, 3> g;
  double C = 0;
  for (int which = 0; which < 3; ++which) {
    if (k[which] <= 0) continue;
    evalStretch<double>(x, t.dmInv, which, C, g);
    const double K = k[which] * t.area;
    addQuadratic(out, K, C, g);
    if (which == 2) continue;
    addTension(out, K, C, t.dmInv, which, fcol[which]);
    if (limit[which] > 0 && C > limit[which]) {
      const double KL = limitScale * K;
      const double excess = C - limit[which];
      addQuadratic(out, KL, excess, g);
      addTension(out, KL, excess, t.dmInv, which, fcol[which]);
    }
  }
}

void evalHingeTerm(const HingeTerm& t, const std::array<V3, 4>& x, TermEval<4>& out) {
  double theta = 0;
  std::array<V3, 4> g;
  evalDihedral<double>(x, theta, g);
  addQuadratic(out, t.K, theta - t.restAngle, g);
}

void evalSeamTerm(const std::array<V3, 2>& x, double target, double K, TermEval<2>& out) {
  const V3 diff = x[0] - x[1];
  const double d = diff.norm();
  if (d <= target) return;
  M3 block;
  if (d < 1e-12) {
    block = K * M3::Identity();
  } else {
    const V3 n = diff / d;
    const double ext = d - target;
    out.energy += 0.5 * K * ext * ext;
    out.grad[0] += K * ext * n;
    out.grad[1] -= K * ext * n;
    const M3 nn = n * n.transpose();
    block = K * (nn + (ext / d) * (M3::Identity() - nn));
  }
  out.hess[0][0] += block;
  out.hess[1][1] += block;
  out.hess[0][1] -= block;
  out.hess[1][0] -= block;
}

}  // namespace drape::sim
