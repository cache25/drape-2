#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "drape/core/math.hpp"

namespace drape::sim {

struct BendCoefficients {
  double p = 0, q = 0;
};

// p = (3 Bwarp - Bweft)/2, q = Bweft, each >= 0.05 min(Bwarp, Bweft)   (spec 5.4)
// On the grain-aligned hex lattice, curvature across the grain is carried only by grain-aligned edges (q),
// and curvature along the grain by all edges at 2/3 p + 1/3 q (measured on a bent cylinder).
BendCoefficients bendingCoefficients(double bendWarp, double bendWeft);

// B(phi) = p sin^2(phi) + q cos^2(phi), phi = angle between the rest edge direction and the piece grain.
double edgeBendStiffness(const BendCoefficients& c, const Vec2d& restEdgeDir, const Vec2d& grain);

// x = {x1, x2, x3, x4}: x1, x2 opposite the hinge in triangles A and B; x3, x4 the hinge edge.
// theta = atan2((n1 x n2) . e, n1 . n2) with n1 = (x1-x3) x (x1-x4), n2 = (x2-x4) x (x2-x3), e = (x4-x3)/|x4-x3|.
// Gradients after Bridson, Marino & Fedkiw 2003, "Simulation of Clothing with Folds and Wrinkles", Sec. 4.
template <class S>
void evalDihedral(const std::array<Eigen::Matrix<S, 3, 1>, 4>& x, S& theta,
                  std::array<Eigen::Matrix<S, 3, 1>, 4>& grad) {
  using V = Eigen::Matrix<S, 3, 1>;
  const V n1 = (x[0] - x[2]).cross(x[0] - x[3]);
  const V n2 = (x[1] - x[3]).cross(x[1] - x[2]);
  const V E = x[3] - x[2];
  const S n1sq = n1.squaredNorm(), n2sq = n2.squaredNorm(), elen = E.norm();
  if (n1sq < S(1e-24) || n2sq < S(1e-24) || elen < S(1e-12)) {
    theta = S(0);
    for (auto& g : grad) g = V::Zero();
    return;
  }
  const V e = E / elen;
  const V m1 = n1 / std::sqrt(n1sq), m2 = n2 / std::sqrt(n2sq);
  theta = std::atan2(m1.cross(m2).dot(e), m1.dot(m2));
  const V a = n1 / n1sq, b = n2 / n2sq;
  // Bridson's expressions are for the opposite angle sign; negated to match theta above.
  grad[0] = -elen * a;
  grad[1] = -elen * b;
  grad[2] = -(((x[0] - x[3]).dot(E) / elen) * a + ((x[1] - x[3]).dot(E) / elen) * b);
  grad[3] = ((x[0] - x[2]).dot(E) / elen) * a + ((x[1] - x[2]).dot(E) / elen) * b;
}

}  // namespace drape::sim
