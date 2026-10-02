#include "drape/core/collision_field.hpp"

#include <algorithm>
#include <cmath>

namespace drape {

double CollisionField::sample(const Vec3d& p) const {
  if (values.empty()) return band;
  const Vec3d local = (p - origin) / cellSize;
  std::array<int, 3> i{};
  std::array<double, 3> t{};
  for (int a = 0; a < 3; ++a) {
    const double maxCoord = dims[a] - 1;
    if (!(local[a] >= 0.0 && local[a] <= maxCoord)) return band;  // also rejects NaN
    i[a] = std::min(static_cast<int>(std::floor(local[a])), dims[a] - 2);
    t[a] = local[a] - i[a];
  }
  auto at = [&](int x, int y, int z) {
    return static_cast<double>(values[(static_cast<std::size_t>(z) * dims[1] + y) * dims[0] + x]);
  };
  double result = 0;
  for (int dz = 0; dz < 2; ++dz)
    for (int dy = 0; dy < 2; ++dy)
      for (int dx = 0; dx < 2; ++dx) {
        const double w = (dx ? t[0] : 1 - t[0]) * (dy ? t[1] : 1 - t[1]) * (dz ? t[2] : 1 - t[2]);
        result += w * at(i[0] + dx, i[1] + dy, i[2] + dz);
      }
  return result;
}

Vec3d CollisionField::gradient(const Vec3d& p) const {
  Vec3d g;
  for (int a = 0; a < 3; ++a) {
    Vec3d d = Vec3d::Zero();
    d[a] = cellSize;
    g[a] = (sample(p + d) - sample(p - d)) / (2 * cellSize);
  }
  const double n = g.norm();
  return n < 1e-9 ? Vec3d::Zero() : Vec3d(g / n);
}

}  // namespace drape
