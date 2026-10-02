#include "drape/avatar/primitives.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <utility>

namespace drape::avatar {

namespace {

void makeOutward(TriangleMesh& m) {
  if (signedVolume(m) < 0) {
    for (auto& t : m.triangles) std::swap(t[1], t[2]);
  }
}

// Ring-based closed surface: a south pole, `ringCount` rings of `segments` vertices, a north pole.
// point(ring, segment) gives ring vertices; poles are given separately.
TriangleMesh ringSurface(const Vec3d& south, const Vec3d& north, int ringCount, int segments,
                         const std::function<Vec3d(int, int)>& point) {
  TriangleMesh m;
  m.vertices.push_back(south);
  for (int r = 0; r < ringCount; ++r)
    for (int s = 0; s < segments; ++s) m.vertices.push_back(point(r, s));
  m.vertices.push_back(north);
  const auto northIdx = static_cast<std::uint32_t>(m.vertices.size() - 1);
  auto idx = [&](int r, int s) { return static_cast<std::uint32_t>(1 + r * segments + (s % segments)); };
  for (int s = 0; s < segments; ++s) m.triangles.push_back({0, idx(0, s + 1), idx(0, s)});
  for (int r = 0; r + 1 < ringCount; ++r) {
    for (int s = 0; s < segments; ++s) {
      m.triangles.push_back({idx(r, s), idx(r, s + 1), idx(r + 1, s + 1)});
      m.triangles.push_back({idx(r, s), idx(r + 1, s + 1), idx(r + 1, s)});
    }
  }
  for (int s = 0; s < segments; ++s) m.triangles.push_back({northIdx, idx(ringCount - 1, s), idx(ringCount - 1, s + 1)});
  makeOutward(m);
  return m;
}

// Latitude of ring r for a capsule with `rings` rings per hemisphere; the equator appears twice
// (ring rings-1 on the south half, ring rings on the north half) so the cylinder joins them.
double ringLatitude(int r, int rings) {
  if (r < rings) return -kPi / 2 + (r + 1) * (kPi / 2) / rings;
  return (r - rings) * (kPi / 2) / rings;
}

}  // namespace

TriangleMesh icosphere(const Vec3d& center, double radius, int subdivisions) {
  const double t = (1.0 + std::sqrt(5.0)) / 2.0;
  std::vector<Vec3d> v{{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                       {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
  for (auto& p : v) p.normalize();
  std::vector<std::array<std::uint32_t, 3>> f{{0, 11, 5}, {0, 5, 1},  {0, 1, 7},   {0, 7, 10}, {0, 10, 11},
                                              {1, 5, 9},  {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
                                              {3, 9, 4},  {3, 4, 2},  {3, 2, 6},   {3, 6, 8},  {3, 8, 9},
                                              {4, 9, 5},  {2, 4, 11}, {6, 2, 10},  {8, 6, 7},  {9, 8, 1}};
  for (int s = 0; s < subdivisions; ++s) {
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t> mid;
    auto midpoint = [&](std::uint32_t a, std::uint32_t b) {
      auto key = std::minmax(a, b);
      auto it = mid.find(key);
      if (it != mid.end()) return it->second;
      v.push_back((v[a] + v[b]).normalized());
      const auto i = static_cast<std::uint32_t>(v.size() - 1);
      mid[key] = i;
      return i;
    };
    std::vector<std::array<std::uint32_t, 3>> next;
    for (const auto& tri : f) {
      const auto a = midpoint(tri[0], tri[1]), b = midpoint(tri[1], tri[2]), c = midpoint(tri[2], tri[0]);
      next.push_back({tri[0], a, c});
      next.push_back({tri[1], b, a});
      next.push_back({tri[2], c, b});
      next.push_back({a, b, c});
    }
    f = std::move(next);
  }
  TriangleMesh m;
  for (const auto& p : v) m.vertices.push_back(center + radius * p);
  m.triangles = std::move(f);
  makeOutward(m);
  return m;
}

TriangleMesh capsule(const Vec3d& a, const Vec3d& b, double radius, int segments, int rings) {
  const Vec3d w = (b - a).normalized();
  Vec3d u = std::abs(w.x()) < 0.9 ? w.cross(Vec3d::UnitX()) : w.cross(Vec3d::UnitY());
  u.normalize();
  const Vec3d v = w.cross(u);
  auto point = [&](int r, int s) {
    const double phi = ringLatitude(r, rings);
    const double lambda = 2 * kPi * s / segments;
    const Vec3d& c = r < rings ? a : b;
    return Vec3d(c + radius * (std::cos(phi) * (std::cos(lambda) * u + std::sin(lambda) * v) + std::sin(phi) * w));
  };
  return ringSurface(a - radius * w, b + radius * w, 2 * rings, segments, point);
}

TriangleMesh ellipticCapsule(double yBottom, double yTop, double rx, double rz, double capHeight, int segments,
                             int rings) {
  auto point = [&](int r, int s) {
    const double phi = ringLatitude(r, rings);
    const double lambda = 2 * kPi * s / segments;
    const double y = (r < rings ? yBottom : yTop) + capHeight * std::sin(phi);
    return Vec3d(rx * std::cos(phi) * std::cos(lambda), y, rz * std::cos(phi) * std::sin(lambda));
  };
  return ringSurface({0, yBottom - capHeight, 0}, {0, yTop + capHeight, 0}, 2 * rings, segments, point);
}

TriangleMesh box(const Vec3d& center, const Vec3d& halfExtents, const Mat3d& rotation) {
  TriangleMesh m;
  for (int i = 0; i < 8; ++i) {
    const Vec3d local((i & 1 ? 1 : -1) * halfExtents.x(), (i & 2 ? 1 : -1) * halfExtents.y(),
                      (i & 4 ? 1 : -1) * halfExtents.z());
    m.vertices.push_back(center + rotation * local);
  }
  // Two triangles per face (corner index bits: x=1, y=2, z=4).
  m.triangles = {{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5}, {0, 5, 4},
                 {2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}};
  makeOutward(m);
  return m;
}

}  // namespace drape::avatar
