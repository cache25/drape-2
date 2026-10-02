#include "drape/avatar/collision_bake.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace drape::avatar {

namespace {

enum class Feature { VertexA, VertexB, VertexC, EdgeAB, EdgeBC, EdgeCA, Face };

// Closest point on triangle abc to p, with the feature it lies on (Ericson, RTCD §5.1.5).
Vec3d closestPointOnTriangle(const Vec3d& p, const Vec3d& a, const Vec3d& b, const Vec3d& c, Feature& feature) {
  const Vec3d ab = b - a, ac = c - a, ap = p - a;
  const double d1 = ab.dot(ap), d2 = ac.dot(ap);
  if (d1 <= 0 && d2 <= 0) {
    feature = Feature::VertexA;
    return a;
  }
  const Vec3d bp = p - b;
  const double d3 = ab.dot(bp), d4 = ac.dot(bp);
  if (d3 >= 0 && d4 <= d3) {
    feature = Feature::VertexB;
    return b;
  }
  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    feature = Feature::EdgeAB;
    return a + (d1 / (d1 - d3)) * ab;
  }
  const Vec3d cp = p - c;
  const double d5 = ab.dot(cp), d6 = ac.dot(cp);
  if (d6 >= 0 && d5 <= d6) {
    feature = Feature::VertexC;
    return c;
  }
  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    feature = Feature::EdgeCA;
    return a + (d2 / (d2 - d6)) * ac;
  }
  const double va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
    feature = Feature::EdgeBC;
    return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
  }
  const double denom = 1.0 / (va + vb + vc);
  feature = Feature::Face;
  return a + ab * (vb * denom) + ac * (vc * denom);
}

struct Pseudonormals {
  std::vector<Vec3d> face, vertex;
  std::map<std::pair<std::uint32_t, std::uint32_t>, Vec3d> edge;

  explicit Pseudonormals(const TriangleMesh& m) : face(m.triangles.size()), vertex(m.vertices.size(), Vec3d::Zero()) {
    for (std::size_t t = 0; t < m.triangles.size(); ++t) {
      const auto& tri = m.triangles[t];
      const Vec3d& a = m.vertices[tri[0]];
      const Vec3d& b = m.vertices[tri[1]];
      const Vec3d& c = m.vertices[tri[2]];
      face[t] = (b - a).cross(c - a).normalized();
      for (int k = 0; k < 3; ++k) {
        const Vec3d& p = m.vertices[tri[k]];
        const Vec3d e1 = (m.vertices[tri[(k + 1) % 3]] - p).normalized();
        const Vec3d e2 = (m.vertices[tri[(k + 2) % 3]] - p).normalized();
        vertex[tri[k]] += std::acos(std::clamp(e1.dot(e2), -1.0, 1.0)) * face[t];
        auto key = std::minmax(tri[k], tri[(k + 1) % 3]);
        auto [it, inserted] = edge.try_emplace(key, Vec3d::Zero());
        it->second += face[t];
      }
    }
  }

  Vec3d of(const TriangleMesh& m, std::size_t t, Feature f) const {
    const auto& tri = m.triangles[t];
    switch (f) {
      case Feature::VertexA: return vertex[tri[0]];
      case Feature::VertexB: return vertex[tri[1]];
      case Feature::VertexC: return vertex[tri[2]];
      case Feature::EdgeAB: return edge.at(std::minmax(tri[0], tri[1]));
      case Feature::EdgeBC: return edge.at(std::minmax(tri[1], tri[2]));
      case Feature::EdgeCA: return edge.at(std::minmax(tri[2], tri[0]));
      case Feature::Face: return face[t];
    }
    return face[t];
  }
};

}  // namespace

CollisionField bakeCollisionField(std::span<const TriangleMesh> components, const BakeOptions& options) {
  Vec3d lo = Vec3d::Constant(std::numeric_limits<double>::infinity());
  Vec3d hi = -lo;
  for (std::size_t c = 0; c < components.size(); ++c) {
    if (!isClosedManifold(components[c])) {
      throw std::invalid_argument("collision component " + std::to_string(c) + " is not a closed manifold");
    }
    for (const auto& v : components[c].vertices) {
      lo = lo.cwiseMin(v);
      hi = hi.cwiseMax(v);
    }
  }

  CollisionField field;
  field.cellSize = options.cellSize;
  field.band = options.band;
  field.origin = lo - Vec3d::Constant(options.padding);
  const Vec3d extent = (hi - lo) + Vec3d::Constant(2 * options.padding);
  for (int a = 0; a < 3; ++a) field.dims[a] = static_cast<int>(std::ceil(extent[a] / options.cellSize)) + 1;
  const std::size_t nx = field.dims[0], ny = field.dims[1], nz = field.dims[2];
  const std::size_t count = nx * ny * nz;
  field.values.assign(count, static_cast<float>(options.band));
  auto index = [&](std::size_t x, std::size_t y, std::size_t z) { return (z * ny + y) * nx + x; };
  auto position = [&](std::size_t x, std::size_t y, std::size_t z) {
    return Vec3d(field.origin + options.cellSize * Vec3d(static_cast<double>(x), static_cast<double>(y),
                                                         static_cast<double>(z)));
  };
  auto cellRange = [&](double minV, double maxV, int axis, std::size_t& i0, std::size_t& i1) {
    const double o = field.origin[axis];
    i0 = static_cast<std::size_t>(std::max(0.0, std::ceil((minV - o) / options.cellSize)));
    i1 = static_cast<std::size_t>(
        std::min<double>(field.dims[axis] - 1, std::floor((maxV - o) / options.cellSize)));
  };

  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> dist2(count);
  std::vector<std::int8_t> sign(count);
  std::vector<std::uint8_t> outside(count);

  for (const auto& mesh : components) {
    std::fill(dist2.begin(), dist2.end(), inf);
    std::fill(sign.begin(), sign.end(), 0);
    const Pseudonormals normals(mesh);
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
      const auto& tri = mesh.triangles[t];
      const Vec3d& a = mesh.vertices[tri[0]];
      const Vec3d& b = mesh.vertices[tri[1]];
      const Vec3d& c = mesh.vertices[tri[2]];
      const Vec3d tmin = a.cwiseMin(b).cwiseMin(c);
      const Vec3d tmax = a.cwiseMax(b).cwiseMax(c);
      std::size_t r[3][2];
      for (int axis = 0; axis < 3; ++axis) {
        cellRange(tmin[axis] - options.band, tmax[axis] + options.band, axis, r[axis][0], r[axis][1]);
      }
      for (std::size_t z = r[2][0]; z <= r[2][1]; ++z)
        for (std::size_t y = r[1][0]; y <= r[1][1]; ++y)
          for (std::size_t x = r[0][0]; x <= r[0][1]; ++x) {
            const std::size_t i = index(x, y, z);
            const Vec3d p = position(x, y, z);
            // Cheap reject: distance to the triangle's bounding box already exceeds the best.
            const Vec3d boxGap = (tmin - p).cwiseMax(p - tmax).cwiseMax(0.0);
            if (boxGap.squaredNorm() >= dist2[i]) continue;
            Feature f;
            const Vec3d q = closestPointOnTriangle(p, a, b, c, f);
            const double d2 = (p - q).squaredNorm();
            if (d2 < dist2[i]) {
              dist2[i] = d2;
              sign[i] = (p - q).dot(normals.of(mesh, t, f)) >= 0 ? 1 : -1;
            }
          }
    }

    // Samples outside the band: flood fill from the grid boundary marks the outside.
    std::fill(outside.begin(), outside.end(), 0);
    std::deque<std::size_t> queue;
    auto seed = [&](std::size_t x, std::size_t y, std::size_t z) {
      const std::size_t i = index(x, y, z);
      if (!outside[i] && dist2[i] > options.band * options.band) {
        outside[i] = 1;
        queue.push_back(i);
      }
    };
    for (std::size_t z = 0; z < nz; ++z)
      for (std::size_t y = 0; y < ny; ++y)
        for (std::size_t x = 0; x < nx; ++x) {
          if (x == 0 || y == 0 || z == 0 || x == nx - 1 || y == ny - 1 || z == nz - 1) seed(x, y, z);
        }
    while (!queue.empty()) {
      const std::size_t i = queue.front();
      queue.pop_front();
      const std::size_t x = i % nx, y = (i / nx) % ny, z = i / (nx * ny);
      if (x > 0) seed(x - 1, y, z);
      if (x + 1 < nx) seed(x + 1, y, z);
      if (y > 0) seed(x, y - 1, z);
      if (y + 1 < ny) seed(x, y + 1, z);
      if (z > 0) seed(x, y, z - 1);
      if (z + 1 < nz) seed(x, y, z + 1);
    }

    for (std::size_t i = 0; i < count; ++i) {
      double v;
      if (dist2[i] <= options.band * options.band) {
        v = sign[i] * std::sqrt(dist2[i]);
      } else {
        v = outside[i] ? options.band : -options.band;
      }
      field.values[i] = std::min(field.values[i], static_cast<float>(std::clamp(v, -options.band, options.band)));
    }
  }
  return field;
}

}  // namespace drape::avatar
