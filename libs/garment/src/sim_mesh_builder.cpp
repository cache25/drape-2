#include "drape/garment/sim_mesh_builder.hpp"

#include <CDT.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "drape/core/polygon.hpp"
#include "drape/garment/validate.hpp"

namespace drape::garment {

double particleDistance(SimQuality q) {
  switch (q) {
    case SimQuality::Draft: return 0.020;
    case SimQuality::Standard: return 0.010;
    case SimQuality::Fine: return 0.005;
  }
  return 0.010;
}

namespace {

constexpr double kEps = 1e-9;
constexpr double kCornerDegrees = 10.0;

// Step count for a length; the small epsilon keeps 0.1/0.01 from rounding up to 11.
int stepsCeil(double len, double h) { return std::max(1, static_cast<int>(std::ceil(len / h - 1e-9))); }
int stepsRound(double len, double h) { return std::max(1, static_cast<int>(std::lround(len / h))); }

struct SeamSide {
  std::size_t seam;
  double start, end;
  bool reversed;
  int steps;
};

// Sorted arc positions along one piece outline, with lookup by position.
struct BoundaryLoop {
  double perimeter = 0;
  std::vector<double> positions;  // ascending, in [0, perimeter)

  double wrap(double s) const { return s >= perimeter - kEps ? 0.0 : s; }

  void add(double s) { positions.push_back(wrap(s)); }

  void finish() {
    std::sort(positions.begin(), positions.end());
    std::vector<double> out;
    for (double s : positions) {
      if (out.empty() || s - out.back() > kEps) out.push_back(s);
    }
    positions = std::move(out);
  }

  std::size_t indexOf(double s) const {
    s = wrap(s);
    auto it = std::lower_bound(positions.begin(), positions.end(), s - kEps);
    if (it == positions.end() || std::abs(*it - s) > kEps) throw MeshError("internal: boundary sample not found");
    return static_cast<std::size_t>(it - positions.begin());
  }
};

std::vector<double> cornerPositions(const Outline& o) {
  std::vector<double> out;
  for (std::size_t i = 0; i < o.size(); ++i) {
    const auto& prev = o.segment((i + o.size() - 1) % o.size());
    const auto& next = o.segment(i);
    Vec2d tin = prev.derivative(1.0);
    Vec2d tout = next.derivative(0.0);
    if (tin.norm() < 1e-14) tin = prev.p3 - prev.p0;
    if (tout.norm() < 1e-14) tout = next.p3 - next.p0;
    const double c = std::clamp(tin.normalized().dot(tout.normalized()), -1.0, 1.0);
    if (std::acos(c) * 180.0 / kPi > kCornerDegrees) out.push_back(o.segmentStart(i));
  }
  return out;
}

// Edges of the boundary polygon bucketed into square cells for near-boundary queries.
class EdgeGrid {
 public:
  EdgeGrid(const std::vector<Vec2d>& poly, double cell) : poly_(poly), cell_(cell) {
    for (std::size_t i = 0; i < poly.size(); ++i) {
      const Vec2d& a = poly[i];
      const Vec2d& b = poly[(i + 1) % poly.size()];
      const int x0 = cellOf(std::min(a.x(), b.x()) - cell), x1 = cellOf(std::max(a.x(), b.x()) + cell);
      const int y0 = cellOf(std::min(a.y(), b.y()) - cell), y1 = cellOf(std::max(a.y(), b.y()) + cell);
      for (int x = x0; x <= x1; ++x)
        for (int y = y0; y <= y1; ++y) cells_[key(x, y)].push_back(i);
    }
  }

  // Distance to the nearest boundary edge, or +inf if none within one cell.
  double nearest(const Vec2d& p) const {
    auto it = cells_.find(key(cellOf(p.x()), cellOf(p.y())));
    if (it == cells_.end()) return std::numeric_limits<double>::infinity();
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i : it->second) {
      best = std::min(best, distancePointSegment(p, poly_[i], poly_[(i + 1) % poly_.size()]));
    }
    return best;
  }

 private:
  int cellOf(double v) const { return static_cast<int>(std::floor(v / cell_)); }
  static std::int64_t key(int x, int y) { return (static_cast<std::int64_t>(x) << 32) ^ static_cast<std::uint32_t>(y); }

  const std::vector<Vec2d>& poly_;
  double cell_;
  std::unordered_map<std::int64_t, std::vector<std::size_t>> cells_;
};

// Grain-aligned hexagonal lattice points inside the polygon and >= 0.5 h from its boundary.
std::vector<Vec2d> latticePoints(const std::vector<Vec2d>& poly, const Vec2d& grain, double h) {
  const Vec2d u = grain.normalized();
  const Vec2d v(-u.y(), u.x());
  std::vector<Vec2d> local(poly.size());
  double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
  for (std::size_t i = 0; i < poly.size(); ++i) {
    local[i] = Vec2d(poly[i].dot(u), poly[i].dot(v));
    umin = std::min(umin, local[i].x());
    umax = std::max(umax, local[i].x());
    vmin = std::min(vmin, local[i].y());
    vmax = std::max(vmax, local[i].y());
  }
  const double rowStep = std::sqrt(3.0) / 2.0 * h;
  EdgeGrid grid(poly, h);
  std::vector<Vec2d> out;
  const int j0 = static_cast<int>(std::floor(vmin / rowStep)) - 1;
  const int j1 = static_cast<int>(std::ceil(vmax / rowStep)) + 1;
  std::vector<double> crossings;
  for (int j = j0; j <= j1; ++j) {
    const double vy = j * rowStep;
    crossings.clear();
    for (std::size_t k = 0; k < local.size(); ++k) {
      const Vec2d& a = local[k];
      const Vec2d& b = local[(k + 1) % local.size()];
      if ((a.y() > vy) != (b.y() > vy)) crossings.push_back(a.x() + (vy - a.y()) * (b.x() - a.x()) / (b.y() - a.y()));
    }
    std::sort(crossings.begin(), crossings.end());
    const double shift = ((j % 2) + 2) % 2 == 1 ? 0.5 : 0.0;
    const int i0 = static_cast<int>(std::floor(umin / h)) - 1;
    const int i1 = static_cast<int>(std::ceil(umax / h)) + 1;
    for (int i = i0; i <= i1; ++i) {
      const double ux = (i + shift) * h;
      const auto below = std::lower_bound(crossings.begin(), crossings.end(), ux) - crossings.begin();
      if (below % 2 == 0) continue;  // outside
      const Vec2d p = ux * u + vy * v;
      if (grid.nearest(p) < 0.5 * h) continue;
      out.push_back(p);
    }
  }
  return out;
}

double signedTriArea(const Vec2d& a, const Vec2d& b, const Vec2d& c) {
  return 0.5 * ((b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x()));
}

// Jacobi Laplacian smoothing of interior vertices; a move is kept only if no incident triangle flips.
void smooth(std::vector<Vec2d>& pts, const std::vector<std::array<std::uint32_t, 3>>& tris, std::size_t boundaryCount,
            int iterations) {
  std::vector<std::vector<std::uint32_t>> nbrs(pts.size()), incident(pts.size());
  for (std::uint32_t t = 0; t < tris.size(); ++t) {
    for (int k = 0; k < 3; ++k) {
      const auto a = tris[t][k];
      incident[a].push_back(t);
      for (int m = 1; m < 3; ++m) nbrs[a].push_back(tris[t][(k + m) % 3]);
    }
  }
  for (auto& n : nbrs) {
    std::sort(n.begin(), n.end());
    n.erase(std::unique(n.begin(), n.end()), n.end());
  }
  for (int it = 0; it < iterations; ++it) {
    const std::vector<Vec2d> prev = pts;
    for (std::size_t i = boundaryCount; i < pts.size(); ++i) {
      if (nbrs[i].empty()) continue;
      Vec2d mean = Vec2d::Zero();
      for (auto n : nbrs[i]) mean += prev[n];
      mean /= static_cast<double>(nbrs[i].size());
      const Vec2d old = pts[i];
      pts[i] = mean;
      for (auto t : incident[i]) {
        if (signedTriArea(pts[tris[t][0]], pts[tris[t][1]], pts[tris[t][2]]) <= 1e-14) {
          pts[i] = old;
          break;
        }
      }
    }
  }
}

}  // namespace

SimMesh buildSimMesh(const Pattern& pattern, double h) {
  if (const auto issues = validatePattern(pattern); !issues.empty()) {
    std::string msg = "pattern is invalid:";
    for (const auto& i : issues) msg += " [" + i.rule + "] " + i.message + ";";
    throw MeshError(msg);
  }

  // Seam sides per piece, each with the seam's shared step count.
  std::vector<std::vector<SeamSide>> sides(pattern.pieces.size());
  std::vector<int> seamSteps(pattern.seams.size());
  auto pieceIndex = [&](const Id& id) {
    for (std::size_t i = 0; i < pattern.pieces.size(); ++i)
      if (pattern.pieces[i].id == id) return i;
    throw MeshError("internal: unknown piece " + id);
  };
  for (std::size_t s = 0; s < pattern.seams.size(); ++s) {
    const auto& seam = pattern.seams[s];
    const int steps = stepsCeil(std::max(seam.a.end - seam.a.start, seam.b.end - seam.b.start), h);
    seamSteps[s] = steps;
    for (const EdgeRef* e : {&seam.a, &seam.b}) {
      sides[pieceIndex(e->piece)].push_back({s, e->start, e->end, e->reversed, steps});
    }
  }

  SimMesh mesh;
  std::vector<BoundaryLoop> loops(pattern.pieces.size());
  std::vector<std::uint32_t> loopOffset(pattern.pieces.size());

  for (std::size_t pi = 0; pi < pattern.pieces.size(); ++pi) {
    const auto& piece = pattern.pieces[pi];
    const Outline& o = piece.outline;
    BoundaryLoop& loop = loops[pi];
    loop.perimeter = o.perimeter();

    // Breakpoints: start, corners and seam ends.
    std::vector<double> breaks{0.0};
    for (double c : cornerPositions(o)) breaks.push_back(c);
    for (const auto& side : sides[pi]) {
      breaks.push_back(side.start);
      breaks.push_back(side.end);
    }
    for (auto& b : breaks) b = loop.wrap(b);
    std::sort(breaks.begin(), breaks.end());
    breaks.erase(std::unique(breaks.begin(), breaks.end(), [](double a, double b) { return b - a <= kEps; }),
                 breaks.end());

    // Seam edges are sampled as a whole with the seam's step count.
    for (const auto& side : sides[pi]) {
      for (int k = 0; k <= side.steps; ++k) loop.add(side.start + (side.end - side.start) * k / side.steps);
    }
    // Free spans between breakpoints.
    for (std::size_t b = 0; b < breaks.size(); ++b) {
      const double s0 = breaks[b];
      const double s1 = b + 1 < breaks.size() ? breaks[b + 1] : loop.perimeter;
      const double mid = 0.5 * (s0 + s1);
      const bool inSeam = std::any_of(sides[pi].begin(), sides[pi].end(),
                                      [&](const SeamSide& sd) { return mid > sd.start && mid < sd.end; });
      if (inSeam) continue;
      const int n = stepsRound(s1 - s0, h);
      for (int k = 0; k < n; ++k) loop.add(s0 + (s1 - s0) * k / n);
    }
    loop.finish();

    std::vector<Vec2d> pts;
    for (double s : loop.positions) pts.push_back(o.pointAt(s));
    const std::size_t boundaryCount = pts.size();
    const Vec2d grain = piece.grainline.normalized();
    for (const auto& p : latticePoints(pts, grain, h)) pts.push_back(p);

    // Constrained Delaunay triangulation of boundary + lattice.
    std::vector<CDT::V2d<double>> cdtPts;
    cdtPts.reserve(pts.size());
    for (const auto& p : pts) cdtPts.push_back({p.x(), p.y()});
    std::vector<CDT::Edge> cdtEdges;
    for (std::size_t i = 0; i < boundaryCount; ++i) {
      cdtEdges.emplace_back(static_cast<CDT::VertInd>(i), static_cast<CDT::VertInd>((i + 1) % boundaryCount));
    }
    const auto dup = CDT::RemoveDuplicatesAndRemapEdges(cdtPts, cdtEdges);
    if (!dup.duplicates.empty()) throw MeshError("piece '" + piece.id + "': duplicate mesh vertices");
    CDT::Triangulation<double> cdt;
    cdt.insertVertices(cdtPts);
    cdt.insertEdges(cdtEdges);
    cdt.eraseOuterTriangles();

    std::vector<std::array<std::uint32_t, 3>> tris;
    for (const auto& t : cdt.triangles) {
      std::array<std::uint32_t, 3> tri{t.vertices[0], t.vertices[1], t.vertices[2]};
      if (signedTriArea(pts[tri[0]], pts[tri[1]], pts[tri[2]]) < 0) std::swap(tri[1], tri[2]);
      tris.push_back(tri);
    }
    smooth(pts, tris, boundaryCount, 3);

    const auto offset = static_cast<std::uint32_t>(mesh.rest.size());
    loopOffset[pi] = offset;
    for (std::size_t i = 0; i < pts.size(); ++i) {
      mesh.rest.push_back(pts[i]);
      mesh.vertexPiece.push_back(static_cast<std::uint32_t>(pi));
      mesh.boundary.push_back(i < boundaryCount ? 1 : 0);
      mesh.vertexArea.push_back(0.0);
    }
    for (const auto& t : tris) {
      const double area = signedTriArea(pts[t[0]], pts[t[1]], pts[t[2]]);
      if (!(area > 1e-12)) throw MeshError("piece '" + piece.id + "': degenerate triangle");
      mesh.triangles.push_back({t[0] + offset, t[1] + offset, t[2] + offset});
      mesh.trianglePiece.push_back(static_cast<std::uint32_t>(pi));
      for (auto v : t) mesh.vertexArea[v + offset] += area / 3.0;
    }
    mesh.pieceGrain.push_back(grain);
  }

  // Seam pairs: A's i-th sample pairs with B's i-th, each in its traversal direction.
  for (std::size_t s = 0; s < pattern.seams.size(); ++s) {
    const auto& seam = pattern.seams[s];
    const int n = seamSteps[s];
    auto vertexAt = [&](const EdgeRef& e, int k) {
      const std::size_t pi = pieceIndex(e.piece);
      const int kk = e.reversed ? n - k : k;
      const double pos = e.start + (e.end - e.start) * kk / n;
      return loopOffset[pi] + static_cast<std::uint32_t>(loops[pi].indexOf(pos));
    };
    for (int k = 0; k <= n; ++k) {
      mesh.seamPairs.push_back({vertexAt(seam.a, k), vertexAt(seam.b, k), static_cast<std::uint32_t>(s),
                                static_cast<std::uint32_t>(k)});
    }
  }
  return mesh;
}

}  // namespace drape::garment
