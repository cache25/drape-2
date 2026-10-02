#include "drape/core/polygon.hpp"

#include <algorithm>
#include <vector>

namespace drape {

double signedArea(std::span<const Vec2d> poly) {
  double a = 0;
  const std::size_t n = poly.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2d& p = poly[i];
    const Vec2d& q = poly[(i + 1) % n];
    a += p.x() * q.y() - q.x() * p.y();
  }
  return 0.5 * a;
}

bool pointInPolygon(const Vec2d& p, std::span<const Vec2d> poly) {
  bool inside = false;
  const std::size_t n = poly.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const Vec2d& a = poly[i];
    const Vec2d& b = poly[j];
    if ((a.y() > p.y()) != (b.y() > p.y())) {
      const double x = a.x() + (p.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
      if (p.x() < x) inside = !inside;
    }
  }
  return inside;
}

double distancePointSegment(const Vec2d& p, const Vec2d& a, const Vec2d& b) {
  const Vec2d ab = b - a;
  const double len2 = ab.squaredNorm();
  if (len2 <= 0) return (p - a).norm();
  const double t = std::clamp((p - a).dot(ab) / len2, 0.0, 1.0);
  return (p - (a + t * ab)).norm();
}

namespace {
double orient(const Vec2d& a, const Vec2d& b, const Vec2d& c) {
  return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}
bool onSegment(const Vec2d& a, const Vec2d& b, const Vec2d& p) {
  return std::min(a.x(), b.x()) <= p.x() && p.x() <= std::max(a.x(), b.x()) &&
         std::min(a.y(), b.y()) <= p.y() && p.y() <= std::max(a.y(), b.y());
}
int sign(double v) { return (v > 0) - (v < 0); }
}  // namespace

bool segmentsIntersect(const Vec2d& a, const Vec2d& b, const Vec2d& c, const Vec2d& d) {
  const int o1 = sign(orient(a, b, c));
  const int o2 = sign(orient(a, b, d));
  const int o3 = sign(orient(c, d, a));
  const int o4 = sign(orient(c, d, b));
  if (o1 != o2 && o3 != o4) return true;
  if (o1 == 0 && onSegment(a, b, c)) return true;
  if (o2 == 0 && onSegment(a, b, d)) return true;
  if (o3 == 0 && onSegment(c, d, a)) return true;
  if (o4 == 0 && onSegment(c, d, b)) return true;
  return false;
}

bool isSimplePolygon(std::span<const Vec2d> poly) {
  const std::size_t n = poly.size();
  if (n < 4) return true;
  // Sweep and prune on edge x-extents: only edges whose x-ranges overlap are tested.
  std::vector<std::size_t> order(n);
  std::vector<double> minX(n), maxX(n);
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2d& a = poly[i];
    const Vec2d& b = poly[(i + 1) % n];
    minX[i] = std::min(a.x(), b.x());
    maxX[i] = std::max(a.x(), b.x());
    order[i] = i;
  }
  std::sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) { return minX[x] < minX[y]; });
  for (std::size_t oi = 0; oi < n; ++oi) {
    const std::size_t i = order[oi];
    for (std::size_t oj = oi + 1; oj < n && minX[order[oj]] <= maxX[i]; ++oj) {
      const std::size_t j = order[oj];
      const std::size_t d = i > j ? i - j : j - i;
      if (d == 1 || d == n - 1) continue;  // adjacent edges share a vertex
      if (segmentsIntersect(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n])) return false;
    }
  }
  return true;
}

}  // namespace drape
