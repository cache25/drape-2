#pragma once

#include <vector>

#include "drape/core/math.hpp"

namespace drape {

// Cubic Bézier segment in pattern space (meters).
struct CubicBezier {
  Vec2d p0{0, 0}, p1{0, 0}, p2{0, 0}, p3{0, 0};

  Vec2d point(double t) const;
  Vec2d derivative(double t) const;
  CubicBezier reversed() const;   // point(t) == original.point(1 - t)
  CubicBezier mirroredX() const;  // (x, y) -> (-x, y) on every control point
  static CubicBezier line(const Vec2d& a, const Vec2d& b);  // controls at 1/3 and 2/3

  bool operator==(const CubicBezier&) const = default;
};

// Cumulative arc length at `samples` evenly spaced parameter values,
// integrated with 5-point Gauss-Legendre per interval.
class ArcLengthTable {
 public:
  explicit ArcLengthTable(const CubicBezier& c, int samples = 256);
  double length() const { return cumulative_.back(); }
  // Parameter t whose arc length from t = 0 is s (s clamped to [0, length]).
  double tAtLength(double s) const;

 private:
  std::vector<double> cumulative_;
};

double length(const CubicBezier& c);

}  // namespace drape
