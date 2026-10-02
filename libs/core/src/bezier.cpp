#include "drape/core/bezier.hpp"

#include <algorithm>
#include <array>

namespace drape {

Vec2d CubicBezier::point(double t) const {
  const double u = 1.0 - t;
  return u * u * u * p0 + 3 * u * u * t * p1 + 3 * u * t * t * p2 + t * t * t * p3;
}

Vec2d CubicBezier::derivative(double t) const {
  const double u = 1.0 - t;
  return 3 * u * u * (p1 - p0) + 6 * u * t * (p2 - p1) + 3 * t * t * (p3 - p2);
}

CubicBezier CubicBezier::reversed() const { return {p3, p2, p1, p0}; }

CubicBezier CubicBezier::mirroredX() const {
  auto m = [](const Vec2d& p) { return Vec2d(-p.x(), p.y()); };
  return {m(p0), m(p1), m(p2), m(p3)};
}

CubicBezier CubicBezier::line(const Vec2d& a, const Vec2d& b) {
  return {a, a + (b - a) / 3.0, a + 2.0 * (b - a) / 3.0, b};
}

namespace {
constexpr std::array<double, 5> kGaussX{-0.9061798459386640, -0.5384693101056831, 0.0,
                                        0.5384693101056831, 0.9061798459386640};
constexpr std::array<double, 5> kGaussW{0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
                                        0.4786286704993665, 0.2369268850561891};

double speedIntegral(const CubicBezier& c, double a, double b) {
  const double half = 0.5 * (b - a);
  const double mid = 0.5 * (a + b);
  double sum = 0;
  for (std::size_t i = 0; i < kGaussX.size(); ++i) {
    sum += kGaussW[i] * c.derivative(mid + half * kGaussX[i]).norm();
  }
  return sum * half;
}
}  // namespace

ArcLengthTable::ArcLengthTable(const CubicBezier& c, int samples) {
  samples = std::max(samples, 1);
  cumulative_.resize(static_cast<std::size_t>(samples) + 1);
  cumulative_[0] = 0;
  for (int i = 0; i < samples; ++i) {
    const double a = static_cast<double>(i) / samples;
    const double b = static_cast<double>(i + 1) / samples;
    cumulative_[static_cast<std::size_t>(i) + 1] = cumulative_[static_cast<std::size_t>(i)] + speedIntegral(c, a, b);
  }
}

double ArcLengthTable::tAtLength(double s) const {
  const double total = length();
  if (total <= 0) return 0;
  s = std::clamp(s, 0.0, total);
  const auto it = std::upper_bound(cumulative_.begin(), cumulative_.end(), s);
  if (it == cumulative_.end()) return 1.0;
  const std::size_t i = static_cast<std::size_t>(it - cumulative_.begin()) - 1;
  const double s0 = cumulative_[i];
  const double s1 = cumulative_[i + 1];
  const double n = static_cast<double>(cumulative_.size() - 1);
  const double frac = s1 > s0 ? (s - s0) / (s1 - s0) : 0.0;
  return (static_cast<double>(i) + frac) / n;
}

double length(const CubicBezier& c) { return ArcLengthTable(c).length(); }

}  // namespace drape
