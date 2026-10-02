#include "drape/core/outline.hpp"

#include <algorithm>
#include <cmath>

namespace drape {

void Outline::add(std::string name, const CubicBezier& segment) {
  segments_.push_back(segment);
  names_.push_back(std::move(name));
  tables_.emplace_back(segment);
  starts_.push_back(starts_.back() + tables_.back().length());
}

std::optional<std::size_t> Outline::find(std::string_view name) const {
  for (std::size_t i = 0; i < names_.size(); ++i) {
    if (names_[i] == name) return i;
  }
  return std::nullopt;
}

bool Outline::isClosed(double tol) const {
  if (segments_.empty()) return false;
  for (std::size_t i = 0; i < segments_.size(); ++i) {
    const auto& next = segments_[(i + 1) % segments_.size()];
    if ((segments_[i].p3 - next.p0).norm() > tol) return false;
  }
  return true;
}

std::size_t Outline::locate(double s, double& localT) const {
  s = std::clamp(s, 0.0, perimeter());
  if (s >= perimeter()) {
    localT = 0.0;
    return 0;  // wraps to the start point
  }
  const auto it = std::upper_bound(starts_.begin(), starts_.end(), s);
  const std::size_t i = static_cast<std::size_t>(it - starts_.begin()) - 1;
  localT = tables_[i].tAtLength(s - starts_[i]);
  return i;
}

Vec2d Outline::pointAt(double s) const {
  double t = 0;
  const std::size_t i = locate(s, t);
  return segments_[i].point(t);
}

Vec2d Outline::tangentAt(double s) const {
  double t = 0;
  const std::size_t i = locate(s, t);
  Vec2d d = segments_[i].derivative(t);
  if (d.norm() < 1e-14) d = segments_[i].p3 - segments_[i].p0;
  return d.normalized();
}

std::vector<Vec2d> Outline::polyline(double maxSpacing) const {
  std::vector<Vec2d> out;
  for (std::size_t i = 0; i < segments_.size(); ++i) {
    const double len = tables_[i].length();
    const int n = std::max(1, static_cast<int>(std::ceil(len / maxSpacing)));
    for (int k = 0; k < n; ++k) {
      out.push_back(segments_[i].point(tables_[i].tAtLength(len * k / n)));
    }
  }
  return out;
}

}  // namespace drape
