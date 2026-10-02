#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "drape/core/bezier.hpp"

namespace drape {

// Closed piece outline made of named cubic segments. Positions along it are arc length (m)
// measured from the start of the first segment.
class Outline {
 public:
  void add(std::string name, const CubicBezier& segment);

  std::size_t size() const { return segments_.size(); }
  const CubicBezier& segment(std::size_t i) const { return segments_[i]; }
  const std::string& name(std::size_t i) const { return names_[i]; }
  std::optional<std::size_t> find(std::string_view name) const;

  bool isClosed(double tol = 1e-9) const;
  double perimeter() const { return starts_.empty() ? 0.0 : starts_.back(); }
  double segmentStart(std::size_t i) const { return starts_[i]; }
  double segmentEnd(std::size_t i) const { return starts_[i + 1]; }
  Vec2d pointAt(double s) const;    // s clamped to [0, perimeter]; perimeter -> start point
  Vec2d tangentAt(double s) const;  // unit
  std::vector<Vec2d> polyline(double maxSpacing) const;

  bool operator==(const Outline& o) const { return segments_ == o.segments_ && names_ == o.names_; }

 private:
  std::size_t locate(double s, double& localT) const;

  std::vector<CubicBezier> segments_;
  std::vector<std::string> names_;
  std::vector<ArcLengthTable> tables_;
  std::vector<double> starts_{0.0};  // starts_[i] = arc length at segment i; last = perimeter
};

}  // namespace drape
