#include "drape/garment/validate.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "drape/core/polygon.hpp"

namespace drape::garment {

namespace {

constexpr double kEps = 1e-9;
constexpr double kSeamLengthTolerance = 0.002;

struct Interval {
  double start;
  double end;
  std::string seam;
};

}  // namespace

std::vector<ValidationIssue> validatePattern(const Pattern& pattern) {
  std::vector<ValidationIssue> issues;
  auto add = [&](const std::string& rule, const std::string& message) { issues.push_back({rule, message}); };

  for (const auto& piece : pattern.pieces) {
    const std::string who = "piece '" + piece.id + "'";
    if (!piece.outline.isClosed()) {
      add("outline-closed", who + ": outline is not closed");
    } else {
      const auto poly = piece.outline.polyline(0.002);
      if (!isSimplePolygon(poly)) {
        add("outline-simple", who + ": outline crosses itself");
      } else if (signedArea(poly) <= 0) {
        add("outline-ccw", who + ": outline must run counter-clockwise");
      }
    }
    if (piece.grainline.norm() < kEps) add("grainline", who + ": grainline is zero");
    if (piece.fabricSlot.empty()) add("fabric-slot", who + ": fabric slot is empty");
  }

  std::map<Id, std::vector<Interval>> intervals;
  for (const auto& seam : pattern.seams) {
    const std::string who = "seam '" + seam.id + "'";
    bool rangesOk = true;
    for (const EdgeRef* e : {&seam.a, &seam.b}) {
      const PatternPiece* piece = pattern.findPiece(e->piece);
      if (!piece) {
        add("seam-piece", who + ": unknown piece '" + e->piece + "'");
        rangesOk = false;
        continue;
      }
      const double perimeter = piece->outline.perimeter();
      if (!(e->start >= 0 && e->start < e->end && e->end <= perimeter + kEps)) {
        add("seam-range", who + ": edge on piece '" + e->piece + "' is outside [0, perimeter] or empty");
        rangesOk = false;
        continue;
      }
      intervals[e->piece].push_back({e->start, e->end, seam.id});
    }
    if (seam.a.reversed == seam.b.reversed) {
      add("seam-twisted", who + ": edges must be joined with opposite traversal (one reversed)");
    }
    if (rangesOk) {
      const double lenA = seam.a.end - seam.a.start;
      const double lenB = seam.b.end - seam.b.start;
      if (std::abs(lenA - lenB) > kSeamLengthTolerance + std::abs(seam.ease)) {
        add("seam-length", who + ": edge lengths differ by " + std::to_string(std::abs(lenA - lenB) * 1000) +
                               " mm (allowed " + std::to_string((kSeamLengthTolerance + std::abs(seam.ease)) * 1000) +
                               " mm)");
      }
    }
  }

  for (auto& [pieceId, list] : intervals) {
    std::sort(list.begin(), list.end(), [](const Interval& x, const Interval& y) { return x.start < y.start; });
    for (std::size_t i = 1; i < list.size(); ++i) {
      if (list[i].start < list[i - 1].end - kEps) {
        add("seam-overlap", "piece '" + pieceId + "': seams '" + list[i - 1].seam + "' and '" + list[i].seam +
                                "' overlap");
      }
    }
  }
  return issues;
}

}  // namespace drape::garment
