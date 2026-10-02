#pragma once

#include <span>

#include "drape/core/math.hpp"

namespace drape {

// All polylines are closed: the last point connects back to the first (not repeated).
double signedArea(std::span<const Vec2d> closedPolyline);
bool pointInPolygon(const Vec2d& p, std::span<const Vec2d> closedPolyline);
double distancePointSegment(const Vec2d& p, const Vec2d& a, const Vec2d& b);
bool segmentsIntersect(const Vec2d& a, const Vec2d& b, const Vec2d& c, const Vec2d& d);  // includes touching
bool isSimplePolygon(std::span<const Vec2d> closedPolyline);  // no intersections between non-adjacent edges

}  // namespace drape
