#pragma once

#include "drape/core/triangle_mesh.hpp"

namespace drape::avatar {

TriangleMesh icosphere(const Vec3d& center, double radius, int subdivisions);
TriangleMesh capsule(const Vec3d& a, const Vec3d& b, double radius, int segments = 32, int rings = 8);
// Vertical (Y axis) elliptic cylinder from yBottom to yTop with semi-axes rx (X) and rz (Z),
// closed by half-ellipsoid caps of height capHeight.
TriangleMesh ellipticCapsule(double yBottom, double yTop, double rx, double rz, double capHeight, int segments = 48,
                             int rings = 12);

// Oriented box: 8 corners, 12 outward triangles.
TriangleMesh box(const Vec3d& center, const Vec3d& halfExtents, const Mat3d& rotation);

}  // namespace drape::avatar
