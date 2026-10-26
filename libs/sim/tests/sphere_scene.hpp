#pragma once

#include <vector>

#include "drape/avatar/collision_bake.hpp"
#include "drape/avatar/primitives.hpp"
#include "drape/core/test_fabrics.hpp"
#include "sim_test_utils.hpp"

namespace drape::sim::testing {

// A 15 cm sphere's collision field, baked once per test binary.
inline const CollisionField& sphereField() {
  static const CollisionField f = [] {
    std::vector<TriangleMesh> c{avatar::icosphere({0, 0, 0}, 0.15, 4)};
    return avatar::bakeCollisionField(c);
  }();
  return f;
}

inline constexpr double kContact = 0.003 + 0.0006 / 2;  // collision offset + half testJersey thickness

inline SimGarment centredSheet(double size, double h, double height) {
  return flatGarment("sheet", garment::rectanglePiece("sheet", size, size), h, testdata::testJersey(),
                     {-size / 2, height, -size / 2});
}

}  // namespace drape::sim::testing
