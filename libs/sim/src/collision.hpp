#pragma once

#include <Eigen/Dense>

#include "drape/core/collision_field.hpp"

namespace drape::sim {

// Body contact for one particle (spec 5.5): push out to `radius` along the field gradient, then positional
// Coulomb friction against this substep's tangential motion (static if |dt| <= mu d, else scaled back by mu d).
void collideBody(const CollisionField& body, float radius, float friction, const Eigen::Vector3f& xPrev,
                 Eigen::Vector3f& x);

}  // namespace drape::sim
