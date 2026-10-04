#pragma once

#include <Eigen/Dense>

#include "drape/core/collision_field.hpp"

namespace drape::sim {

// Body contact for one particle (spec 5.5): push out to `radius` along the field gradient, then positional
// Coulomb friction against this substep's tangential motion (static if |dt| <= mu d, else scaled back by mu d).
void collideBody(const CollisionField& body, float radius, float friction, const Eigen::Vector3f& xPrev,
                 Eigen::Vector3f& x);

// Last point on the segment from -> to that is at least `radius` from the body (bisection on the field), or `from`
// itself when it is already closer. Used where the field is flat (beyond its band) and gives no push-out direction.
Eigen::Vector3f landOnBody(const CollisionField& body, float radius, const Eigen::Vector3f& from,
                           const Eigen::Vector3f& to);

}  // namespace drape::sim
