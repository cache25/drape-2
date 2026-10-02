#include <gtest/gtest.h>

#include <chrono>

#include "drape/avatar/collision_bake.hpp"
#include "drape/avatar/test_body.hpp"

TEST(CollisionField, BakeTestBodyUnderTwoSeconds) {
  const auto body = drape::avatar::makeTestBody();
  const auto t0 = std::chrono::steady_clock::now();
  const auto field = drape::avatar::bakeCollisionField(body.components);
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_LT(seconds, 2.0);
  EXPECT_LT(field.sample({0, 1.2, 0}), 0.0);  // inside the torso
}
