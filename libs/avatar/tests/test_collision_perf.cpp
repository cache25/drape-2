#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <span>

#include "drape/avatar/collision_bake.hpp"
#include "drape/avatar/mannequin.hpp"
#include "drape/avatar/test_body.hpp"

TEST(CollisionField, BakeTestBodyUnderTwoSeconds) {
  const auto body = drape::avatar::makeTestBody();
  const auto t0 = std::chrono::steady_clock::now();
  const auto field = drape::avatar::bakeCollisionField(body.components);
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_LT(seconds, 2.0);
  EXPECT_LT(field.sample({0, 1.2, 0}), 0.0);  // inside the torso
}

TEST(CollisionField, BakeMannequinUnderTwoSeconds) {
  const auto mannequin = drape::avatar::loadMannequin(drape::avatar::BodyType::Male);
  const auto t0 = std::chrono::steady_clock::now();
  const auto field = drape::avatar::bakeCollisionField(std::span<const drape::TriangleMesh>(&mannequin.mesh, 1));
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::cout << "mannequin bake: " << seconds << " s\n";
  EXPECT_LT(seconds, 2.0);
  const auto& torso = mannequin.regions.cylinders.at(drape::BodyRegion::Torso);
  EXPECT_LT(field.sample(torso.origin - drape::Vec3d(0, 0.3, 0)), 0.0);  // inside the torso
}
