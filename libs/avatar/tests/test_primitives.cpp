#include <gtest/gtest.h>

#include <cmath>

#include "drape/avatar/primitives.hpp"
#include "drape/avatar/test_body.hpp"

using namespace drape;
using namespace drape::avatar;

TEST(Primitives, AllAreClosedAndOutward) {
  for (const TriangleMesh& m : {icosphere({0, 0, 0}, 0.1, 3), capsule({0, 0, 0}, {0, 0.3, 0.1}, 0.05),
                                ellipticCapsule(0.85, 1.38, 0.17, 0.11, 0.08)}) {
    EXPECT_TRUE(isClosedManifold(m));
    EXPECT_GT(signedVolume(m), 0.0);
  }
}

TEST(Primitives, OpenMeshIsNotClosed) {
  TriangleMesh m = icosphere({0, 0, 0}, 0.1, 1);
  m.triangles.pop_back();
  EXPECT_FALSE(isClosedManifold(m));
}

TEST(Primitives, VolumesMatchAnalytic) {
  const double r = 0.1;
  EXPECT_NEAR(signedVolume(icosphere({0, 0, 0}, r, 4)) / (4.0 / 3.0 * kPi * r * r * r), 1.0, 0.005);
  const double cr = 0.05, len = std::sqrt(0.3 * 0.3 + 0.1 * 0.1);
  const double capsuleVol = kPi * cr * cr * len + 4.0 / 3.0 * kPi * cr * cr * cr;
  EXPECT_NEAR(signedVolume(capsule({0, 0, 0}, {0, 0.3, 0.1}, cr)) / capsuleVol, 1.0, 0.01);
  const double ecVol = kPi * 0.17 * 0.11 * (1.38 - 0.85) + 4.0 / 3.0 * kPi * 0.17 * 0.11 * 0.08;
  EXPECT_NEAR(signedVolume(ellipticCapsule(0.85, 1.38, 0.17, 0.11, 0.08)) / ecVol, 1.0, 0.01);
}

TEST(TestBody, ComponentsAndRegions) {
  drape::avatar::TestBody b = makeTestBody();
  ASSERT_EQ(b.components.size(), 5u);
  for (const auto& c : b.components) EXPECT_TRUE(isClosedManifold(c));
  ASSERT_EQ(b.regions.cylinders.size(), 4u);
  const double s = std::sqrt(0.5);
  const auto& torso = b.regions.cylinders.at(BodyRegion::Torso);
  EXPECT_TRUE(torso.origin.isApprox(Vec3d(0, 1.47, 0)));
  EXPECT_DOUBLE_EQ(torso.radius, 0.17);
  EXPECT_DOUBLE_EQ(b.regions.cylinders.at(BodyRegion::Neck).radius, 0.055);
  const auto& armL = b.regions.cylinders.at(BodyRegion::ArmLeft);
  EXPECT_TRUE(armL.origin.isApprox(Vec3d(0.19, 1.37, 0)));
  EXPECT_TRUE(armL.axis.isApprox(Vec3d(s, -s, 0)));
  EXPECT_TRUE(armL.e0.isApprox(Vec3d(s, s, 0)));
  EXPECT_DOUBLE_EQ(armL.radius, 0.045);
  const auto& armR = b.regions.cylinders.at(BodyRegion::ArmRight);
  EXPECT_TRUE(armR.origin.isApprox(Vec3d(-0.19, 1.37, 0)));
  EXPECT_TRUE(armR.axis.isApprox(Vec3d(-s, -s, 0)));
  EXPECT_TRUE(armR.e0.isApprox(Vec3d(-s, s, 0)));
  for (const auto& [region, c] : b.regions.cylinders) {
    EXPECT_NEAR(c.e1().norm(), 1.0, 1e-12);
    EXPECT_NEAR(c.e1().dot(c.axis), 0.0, 1e-12);
    EXPECT_NEAR(c.e1().dot(c.e0), 0.0, 1e-12);
  }
}

TEST(Primitives, BoxIsClosed) {
  const Mat3d rot = Eigen::AngleAxisd(0.3, Vec3d(0, 0, 1)).toRotationMatrix();
  const TriangleMesh b = box({0.1, 0.2, 0.3}, {0.5, 0.05, 0.25}, rot);
  EXPECT_EQ(b.triangles.size(), 12u);
  EXPECT_TRUE(isClosedManifold(b));
  EXPECT_NEAR(signedVolume(b), 8 * 0.5 * 0.05 * 0.25, 1e-12);
}
