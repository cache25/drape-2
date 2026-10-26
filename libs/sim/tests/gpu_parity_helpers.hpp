#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "backends.hpp"
#include "drape/session/tee_scene.hpp"
#include "drape/sim/drape_session.hpp"
#include "sim_test_utils.hpp"

namespace drape::sim::testing {

// GPU backends compared against the CPU reference.
class GpuBackendTest : public ::testing::TestWithParam<SolverBackend> {};

#define DRAPE_INSTANTIATE_GPU_BACKENDS(suite)                                                                   \
  GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(suite);                                                         \
  INSTANTIATE_TEST_SUITE_P(Gpu, suite, ::testing::ValuesIn(::drape::sim::testing::gpuBackends()), \
                           ::drape::sim::testing::backendTestName)

inline double maxDistance(const std::vector<Vec3f>& a, const std::vector<Vec3f>& b) {
  EXPECT_EQ(a.size(), b.size());
  double m = 0;
  for (std::size_t i = 0; i < std::min(a.size(), b.size()); ++i)
    m = std::max(m, static_cast<double>((a[i] - b[i]).norm()));
  return m;
}

// Mean height of the tee's hem: front and back vertices on the pattern's y = 0 edge (as in expectTeeDrapes).
inline double hemHeight(const session::TeeScene& ts, const std::vector<Vec3f>& x) {
  double hem = 0;
  int count = 0;
  for (std::uint32_t v = 0; v < ts.mesh.rest.size(); ++v) {
    const auto& piece = ts.pattern.pieces[ts.mesh.vertexPiece[v]].id;
    if ((piece == "front" || piece == "back") && ts.mesh.rest[v].y() < 1e-9) {
      hem += x[v].y();
      ++count;
    }
  }
  return count ? hem / count : 0.0;
}

inline Vec3d centerOfMass(const std::vector<Vec3f>& x) {
  Vec3d c = Vec3d::Zero();
  for (const auto& p : x) c += p.cast<double>();
  return c / static_cast<double>(x.size());
}

inline DrapeOptions draftOptions() {
  DrapeOptions o;
  o.substeps = substepsFor(SimQuality::Draft);
  o.iterations = iterationsFor(SimQuality::Draft);
  return o;
}

}  // namespace drape::sim::testing
