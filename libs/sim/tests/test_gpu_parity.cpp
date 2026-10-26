#include <cstdio>
#include <random>
#include <string>

#include "gpu_parity_helpers.hpp"
#include "sphere_scene.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

// Spec 12.2 test 5, short runs: maximum per-particle difference < 1 mm against the CPU reference.
//
// A 1 mm comparison only means something on a scene the reference itself reproduces to well under 1 mm. Impacts that
// buckle cloth into folds do not: dropped onto the sphere, a sheet's CPU result moves by ~10 cm within 60 frames when
// its start moves by 1 nm, and the tee's assembly by ~1 cm within 2 frames (contact decisions at the body surface).
// The scenes below are ones the CPU reproduces to under 1 mm when its start moves by 1e-6 m (measured 2026-10-26:
// patch 0.21 mm, settled tee 0.47 mm); each test prints that self-spread next to the GPU difference. (The flat hanging
// sheet is the exception that proves the rule: an out-of-plane nudge buckles it, so its self-spread is large while a
// faithful port stays bit-close; there the print is informational.)

using GpuParity = GpuBackendTest;
DRAPE_INSTANTIATE_GPU_BACKENDS(GpuParity);

namespace {

// Max per-particle distance after `frames` frames between `a` and `b` built from `scene` (optionally continued from
// `start`), with b's start shifted by up to `perturb` metres.
double framesApart(SolverBackend a, SolverBackend b, const SimScene& scene, int frames, double perturb = 0,
                   const Snapshot* start = nullptr) {
  const auto sa = makeSolver(a);
  const auto sb = makeSolver(b);
  sa->build(scene);
  sb->build(scene);
  if (start) {
    sa->restore(*start);
    sb->restore(*start);
  }
  if (perturb > 0) {
    Snapshot s = sb->snapshot();
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> u(-1, 1);
    for (auto& x : s.positions) x += (perturb * Vec3d(u(rng), u(rng), u(rng))).cast<float>();
    sb->restore(s);
  }
  for (int f = 0; f < frames; ++f) {
    sa->step({1.0 / 60.0, 1, 2});
    sb->step({1.0 / 60.0, 1, 2});
  }
  EXPECT_FALSE(sb->stats().hasNonFinite);
  return maxDistance(positions(*sa), positions(*sb));
}

void expectParity(SolverBackend gpu, const SimScene& scene, const Snapshot* start = nullptr) {
  const double self = framesApart(SolverBackend::Cpu, SolverBackend::Cpu, scene, 60, 1e-6, start);
  const double apart = framesApart(SolverBackend::Cpu, gpu, scene, 60, 0, start);
  std::printf("parity: cpu self-spread %.3f mm, %s vs cpu %.3f mm\n", self * 1000, std::string(backendName(gpu)).c_str(),
              apart * 1000);
  EXPECT_LT(apart, 0.001);
}

}  // namespace

TEST_P(GpuParity, HangingSheet60Frames) { expectParity(GetParam(), hangingSheet().scene); }

TEST_P(GpuParity, PatchOnSphere60Frames) {
  SimScene scene;
  scene.body = &sphereField();
  scene.garments.push_back(centredSheet(0.004, 0.004, 0.2));
  expectParity(GetParam(), scene);
}

// The full tee, settled on the male mannequin by the CPU, continued 60 frames on both backends: every term, seams,
// body contact and friction at once.
TEST_P(GpuParity, TeeFromSettled60Frames) {
  const auto ts = session::makeTeeScene(avatar::BodyType::Male, SimQuality::Draft);
  const auto cpu = makeSolver(SolverBackend::Cpu);
  cpu->build(ts->scene);
  ASSERT_TRUE(drapeToRest(*cpu, {"tee"}, draftOptions()).settled);
  const Snapshot settled = cpu->snapshot();
  expectParity(GetParam(), ts->scene, &settled);
}

using GpuDeterminism = GpuBackendTest;
DRAPE_INSTANTIATE_GPU_BACKENDS(GpuDeterminism);

TEST_P(GpuDeterminism, TwoRunsBitIdentical) {
  SimScene scene;
  scene.body = &sphereField();
  scene.garments.push_back(centredSheet(0.2, 0.02, 0.17));
  std::vector<Vec3f> runs[2];
  for (auto& run : runs) {
    const auto s = makeSolver(GetParam());
    s->build(scene);
    for (int f = 0; f < 60; ++f) s->step({1.0 / 60.0, 1, 2});
    run = positions(*s);
  }
  EXPECT_EQ(runs[0], runs[1]);
}

TEST_P(GpuDeterminism, SnapshotReplayBitIdentical) {
  SimScene scene;
  scene.body = &sphereField();
  scene.garments.push_back(centredSheet(0.2, 0.02, 0.17));
  const auto s = makeSolver(GetParam());
  s->build(scene);
  for (int f = 0; f < 60; ++f) s->step({1.0 / 60.0, 1, 2});
  const Snapshot snap = s->snapshot();
  for (int f = 0; f < 20; ++f) s->step({1.0 / 60.0, 1, 2});
  const auto first = positions(*s);
  s->restore(snap);
  for (int f = 0; f < 20; ++f) s->step({1.0 / 60.0, 1, 2});
  EXPECT_EQ(first, positions(*s));
}
