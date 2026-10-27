#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>

#include "gpu_parity_helpers.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

// Spec 12.2 test 5, long runs: the tee draped to rest on each backend; hem height and center of mass agree within
// 2 mm, or within 1.5x the CPU reference's own spread when that is larger (Ian, 2026-10-26; revised 2026-10-27 from
// max-of-3 to 1.5x max-of-10, because max-of-3 failed a sound run about one time in four). The spread is the largest
// difference from the unperturbed CPU result over ten CPU runs whose start moves by up to 1 nm: the tee's assembly
// makes discrete contact decisions at the body surface, so its settled hem varies by millimetres on its own (female:
// up to 6.7 mm hem, 3.4 mm center of mass over ten runs, measured 2026-10-26).

constexpr int kNudgedRuns = 10;
constexpr double kSpreadFactor = 1.5;
constexpr double kFloor = 0.002;

using GpuParitySlow = GpuBackendTest;
DRAPE_INSTANTIATE_GPU_BACKENDS(GpuParitySlow);

namespace {

struct Settled {
  bool settled = false;
  double seconds = 0, hem = 0;
  Vec3d com = Vec3d::Zero();
};

Settled drapeOnce(const session::TeeScene& ts, SolverBackend backend, unsigned perturbSeed) {
  SimScene scene = ts.scene;
  if (perturbSeed != 0) {
    std::mt19937 rng(perturbSeed);
    std::uniform_real_distribution<double> u(-1, 1);
    for (auto& x : scene.garments[0].initialPositions) x += 1e-9 * Vec3d(u(rng), u(rng), u(rng));
  }
  const auto s = makeSolver(backend);
  s->build(scene);
  const DrapeResult r = drapeToRest(*s, {"tee"}, draftOptions());
  const auto x = positions(*s);
  return {r.settled, r.simulatedSeconds, hemHeight(ts, x), centerOfMass(x)};
}

void expectSettledAlike(SolverBackend backend, avatar::BodyType body) {
  const auto ts = session::makeTeeScene(body, SimQuality::Draft);
  const Settled cpu = drapeOnce(*ts, SolverBackend::Cpu, 0);
  const Settled gpu = drapeOnce(*ts, backend, 0);
  double spreadHem = 0, spreadCom = 0;
  for (unsigned seed = 1; seed <= kNudgedRuns; ++seed) {
    const Settled nudged = drapeOnce(*ts, SolverBackend::Cpu, seed);
    EXPECT_TRUE(nudged.settled);
    spreadHem = std::max(spreadHem, std::abs(nudged.hem - cpu.hem));
    spreadCom = std::max(spreadCom, (nudged.com - cpu.com).norm());
  }
  const double tolHem = std::max(kFloor, kSpreadFactor * spreadHem);
  const double tolCom = std::max(kFloor, kSpreadFactor * spreadCom);
  std::printf("parity %s %s: cpu t=%.2fs gpu t=%.2fs | hem diff %.2f mm (tol %.2f) | com diff %.2f mm (tol %.2f)\n",
              std::string(avatar::bodyName(body)).c_str(), std::string(backendName(backend)).c_str(), cpu.seconds,
              gpu.seconds, std::abs(gpu.hem - cpu.hem) * 1000, tolHem * 1000, (gpu.com - cpu.com).norm() * 1000,
              tolCom * 1000);
  EXPECT_TRUE(cpu.settled);
  EXPECT_TRUE(gpu.settled);
  EXPECT_LE(std::abs(gpu.hem - cpu.hem), tolHem);
  EXPECT_LE((gpu.com - cpu.com).norm(), tolCom);
}

}  // namespace

TEST_P(GpuParitySlow, TeeSettledMale) { expectSettledAlike(GetParam(), avatar::BodyType::Male); }

TEST_P(GpuParitySlow, TeeSettledFemale) { expectSettledAlike(GetParam(), avatar::BodyType::Female); }
