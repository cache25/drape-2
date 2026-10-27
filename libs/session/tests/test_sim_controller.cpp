#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "drape/session/position_exchange.hpp"
#include "drape/session/sim_controller.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "drape/sim/drape_runner.hpp"

using namespace drape;
using namespace drape::session;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace {

SimConfig cpuDraft(avatar::BodyType body = avatar::BodyType::Male) {
  SimConfig c;
  c.body = body;
  c.quality = SimQuality::Draft;
  c.backend = sim::SolverBackend::Cpu;
  return c;
}

// Polls `pred` every 2 ms until it holds or `timeout` passes.
template <class Pred>
bool waitFor(Pred pred, std::chrono::milliseconds timeout) {
  const auto end = Clock::now() + timeout;
  while (Clock::now() < end) {
    if (pred()) return true;
    std::this_thread::sleep_for(2ms);
  }
  return pred();
}

// A real CPU solver whose stats always report a non-finite state, so the stability guard gives up.
class AlwaysNonFinite final : public sim::Solver {
 public:
  void build(const sim::SimScene& s) override { inner_.build(s); }
  void updateFabric(const sim::GarmentId& g, const std::vector<FabricPhysical>& f) override { inner_.updateFabric(g, f); }
  void setPhase(const sim::GarmentId& g, sim::SimPhase p) override { inner_.setPhase(g, p); }
  void step(const sim::StepInput&) override {}
  void readPositions(std::span<Vec3f> out) const override { inner_.readPositions(out); }
  sim::SimStats stats() const override {
    sim::SimStats s = inner_.stats();
    s.hasNonFinite = true;
    return s;
  }
  sim::Snapshot snapshot() const override { return inner_.snapshot(); }
  void restore(const sim::Snapshot& s) override { inner_.restore(s); }
  sim::ParticleRange range(const sim::GarmentId& g) const override { return inner_.range(g); }
  std::size_t particleCount() const override { return inner_.particleCount(); }

 private:
  sim::CpuSolver inner_;
};

// A real CPU solver whose build or step throws, standing in for a GPU that fails to allocate or faults mid-drape.
class Faulty final : public sim::Solver {
 public:
  Faulty(bool failBuild, int failAfterSteps) : failBuild_(failBuild), stepsLeft_(failAfterSteps) {}
  void build(const sim::SimScene& s) override {
    if (failBuild_) throw std::runtime_error("out of device memory");
    inner_.build(s);
  }
  void updateFabric(const sim::GarmentId& g, const std::vector<FabricPhysical>& f) override { inner_.updateFabric(g, f); }
  void setPhase(const sim::GarmentId& g, sim::SimPhase p) override { inner_.setPhase(g, p); }
  void step(const sim::StepInput& in) override {
    if (stepsLeft_-- <= 0) throw std::runtime_error("command buffer failed");
    inner_.step(in);
  }
  void readPositions(std::span<Vec3f> out) const override { inner_.readPositions(out); }
  sim::SimStats stats() const override { return inner_.stats(); }
  sim::Snapshot snapshot() const override { return inner_.snapshot(); }
  void restore(const sim::Snapshot& s) override { inner_.restore(s); }
  sim::ParticleRange range(const sim::GarmentId& g) const override { return inner_.range(g); }
  std::size_t particleCount() const override { return inner_.particleCount(); }

 private:
  sim::CpuSolver inner_;
  bool failBuild_;
  int stepsLeft_;
};

SimConfig gpuDraft() {
  SimConfig c = cpuDraft();
  c.backend = sim::SolverBackend::Metal;
  return c;
}

}  // namespace

TEST(PositionExchange, LatestSinceFrame) {
  PositionExchange ex;
  PublishedFrame out;
  EXPECT_FALSE(ex.latest(out, 0));
  PublishedFrame f;
  f.revision = 1;
  f.frame = 1;
  f.positions = {Vec3f(1, 2, 3)};
  ex.publish(std::move(f));
  ASSERT_TRUE(ex.latest(out, 0));
  EXPECT_EQ(out.frame, 1u);
  EXPECT_EQ(out.positions[0], Vec3f(1, 2, 3));
  EXPECT_FALSE(ex.latest(out, 1));
  PublishedFrame g;
  g.revision = 1;
  g.frame = 2;
  g.positions = {Vec3f(4, 5, 6)};
  ex.publish(std::move(g));
  ASSERT_TRUE(ex.latest(out, 1));
  EXPECT_EQ(out.frame, 2u);
  EXPECT_EQ(out.positions[0], Vec3f(4, 5, 6));
}

TEST(SimController, ReachesSettled) {
  SimController c(cpuDraft());
  ASSERT_TRUE(waitFor([&] { return c.status().state == sim::DrapeState::Settled; }, 120s)) << c.status().message;
  PublishedFrame f;
  ASSERT_TRUE(c.exchange().latest(f, 0));
  ASSERT_NE(f.scene, nullptr);
  EXPECT_EQ(f.positions.size(), f.scene->mesh.rest.size());
  EXPECT_EQ(c.status().particles, f.positions.size());
  EXPECT_EQ(c.status().backend, sim::SolverBackend::Cpu);
  EXPECT_GT(c.status().simulatedSeconds, 1.0);
}

TEST(SimController, PauseStopsFrames) {
  SimController c(cpuDraft());
  PublishedFrame f;
  ASSERT_TRUE(waitFor([&] { return c.exchange().latest(f, 3); }, 60s));
  c.setRunning(false);
  // A frame already in flight still publishes; status().running turns false once the thread has gone to sleep.
  ASSERT_TRUE(waitFor([&] { return !c.status().running; }, 30s));
  ASSERT_TRUE(c.exchange().latest(f, 0));
  const auto frozen = f.frame;
  std::this_thread::sleep_for(500ms);
  ASSERT_TRUE(c.exchange().latest(f, 0));
  EXPECT_EQ(f.frame, frozen);
  EXPECT_FALSE(c.status().running);
  c.setRunning(true);
  EXPECT_TRUE(waitFor([&] { return c.exchange().latest(f, frozen); }, 10s));
}

TEST(SimController, RapidRebuildsNeverMixRevisions) {
  SimController c(cpuDraft());
  SimConfig last = cpuDraft();
  bool mixed = false;
  PublishedFrame f;
  std::uint64_t seen = 0;
  auto poll = [&] {
    if (c.exchange().latest(f, seen)) {
      seen = f.frame;
      if (!f.scene || f.positions.size() != f.scene->mesh.rest.size()) mixed = true;
    }
  };
  for (int i = 0; i < 20; ++i) {
    if (i % 2 == 0) {
      last.body = last.body == avatar::BodyType::Male ? avatar::BodyType::Female : avatar::BodyType::Male;
    } else {
      last.quality = last.quality == SimQuality::Draft ? SimQuality::Standard : SimQuality::Draft;
    }
    c.configure(last);
    for (int k = 0; k < 5; ++k) {
      poll();
      std::this_thread::sleep_for(1ms);
    }
  }
  const bool reached = waitFor(
      [&] {
        poll();
        return f.scene && f.scene->bodyType == last.body && f.scene->quality == last.quality;
      },
      120s);
  EXPECT_TRUE(reached);
  EXPECT_FALSE(mixed);
}

TEST(SimController, DestroyWhileAssemblingJoinsQuickly) {
  auto c = std::make_unique<SimController>(cpuDraft());
  ASSERT_TRUE(waitFor([&] { return c->status().state == sim::DrapeState::Assembling && c->status().particles > 0; }, 60s));
  const auto start = Clock::now();
  c.reset();
  EXPECT_LT(Clock::now() - start, 1s);
}

TEST(SimController, FailureStopsAndReportsMessage) {
  SimController c(cpuDraft(), [](sim::SolverBackend) { return std::make_unique<AlwaysNonFinite>(); });
  ASSERT_TRUE(waitFor([&] { return c.status().state == sim::DrapeState::Failed; }, 60s));
  EXPECT_EQ(c.status().message, sim::kDrapeFailureMessage);
  PublishedFrame f;
  ASSERT_TRUE(c.exchange().latest(f, 0));
  const auto frozen = f.frame;
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(c.exchange().latest(f, 0));
  EXPECT_EQ(f.frame, frozen);
}

TEST(SimController, FallbackNoteSurvivesCpuRebuild) {
  SimConfig cfg = cpuDraft();
  cfg.backend = sim::SolverBackend::Metal;
  auto factory = [](sim::SolverBackend b) -> std::unique_ptr<sim::Solver> {
    if (b != sim::SolverBackend::Cpu) throw std::runtime_error("no device");
    return std::make_unique<sim::CpuSolver>();
  };
  SimController c(cfg, factory);
  PublishedFrame f;
  ASSERT_TRUE(waitFor([&] { return c.exchange().latest(f, 0) && f.revision == 1; }, 60s));
  c.configure(cpuDraft());  // the app switches its Solver menu to CPU once the GPU fails
  ASSERT_TRUE(waitFor([&] { return c.exchange().latest(f, 0) && f.revision == 2; }, 60s));
  const SimStatus s = c.status();
  EXPECT_EQ(s.backend, sim::SolverBackend::Cpu);
  EXPECT_FALSE(s.gpuAvailable);
  EXPECT_EQ(s.message, "GPU unavailable: no device; using CPU");
}

#if !defined(__APPLE__)
TEST(SimController, FallsBackToCpuWhenGpuUnavailable) {
  SimConfig cfg = cpuDraft();
  cfg.backend = sim::SolverBackend::Metal;
  SimController c(cfg);
  ASSERT_TRUE(waitFor([&] { return c.status().particles > 0; }, 60s));
  const SimStatus s = c.status();
  EXPECT_EQ(s.backend, sim::SolverBackend::Cpu);
  EXPECT_FALSE(s.gpuAvailable);
  EXPECT_EQ(s.message.rfind("GPU unavailable", 0), 0u) << s.message;
}
#endif

TEST(SimController, GpuBuildFailureFallsBackToCpu) {
  SimController c(gpuDraft(), [](sim::SolverBackend b) -> std::unique_ptr<sim::Solver> {
    if (b != sim::SolverBackend::Cpu) return std::make_unique<Faulty>(true, 0);
    return std::make_unique<sim::CpuSolver>();
  });
  PublishedFrame f;
  ASSERT_TRUE(waitFor([&] { return c.exchange().latest(f, 0); }, 60s));
  const SimStatus s = c.status();
  EXPECT_EQ(s.backend, sim::SolverBackend::Cpu);
  EXPECT_FALSE(s.gpuAvailable);
  EXPECT_EQ(s.message, "GPU unavailable: out of device memory; using CPU");
}

TEST(SimController, GpuFaultMidDrapeRedoesOnCpu) {
  SimController c(gpuDraft(), [](sim::SolverBackend b) -> std::unique_ptr<sim::Solver> {
    if (b != sim::SolverBackend::Cpu) return std::make_unique<Faulty>(false, 3);
    return std::make_unique<sim::CpuSolver>();
  });
  PublishedFrame f;
  ASSERT_TRUE(waitFor([&] { return c.exchange().latest(f, 0) && f.revision == 2; }, 60s));
  SimStatus s = c.status();
  EXPECT_EQ(s.backend, sim::SolverBackend::Cpu);
  EXPECT_FALSE(s.gpuAvailable);
  EXPECT_EQ(s.message, "GPU failed: command buffer failed; using CPU");
  c.reset();  // later drapes stay on the CPU
  ASSERT_TRUE(waitFor([&] { return c.exchange().latest(f, 0) && f.revision == 3; }, 60s));
  s = c.status();
  EXPECT_EQ(s.backend, sim::SolverBackend::Cpu);
  EXPECT_EQ(s.message, "GPU failed: command buffer failed; using CPU");
}

TEST(SimController, CpuFaultFailsWithMessage) {
  SimController c(cpuDraft(), [](sim::SolverBackend) { return std::make_unique<Faulty>(false, 2); });
  ASSERT_TRUE(waitFor([&] { return c.status().state == sim::DrapeState::Failed; }, 60s));
  EXPECT_EQ(c.status().message, "command buffer failed");
  EXPECT_TRUE(waitFor([&] { return !c.status().running; }, 10s));
  EXPECT_EQ(c.status().fps, 0);
}
