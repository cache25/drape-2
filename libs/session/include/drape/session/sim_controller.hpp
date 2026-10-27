#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "drape/session/position_exchange.hpp"
#include "drape/session/tee_scene.hpp"
#include "drape/sim/drape_session.hpp"
#include "drape/sim/solver_factory.hpp"

namespace drape::session {

struct SimConfig {
  avatar::BodyType body = avatar::BodyType::Male;
  SimQuality quality = SimQuality::Standard;
  sim::SolverBackend backend = sim::SolverBackend::Metal;
  std::string size = "M";
  std::optional<std::filesystem::path> mannequinDir;
};

struct SimStatus {
  sim::DrapeState state = sim::DrapeState::Assembling;
  bool running = true;  // the thread is simulating (false once it sleeps: paused, finished or failed)
  double fps = 0;              // frames per second of wall time over the last second
  double simulatedSeconds = 0; // simulated time of the current drape
  std::size_t particles = 0;   // 0 until the first build finishes
  std::string message;         // fallback note, the spec 5.8 failure message, or a build error
  sim::SolverBackend backend = sim::SolverBackend::Cpu;  // the backend actually in use
  bool gpuAvailable = true;    // false once a GPU backend failed to start
};

// Runs the tee drape on its own thread (spec Phase B 4.3). Commands are queued; a rebuild request supersedes any
// earlier one still pending. After each frame the thread publishes positions to exchange(); it sleeps once the drape
// has finished or while paused. The destructor stops the thread at the next frame boundary and joins it.
class SimController {
 public:
  using SolverFactory = std::function<std::unique_ptr<sim::Solver>(sim::SolverBackend)>;

  explicit SimController(SimConfig config, SolverFactory factory = sim::makeSolver);
  ~SimController();
  SimController(const SimController&) = delete;
  SimController& operator=(const SimController&) = delete;

  void setRunning(bool running);  // Simulate play/pause; status().running turns false once the thread sleeps
  void reset();                   // rebuild with the current config
  void configure(SimConfig config);  // Quality/Body/Solver change: rebuild
  SimStatus status() const;
  const PositionExchange& exchange() const { return exchange_; }

 private:
  void loop();
  void rebuild(const SimConfig& config);
  void publish();

  SolverFactory factory_;
  PositionExchange exchange_;

  mutable std::mutex mutex_;  // guards the fields below up to thread_
  std::condition_variable wake_;
  SimConfig config_;
  bool rebuildPending_ = true;
  bool running_ = true;
  bool stop_ = false;
  SimStatus status_;

  // Owned by the simulation thread.
  std::shared_ptr<const TeeScene> scene_;
  std::unique_ptr<sim::Solver> solver_;
  std::unique_ptr<sim::DrapeSession> session_;
  std::uint64_t revision_ = 0, frame_ = 0;
  std::string gpuNote_;  // the first GPU failure, kept for later CPU drapes
  std::deque<std::chrono::steady_clock::time_point> frameTimes_;

  std::thread thread_;  // last: starts after everything above is constructed
};

}  // namespace drape::session
