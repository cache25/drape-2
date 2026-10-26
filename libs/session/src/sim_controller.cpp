#include "drape/session/sim_controller.hpp"

#include <exception>
#include <utility>

#include "drape/sim/drape_runner.hpp"

namespace drape::session {

using Clock = std::chrono::steady_clock;

SimController::SimController(SimConfig config, SolverFactory factory)
    : factory_(std::move(factory)), config_(std::move(config)) {
  status_.backend = config_.backend;
  thread_ = std::thread([this] { loop(); });
}

SimController::~SimController() {
  {
    std::lock_guard lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void SimController::setRunning(bool running) {
  {
    std::lock_guard lock(mutex_);
    running_ = running;
  }
  wake_.notify_all();
}

void SimController::reset() {
  {
    std::lock_guard lock(mutex_);
    rebuildPending_ = true;
  }
  wake_.notify_all();
}

void SimController::configure(SimConfig config) {
  {
    std::lock_guard lock(mutex_);
    config_ = std::move(config);
    rebuildPending_ = true;
  }
  wake_.notify_all();
}

SimStatus SimController::status() const {
  std::lock_guard lock(mutex_);
  return status_;
}

void SimController::loop() {
  while (true) {
    SimConfig config;
    bool doRebuild = false;
    {
      std::unique_lock lock(mutex_);
      auto ready = [&] { return stop_ || rebuildPending_ || (running_ && session_ && !session_->finished()); };
      if (!ready()) {
        status_.running = false;  // the thread is going to sleep: no frame runs until it is woken
        wake_.wait(lock, ready);
      }
      if (stop_) return;
      status_.running = true;
      if (rebuildPending_) {
        rebuildPending_ = false;
        doRebuild = true;
        config = config_;
      }
    }
    if (doRebuild) {
      rebuild(config);
      continue;
    }
    const sim::DrapeState state = session_->advance();
    const auto now = Clock::now();
    frameTimes_.push_back(now);
    while (!frameTimes_.empty() && now - frameTimes_.front() > std::chrono::seconds(1)) frameTimes_.pop_front();
    publish();
    std::lock_guard lock(mutex_);
    status_.state = state;
    status_.fps = static_cast<double>(frameTimes_.size());
    if (state == sim::DrapeState::Failed) status_.message = session_->result().message;
  }
}

void SimController::rebuild(const SimConfig& config) {
  session_.reset();
  solver_.reset();
  frameTimes_.clear();
  std::string message;
  bool gpuAvailable = true;
  sim::SolverBackend backend = config.backend;
  try {
    scene_ = makeTeeScene(config.body, config.quality, config.size, config.mannequinDir);
    try {
      solver_ = factory_(backend);
    } catch (const std::exception& e) {
      if (backend == sim::SolverBackend::Cpu) throw;
      message = std::string("GPU unavailable: ") + e.what() + "; using CPU";
      gpuAvailable = false;
      backend = sim::SolverBackend::Cpu;
      solver_ = factory_(backend);
    }
    solver_->build(scene_->scene);
    sim::DrapeOptions options;
    options.substeps = sim::substepsFor(config.quality);
    options.iterations = sim::iterationsFor(config.quality);
    session_ = std::make_unique<sim::DrapeSession>(*solver_, std::vector<sim::GarmentId>{"tee"}, options);
  } catch (const std::exception& e) {
    session_.reset();
    solver_.reset();
    scene_.reset();
    std::lock_guard lock(mutex_);
    status_.state = sim::DrapeState::Failed;
    status_.particles = 0;
    status_.fps = 0;
    status_.message = e.what();
    status_.backend = backend;
    return;
  }
  ++revision_;
  publish();
  std::lock_guard lock(mutex_);
  status_.state = session_->state();
  status_.particles = solver_->particleCount();
  status_.fps = 0;
  status_.message = message;
  status_.backend = backend;
  if (!gpuAvailable) status_.gpuAvailable = false;
}

void SimController::publish() {
  PublishedFrame f;
  f.revision = revision_;
  f.frame = ++frame_;
  f.scene = scene_;
  f.positions.resize(solver_->particleCount());
  solver_->readPositions(f.positions);
  exchange_.publish(std::move(f));
}

}  // namespace drape::session
