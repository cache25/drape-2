#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "drape/core/math.hpp"
#include "drape/session/tee_scene.hpp"

namespace drape::session {

// One frame of particle positions, tied to the scene (mesh, body, pattern) they belong to.
struct PublishedFrame {
  std::uint64_t revision = 0;             // bumps on every rebuild (new mesh)
  std::uint64_t frame = 0;                // bumps on every published frame, across revisions
  std::shared_ptr<const TeeScene> scene;  // the scene of this revision
  std::vector<Vec3f> positions;           // size == scene->mesh.rest.size()
};

// The simulation thread publishes into the back buffer and swaps; readers copy the front one (spec Phase B 4.3).
// A reader always gets positions and scene from the same publish, so they never mismatch.
class PositionExchange {
 public:
  void publish(PublishedFrame&& f) {
    back_ = std::move(f);
    std::lock_guard lock(mutex_);
    std::swap(front_, back_);
  }

  // Copies the latest frame into `out` if it is newer than `sinceFrame`.
  bool latest(PublishedFrame& out, std::uint64_t sinceFrame) const {
    std::lock_guard lock(mutex_);
    if (front_.frame <= sinceFrame) return false;
    out = front_;
    return true;
  }

 private:
  mutable std::mutex mutex_;
  PublishedFrame front_, back_;  // back_ is touched only by the publishing thread
};

}  // namespace drape::session
