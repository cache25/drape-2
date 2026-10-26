#pragma once

#include <cstddef>
#include <vector>

#include "gpu/compute_device.hpp"

namespace drape::sim {

// Runs the kernels.h bodies on the CPU, one thread index at a time, in 32-bit float exactly as written. Used for
// the GpuEmulated backend and for testing the GPU algorithm without a Mac.
class EmulatedDevice final : public ComputeDevice {
 public:
  BufferId create(std::size_t bytes) override;
  void* contents(BufferId b) override;
  void begin() override;
  void dispatch(Kernel k, std::uint32_t threads, std::span<const BufferId> buffers) override;
  void submitAndWait() override;

 private:
  struct Call {
    Kernel kernel;
    std::uint32_t threads;
    std::vector<BufferId> buffers;
  };
  void run(const Call& c);

  // 16-byte aligned storage per buffer (float4 elements).
  std::vector<std::vector<std::max_align_t>> buffers_;
  std::vector<Call> recorded_;
};

}  // namespace drape::sim
