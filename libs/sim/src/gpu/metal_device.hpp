#pragma once

#include <filesystem>
#include <memory>

#include "gpu/compute_device.hpp"

namespace drape::sim {

// ComputeDevice on the system's Metal GPU (metal-cpp). Compiles the combined kernel source at construction with fast
// math off, makes one pipeline per Kernel, and runs each submission in one serial compute encoder over shared-storage
// buffers. Throws std::runtime_error ("metal: ...") when there is no Metal device or the kernels do not load.
class MetalDevice final : public ComputeDevice {
 public:
  explicit MetalDevice(const std::filesystem::path& kernelSource);
  ~MetalDevice() override;
  MetalDevice(const MetalDevice&) = delete;
  MetalDevice& operator=(const MetalDevice&) = delete;

  BufferId create(std::size_t bytes) override;
  void* contents(BufferId b) override;
  void begin() override;
  void dispatch(Kernel k, std::uint32_t threads, std::span<const BufferId> buffers) override;
  void submitAndWait() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace drape::sim
