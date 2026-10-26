#define NS_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION
#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>

#include "gpu/metal_device.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace drape::sim {

namespace {

std::string describe(NS::Error* e) {
  if (!e || !e->localizedDescription()) return "unknown error";
  return e->localizedDescription()->utf8String();
}

NS::String* nsString(const std::string& s) { return NS::String::string(s.c_str(), NS::UTF8StringEncoding); }

}  // namespace

struct MetalDevice::Impl {
  MTL::Device* device = nullptr;
  MTL::CommandQueue* queue = nullptr;
  std::array<MTL::ComputePipelineState*, static_cast<std::size_t>(Kernel::Count)> pipelines{};
  std::vector<MTL::Buffer*> buffers;
  NS::AutoreleasePool* pool = nullptr;
  MTL::CommandBuffer* cmd = nullptr;
  MTL::ComputeCommandEncoder* enc = nullptr;

  ~Impl() {
    if (enc) enc->endEncoding();
    if (pool) pool->release();
    for (auto* b : buffers) b->release();
    for (auto* p : pipelines)
      if (p) p->release();
    if (queue) queue->release();
    if (device) device->release();
  }
};

MetalDevice::MetalDevice(const std::filesystem::path& kernelSource) : impl_(std::make_unique<Impl>()) {
  NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
  struct PoolGuard {
    NS::AutoreleasePool* p;
    ~PoolGuard() { p->release(); }
  } guard{pool};

  impl_->device = MTL::CreateSystemDefaultDevice();
  if (!impl_->device) throw std::runtime_error("metal: no Metal device");
  const std::string path = std::filesystem::absolute(kernelSource).string();
  std::ifstream in(kernelSource);
  if (!in) throw std::runtime_error("metal: cannot load " + path + ": file not found or unreadable");
  std::stringstream text;
  text << in.rdbuf();

  MTL::CompileOptions* options = MTL::CompileOptions::alloc()->init();
  options->setFastMathEnabled(false);
  options->setLanguageVersion(MTL::LanguageVersion3_1);
  NS::Error* error = nullptr;
  MTL::Library* library = impl_->device->newLibrary(nsString(text.str()), options, &error);
  options->release();
  if (!library) throw std::runtime_error("metal: cannot load " + path + ": " + describe(error));
  for (std::size_t k = 0; k < impl_->pipelines.size(); ++k) {
    const std::string name(kernelName(static_cast<Kernel>(k)));
    MTL::Function* fn = library->newFunction(nsString(name));
    if (!fn) {
      library->release();
      throw std::runtime_error("metal: kernel '" + name + "' missing from " + path);
    }
    impl_->pipelines[k] = impl_->device->newComputePipelineState(fn, &error);
    fn->release();
    if (!impl_->pipelines[k]) {
      library->release();
      throw std::runtime_error("metal: kernel '" + name + "': " + describe(error));
    }
  }
  library->release();
  impl_->queue = impl_->device->newCommandQueue();
  if (!impl_->queue) throw std::runtime_error("metal: cannot create a command queue");
}

MetalDevice::~MetalDevice() = default;

BufferId MetalDevice::create(std::size_t bytes) {
  const std::size_t size = std::max<std::size_t>(bytes, 16);
  MTL::Buffer* b = impl_->device->newBuffer(size, MTL::ResourceStorageModeShared);
  if (!b) throw std::runtime_error("metal: cannot allocate a " + std::to_string(size) + "-byte buffer");
  std::memset(b->contents(), 0, size);
  impl_->buffers.push_back(b);
  return BufferId{static_cast<std::uint32_t>(impl_->buffers.size() - 1)};
}

void* MetalDevice::contents(BufferId b) {
  if (b.index >= impl_->buffers.size()) throw std::out_of_range("metal: unknown buffer");
  return impl_->buffers[b.index]->contents();
}

void MetalDevice::begin() {
  if (impl_->enc) throw std::logic_error("metal: begin() while a submission is open");
  impl_->pool = NS::AutoreleasePool::alloc()->init();
  impl_->cmd = impl_->queue->commandBuffer();
  impl_->enc = impl_->cmd->computeCommandEncoder(MTL::DispatchTypeSerial);
}

void MetalDevice::dispatch(Kernel k, std::uint32_t threads, std::span<const BufferId> buffers) {
  if (!impl_->enc) throw std::logic_error("metal: dispatch() outside begin()/submitAndWait()");
  if (threads == 0) return;
  MTL::ComputePipelineState* p = impl_->pipelines[static_cast<std::size_t>(k)];
  impl_->enc->setComputePipelineState(p);
  for (std::size_t i = 0; i < buffers.size(); ++i)
    impl_->enc->setBuffer(impl_->buffers.at(buffers[i].index), 0, static_cast<NS::UInteger>(i));
  const NS::UInteger group = std::min<NS::UInteger>(64, p->maxTotalThreadsPerThreadgroup());
  impl_->enc->dispatchThreads(MTL::Size(threads, 1, 1), MTL::Size(group, 1, 1));
}

void MetalDevice::submitAndWait() {
  if (!impl_->enc) throw std::logic_error("metal: submitAndWait() without begin()");
  impl_->enc->endEncoding();
  impl_->enc = nullptr;
  impl_->cmd->commit();
  impl_->cmd->waitUntilCompleted();
  const bool failed = impl_->cmd->status() == MTL::CommandBufferStatusError;
  const std::string what = failed ? describe(impl_->cmd->error()) : std::string();
  impl_->cmd = nullptr;
  impl_->pool->release();
  impl_->pool = nullptr;
  if (failed) throw std::runtime_error("metal: command buffer failed: " + what);
}

}  // namespace drape::sim
