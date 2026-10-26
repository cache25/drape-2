#include "drape/render/image_metrics.hpp"

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include <miniz.h>

namespace drape::render {

double coverage(std::span<const std::uint8_t> rgba, std::uint32_t w, std::uint32_t h,
                const std::function<bool(std::uint8_t, std::uint8_t, std::uint8_t)>& pred) {
  const std::size_t pixels = static_cast<std::size_t>(w) * h;
  if (rgba.size() < pixels * 4) throw std::invalid_argument("coverage: buffer smaller than w * h * 4");
  if (pixels == 0) return 0.0;
  std::size_t hits = 0;
  for (std::size_t i = 0; i < pixels; ++i)
    if (pred(rgba[4 * i], rgba[4 * i + 1], rgba[4 * i + 2])) ++hits;
  return static_cast<double>(hits) / static_cast<double>(pixels);
}

bool isBlueTint(std::uint8_t r, std::uint8_t g, std::uint8_t b) { return b > r + 30 && b > g + 10; }

void writePng(const std::filesystem::path& path, std::span<const std::uint8_t> rgba, std::uint32_t w,
              std::uint32_t h) {
  if (rgba.size() < static_cast<std::size_t>(w) * h * 4) throw std::invalid_argument("writePng: buffer too small");
  std::size_t len = 0;
  void* png = tdefl_write_image_to_png_file_in_memory_ex(rgba.data(), static_cast<int>(w), static_cast<int>(h), 4,
                                                         &len, 6, MZ_FALSE);
  if (!png) throw std::runtime_error(path.string() + ": PNG encoding failed");
  std::unique_ptr<void, decltype(&mz_free)> guard(png, &mz_free);
  struct Closer {
    void operator()(std::FILE* f) const { std::fclose(f); }
  };
  std::unique_ptr<std::FILE, Closer> f(std::fopen(path.string().c_str(), "wb"));
  if (!f || std::fwrite(png, 1, len, f.get()) != len) throw std::runtime_error(path.string() + ": cannot write PNG");
}

}  // namespace drape::render
