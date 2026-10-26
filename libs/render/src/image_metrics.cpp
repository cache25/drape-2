#include "drape/render/image_metrics.hpp"

#include <stdexcept>

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

}  // namespace drape::render
