#pragma once

#include <cstdint>
#include <functional>
#include <span>

namespace drape::render {

// Fraction of the w x h RGBA pixels for which pred(r, g, b) holds.
double coverage(std::span<const std::uint8_t> rgba, std::uint32_t w, std::uint32_t h,
                const std::function<bool(std::uint8_t r, std::uint8_t g, std::uint8_t b)>& pred);

// The preview/viewport garment blue: b > r + 30 and b > g + 10.
bool isBlueTint(std::uint8_t r, std::uint8_t g, std::uint8_t b);

}  // namespace drape::render
