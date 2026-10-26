#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <span>

namespace drape::render {

// Fraction of the w x h RGBA pixels for which pred(r, g, b) holds.
double coverage(std::span<const std::uint8_t> rgba, std::uint32_t w, std::uint32_t h,
                const std::function<bool(std::uint8_t r, std::uint8_t g, std::uint8_t b)>& pred);

// The preview/viewport garment blue: b > r + 30 and b > g + 10.
bool isBlueTint(std::uint8_t r, std::uint8_t g, std::uint8_t b);

// Writes w x h RGBA pixels (top row first) as a PNG. Throws std::runtime_error naming the path on failure.
void writePng(const std::filesystem::path& path, std::span<const std::uint8_t> rgba, std::uint32_t w,
              std::uint32_t h);

}  // namespace drape::render
