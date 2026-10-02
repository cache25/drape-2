#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace drape::sim {

inline constexpr std::uint32_t kNoParticle = UINT32_MAX;

// First-fit vertex coloring in vertex order: two vertices that appear together in any item never share a
// color. Items list up to 4 vertices (unused slots kNoParticle). Returns the vertices of each color.
std::vector<std::vector<std::uint32_t>> greedyVertexColor(std::span<const std::array<std::uint32_t, 4>> items,
                                                          std::uint32_t particleCount);

}  // namespace drape::sim
