#include "coloring.hpp"

#include <algorithm>

namespace drape::sim {

std::vector<std::vector<std::uint32_t>> greedyVertexColor(std::span<const std::array<std::uint32_t, 4>> items,
                                                          std::uint32_t particleCount) {
  std::vector<std::vector<std::uint32_t>> neighbors(particleCount);
  for (const auto& item : items) {
    for (auto a : item) {
      if (a == kNoParticle) continue;
      for (auto b : item) {
        if (b != kNoParticle && b != a) neighbors[a].push_back(b);
      }
    }
  }
  std::vector<int> color(particleCount, -1);
  std::vector<std::vector<std::uint32_t>> colors;
  std::vector<char> taken;
  for (std::uint32_t v = 0; v < particleCount; ++v) {
    taken.assign(colors.size() + 1, 0);
    for (auto n : neighbors[v]) {
      if (color[n] >= 0 && static_cast<std::size_t>(color[n]) < taken.size()) taken[color[n]] = 1;
    }
    const auto c = static_cast<std::size_t>(std::find(taken.begin(), taken.end(), 0) - taken.begin());
    if (c >= colors.size()) colors.resize(c + 1);
    colors[c].push_back(v);
    color[v] = static_cast<int>(c);
  }
  return colors;
}

}  // namespace drape::sim
