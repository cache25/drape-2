#include <gtest/gtest.h>

#include <random>
#include <set>

#include "coloring.hpp"

TEST(Coloring, NoConflicts) {
  std::mt19937 rng(3);
  std::uniform_int_distribution<std::uint32_t> pick(0, 199);
  std::vector<std::array<std::uint32_t, 4>> items;
  for (int i = 0; i < 500; ++i) {
    std::set<std::uint32_t> ps;
    const std::size_t n = (i % 2) ? 3 : 4;
    while (ps.size() < n) ps.insert(pick(rng));
    std::array<std::uint32_t, 4> item{UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
    std::size_t k = 0;
    for (auto p : ps) item[k++] = p;
    items.push_back(item);
  }
  const auto colors = drape::sim::greedyVertexColor(items, 200);
  std::vector<int> colorOf(200, -1);
  for (std::size_t c = 0; c < colors.size(); ++c) {
    for (auto v : colors[c]) {
      EXPECT_EQ(colorOf[v], -1) << "vertex " << v << " colored twice";
      colorOf[v] = static_cast<int>(c);
    }
  }
  for (int c : colorOf) EXPECT_GE(c, 0);
  for (const auto& item : items) {
    std::set<int> seen;
    for (auto p : item) {
      if (p == UINT32_MAX) continue;
      EXPECT_TRUE(seen.insert(colorOf[p]).second) << "two vertices of one item share a color";
    }
  }
}
