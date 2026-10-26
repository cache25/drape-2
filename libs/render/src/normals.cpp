#include "drape/render/normals.hpp"

namespace drape::render {

std::vector<Vec3f> vertexNormals(std::span<const Vec3f> positions,
                                 std::span<const std::array<std::uint32_t, 3>> triangles,
                                 std::span<const std::pair<std::uint32_t, std::uint32_t>> weldPairs) {
  std::vector<Vec3f> n(positions.size(), Vec3f::Zero());
  for (const auto& t : triangles) {
    const Vec3f c = (positions[t[1]] - positions[t[0]]).cross(positions[t[2]] - positions[t[0]]);
    for (auto v : t) n[v] += c;
  }
  for (const auto& [a, b] : weldPairs) {
    const Vec3f s = n[a] + n[b];
    n[a] = s;
    n[b] = s;
  }
  for (auto& v : n) {
    const float len = v.norm();
    v = len > 0 ? Vec3f(v / len) : Vec3f(0, 1, 0);
  }
  return n;
}

}  // namespace drape::render
