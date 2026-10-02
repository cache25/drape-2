#include "drape/core/triangle_mesh.hpp"

#include <map>
#include <utility>

namespace drape {

double signedVolume(const TriangleMesh& m) {
  double v = 0;
  for (const auto& t : m.triangles) {
    v += m.vertices[t[0]].dot(m.vertices[t[1]].cross(m.vertices[t[2]]));
  }
  return v / 6.0;
}

bool isClosedManifold(const TriangleMesh& m) {
  if (m.triangles.empty()) return false;
  std::map<std::pair<std::uint32_t, std::uint32_t>, int> directed;
  for (const auto& t : m.triangles) {
    for (int k = 0; k < 3; ++k) ++directed[{t[k], t[(k + 1) % 3]}];
  }
  for (const auto& [e, count] : directed) {
    if (count != 1) return false;
    auto it = directed.find({e.second, e.first});
    if (it == directed.end() || it->second != 1) return false;
  }
  return true;
}

}  // namespace drape
