#include "obj_writer.hpp"

#include <fstream>
#include <stdexcept>

namespace drape::tools {

void writeObj(const std::filesystem::path& path, std::span<const std::pair<std::string, TriangleMesh>> objects) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("could not write " + path.string());
  out.precision(9);
  std::size_t base = 1;
  for (const auto& [name, mesh] : objects) {
    out << "o " << name << '\n';
    for (const auto& v : mesh.vertices) out << "v " << v.x() << ' ' << v.y() << ' ' << v.z() << '\n';
    for (const auto& t : mesh.triangles) out << "f " << t[0] + base << ' ' << t[1] + base << ' ' << t[2] + base << '\n';
    base += mesh.vertices.size();
  }
  if (!out) throw std::runtime_error("could not write " + path.string());
}

}  // namespace drape::tools
