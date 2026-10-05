#include "obj_reader.hpp"

#include <array>
#include <charconv>
#include <cstdint>
#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace drape::avatar {

namespace {

[[noreturn]] void fail(const std::filesystem::path& path, int line, const std::string& reason) {
  throw std::runtime_error(path.string() + ":" + std::to_string(line) + ": " + reason);
}

}  // namespace

TriangleMesh readObjTriangles(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error(path.string() + ": cannot open");

  TriangleMesh mesh;
  std::string text;
  for (int line = 1; std::getline(in, text); ++line) {
    std::istringstream fields(text);
    fields.imbue(std::locale::classic());  // "0.5" parses the same under any global locale
    std::string tag;
    fields >> tag;
    if (tag == "v") {
      double x = 0, y = 0, z = 0;
      if (!(fields >> x >> y >> z)) fail(path, line, "malformed vertex");
      mesh.vertices.emplace_back(x, y, z);
    } else if (tag == "f") {
      std::vector<std::string> corners;
      for (std::string corner; fields >> corner;) corners.push_back(corner);
      if (corners.size() != 3) fail(path, line, "face is not a triangle");
      std::array<std::uint32_t, 3> triangle{};
      for (int k = 0; k < 3; ++k) {
        const std::string& corner = corners[k];
        long index = 0;
        const std::size_t slash = corner.find('/');  // "a/b/c": the vertex index is the part before it
        const char* end = corner.data() + (slash == std::string::npos ? corner.size() : slash);
        const auto parsed = std::from_chars(corner.data(), end, index);
        if (parsed.ec != std::errc{} || parsed.ptr != end) fail(path, line, "malformed face index");
        if (index < 1 || static_cast<std::size_t>(index) > mesh.vertices.size()) {
          fail(path, line, "vertex index out of range");
        }
        triangle[k] = static_cast<std::uint32_t>(index - 1);
      }
      mesh.triangles.push_back(triangle);
    }
  }
  return mesh;
}

}  // namespace drape::avatar
