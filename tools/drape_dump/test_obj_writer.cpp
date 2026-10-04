#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>
#include <vector>

#include "obj_writer.hpp"

using namespace drape;

TEST(ObjWriter, OneObjectBlockPerMeshWithGlobalOneBasedIndices) {
  TriangleMesh a{{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, {{0, 1, 2}}};
  TriangleMesh b{{{0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}}, {{0, 1, 2}, {1, 3, 2}}};
  const std::vector<std::pair<std::string, TriangleMesh>> objects{{"body_0", a}, {"tee", b}};
  const auto path = std::filesystem::temp_directory_path() / "drape_obj_writer_test.obj";
  tools::writeObj(path, objects);

  std::ifstream in(path);
  std::vector<std::string> objectsSeen, faces;
  int vertices = 0;
  for (std::string line; std::getline(in, line);) {
    std::istringstream ls(line);
    std::string tag;
    ls >> tag;
    if (tag == "o") objectsSeen.push_back(line.substr(2));
    if (tag == "v") ++vertices;
    if (tag == "f") faces.push_back(line);
  }
  EXPECT_EQ(objectsSeen, (std::vector<std::string>{"body_0", "tee"}));
  EXPECT_EQ(vertices, 7);
  EXPECT_EQ(faces, (std::vector<std::string>{"f 1 2 3", "f 4 5 6", "f 5 7 6"}));
  std::filesystem::remove(path);
}

TEST(ObjWriter, UnwritablePathThrows) {
  const std::vector<std::pair<std::string, TriangleMesh>> objects;
  EXPECT_THROW(tools::writeObj("/nonexistent-dir/x.obj", objects), std::runtime_error);
}
