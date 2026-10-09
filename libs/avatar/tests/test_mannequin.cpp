#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <numeric>
#include <random>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drape/avatar/collision_bake.hpp"
#include "drape/avatar/mannequin.hpp"

using namespace drape;
using namespace drape::avatar;
using nlohmann::json;
namespace fs = std::filesystem;

namespace {

constexpr std::pair<BodyRegion, const char*> kRegionNames[] = {
    {BodyRegion::Torso, "Torso"},       {BodyRegion::Neck, "Neck"},         {BodyRegion::Head, "Head"},
    {BodyRegion::ArmLeft, "ArmLeft"},   {BodyRegion::ArmRight, "ArmRight"}, {BodyRegion::LegLeft, "LegLeft"},
    {BodyRegion::LegRight, "LegRight"}, {BodyRegion::Waist, "Waist"}};

// The builder's tapered egg (tools/mannequin/style.py): the head's ideal surface.
struct Egg {
  Vec3d centre{0, 0, 0};
  Vec3d radii{1, 1, 1};
  double taper = 1;

  // 1 on the surface, below 1 inside, above 1 outside. Below the centre the horizontal radii shrink with
  // the square of the depth, to `taper` at the bottom pole (no kink at the centre).
  double value(const Vec3d& p) const {
    const Vec3d q = p - centre;
    const double s = std::clamp(-q.y() / radii.y(), 0.0, 1.0);
    const double k = 1.0 - (1.0 - taper) * s * s;
    return std::pow(q.x() / (radii.x() * k), 2) + std::pow(q.y() / radii.y(), 2) +
           std::pow(q.z() / (radii.z() * k), 2);
  }

  // Where the ray from the centre through p meets the surface, by bisection on the distance.
  Vec3d project(const Vec3d& p) const {
    const double length = (p - centre).norm();
    const Vec3d dir = length > 0.0 ? Vec3d((p - centre) / length) : Vec3d(0, 1, 0);
    double lo = 0.0;
    double hi = 4.0 * radii.maxCoeff();
    for (int i = 0; i < 60; ++i) {
      const double mid = 0.5 * (lo + hi);
      (value(centre + mid * dir) < 1.0 ? lo : hi) = mid;
    }
    return centre + 0.5 * (lo + hi) * dir;
  }
};

struct Body {
  Mannequin mannequin;
  double targetHeight = 0;
  Egg egg;
  std::vector<std::uint32_t> headVertices;
  std::map<BodyRegion, std::vector<std::uint32_t>> regionVertices;
};

Vec3d toVec3(const json& j) { return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()}; }

Body loadBody(BodyType type) {
  Body b;
  b.mannequin = loadMannequin(type);
  b.targetHeight = type == BodyType::Male ? 1.80 : 1.70;

  const fs::path path = fs::path(DRAPE_ASSET_DIR) / "mannequins" / (std::string(bodyName(type)) + ".checks.json");
  std::ifstream in(path);
  if (!in) throw std::runtime_error(path.string() + ": cannot open");
  const json checks = json::parse(in);

  const json& egg = checks.at("egg");
  b.egg.centre = toVec3(egg.at("centre"));
  b.egg.radii = toVec3(egg.at("radii"));
  b.egg.taper = egg.at("taper").get<double>();
  b.headVertices = checks.at("head_vertices").get<std::vector<std::uint32_t>>();
  for (const auto& [region, name] : kRegionNames) {
    b.regionVertices[region] = checks.at("region_vertices").at(name).get<std::vector<std::uint32_t>>();
  }
  return b;
}

// Each body loads once for the whole suite.
const Body& body(BodyType type) {
  static const Body male = loadBody(BodyType::Male);
  static const Body female = loadBody(BodyType::Female);
  return type == BodyType::Male ? male : female;
}

template <class F>
void forEachBody(F&& check) {
  for (const BodyType type : {BodyType::Male, BodyType::Female}) {
    SCOPED_TRACE(std::string(bodyName(type)));
    check(body(type));
  }
}

Vec3d landmark(const Mannequin& m, const std::string& name) {
  const auto it = m.landmarks.points.find(name);
  if (it == m.landmarks.points.end()) throw std::runtime_error("missing landmark " + name);
  return it->second;
}

std::uint64_t edgeKey(std::uint32_t a, std::uint32_t b) {
  if (a > b) std::swap(a, b);
  return (static_cast<std::uint64_t>(a) << 32) | b;
}

double triangleArea(const TriangleMesh& m, const std::array<std::uint32_t, 3>& t) {
  return 0.5 * (m.vertices[t[1]] - m.vertices[t[0]]).cross(m.vertices[t[2]] - m.vertices[t[0]]).norm();
}

int componentCount(const TriangleMesh& m) {
  std::vector<std::uint32_t> parent(m.vertices.size());
  std::iota(parent.begin(), parent.end(), 0u);
  const auto find = [&](std::uint32_t v) {
    while (parent[v] != v) v = parent[v] = parent[parent[v]];
    return v;
  };
  for (const auto& t : m.triangles) {
    parent[find(t[0])] = find(t[1]);
    parent[find(t[1])] = find(t[2]);
  }
  int roots = 0;
  for (std::uint32_t v = 0; v < parent.size(); ++v) roots += find(v) == v;
  return roots;
}

// Voxel hash over vertex positions, 1 cm cells.
class VertexGrid {
 public:
  explicit VertexGrid(const std::vector<Vec3d>& vertices) : vertices_(vertices) {
    for (std::uint32_t i = 0; i < vertices.size(); ++i) cells_[key(cell(vertices[i]))].push_back(i);
  }

  bool anyWithin(const Vec3d& p, double distance) const {
    const auto c = cell(p);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto it = cells_.find(key({c[0] + dx, c[1] + dy, c[2] + dz}));
          if (it == cells_.end()) continue;
          for (const std::uint32_t v : it->second) {
            if ((vertices_[v] - p).norm() <= distance) return true;
          }
        }
      }
    }
    return false;
  }

 private:
  static constexpr double kCell = 0.01;

  static std::array<int, 3> cell(const Vec3d& p) {
    return {static_cast<int>(std::floor(p.x() / kCell)), static_cast<int>(std::floor(p.y() / kCell)),
            static_cast<int>(std::floor(p.z() / kCell))};
  }
  static std::uint64_t key(const std::array<int, 3>& c) {
    constexpr int offset = 4096;
    return (static_cast<std::uint64_t>(c[0] + offset) << 40) | (static_cast<std::uint64_t>(c[1] + offset) << 20) |
           static_cast<std::uint64_t>(c[2] + offset);
  }

  const std::vector<Vec3d>& vertices_;
  std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> cells_;
};

// Horizontal cross-section of a mesh at y = cut. A crossing point is identified by the mesh edge it
// lies on; a triangle with two crossing edges links their points. A vertex exactly on the plane
// counts as below it, so every crossed triangle has exactly two crossing edges.
struct Section {
  int loopsPositiveX = 0;
  int loopsNegativeX = 0;
  int loopsStraddlingX = 0;  // a loop with points on both sides of x = 0
  bool everyPointHasTwoSegments = true;
};

Section crossSection(const TriangleMesh& m, double cut) {
  std::unordered_map<std::uint64_t, std::size_t> pointOfEdge;
  std::vector<Vec3d> points;
  std::vector<std::size_t> parent;
  std::vector<int> segments;  // per point
  const auto find = [&](std::size_t v) {
    while (parent[v] != v) v = parent[v] = parent[parent[v]];
    return v;
  };
  const auto above = [&](std::uint32_t v) { return m.vertices[v].y() > cut; };
  const auto pointOn = [&](std::uint32_t a, std::uint32_t b) {
    const auto [it, isNew] = pointOfEdge.try_emplace(edgeKey(a, b), points.size());
    if (isNew) {
      const Vec3d &pa = m.vertices[a], &pb = m.vertices[b];
      points.push_back(pa + (cut - pa.y()) / (pb.y() - pa.y()) * (pb - pa));
      parent.push_back(parent.size());
      segments.push_back(0);
    }
    return it->second;
  };

  for (const auto& t : m.triangles) {
    std::vector<std::size_t> crossing;
    for (int k = 0; k < 3; ++k) {
      const std::uint32_t a = t[k], b = t[(k + 1) % 3];
      if (above(a) != above(b)) crossing.push_back(pointOn(a, b));
    }
    if (crossing.empty()) continue;
    EXPECT_EQ(crossing.size(), 2u);
    if (crossing.size() != 2) continue;
    ++segments[crossing[0]];
    ++segments[crossing[1]];
    parent[find(crossing[0])] = find(crossing[1]);
  }

  Section s;
  std::map<std::size_t, std::pair<double, double>> xRange;  // loop root -> (min x, max x)
  for (std::size_t p = 0; p < points.size(); ++p) {
    s.everyPointHasTwoSegments = s.everyPointHasTwoSegments && segments[p] == 2;
    const auto [it, isNew] = xRange.try_emplace(find(p), points[p].x(), points[p].x());
    if (!isNew) {
      it->second.first = std::min(it->second.first, points[p].x());
      it->second.second = std::max(it->second.second, points[p].x());
    }
  }
  for (const auto& [root, range] : xRange) {
    if (range.first > 0.0) {
      ++s.loopsPositiveX;
    } else if (range.second < 0.0) {
      ++s.loopsNegativeX;
    } else {
      ++s.loopsStraddlingX;
    }
  }
  return s;
}

// Angle from straight down in the frontal (x-y) projection, in degrees.
double frontalAngleDegrees(const Vec3d& d) { return std::atan2(std::abs(d.x()), -d.y()) * 180.0 / kPi; }

// A scratch directory under the system temp dir, removed on destruction.
class TempDir {
 public:
  TempDir() {
    std::random_device rd;
    std::ostringstream name;
    name << "drape_mannequin_test_" << std::hex << rd() << rd();
    path_ = fs::temp_directory_path() / name.str();
    fs::create_directories(path_);
  }
  ~TempDir() {
    std::error_code ignored;
    fs::remove_all(path_, ignored);
  }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;
  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

fs::path assetDir() { return fs::path(DRAPE_ASSET_DIR) / "mannequins"; }

void writeText(const fs::path& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary);
  out << text;
}

json readJson(const fs::path& path) {
  std::ifstream in(path);
  return json::parse(in);
}

// The message of the std::runtime_error loadMannequin throws; any other outcome is a test failure.
std::string loadFailure(BodyType type, const fs::path& dir) {
  try {
    loadMannequin(type, dir);
  } catch (const std::runtime_error& e) {
    return e.what();
  }
  ADD_FAILURE() << "loadMannequin did not throw std::runtime_error";
  return {};
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

}  // namespace

TEST(Mannequin, LoadsBothBodies) {
  forEachBody([](const Body& b) {
    const Mannequin& m = b.mannequin;
    EXPECT_EQ(m.mesh.vertices.size(), 13380u);
    EXPECT_EQ(m.mesh.triangles.size(), 26756u);
    EXPECT_EQ(m.regions.cylinders.size(), 8u);
    for (const auto& [region, name] : kRegionNames) {
      EXPECT_EQ(m.regions.cylinders.count(region), 1u) << name;
    }
    EXPECT_EQ(m.landmarks.points.size(), 19u);
    EXPECT_EQ(m.girths.size(), 4u);
  });
}

TEST(Mannequin, ClosedAndManifold) {
  forEachBody([](const Body& b) {
    const TriangleMesh& mesh = b.mannequin.mesh;
    std::unordered_map<std::uint64_t, int> uses;
    for (const auto& t : mesh.triangles) {
      for (int k = 0; k < 3; ++k) ++uses[edgeKey(t[k], t[(k + 1) % 3])];
    }
    int badEdges = 0;
    for (const auto& [edge, count] : uses) badEdges += count != 2;
    EXPECT_EQ(badEdges, 0) << "edges not used by exactly two triangles";
    EXPECT_TRUE(isClosedManifold(mesh));
    EXPECT_EQ(componentCount(mesh), 1);
    EXPECT_GT(signedVolume(mesh), 0.0) << "triangles face inward";

    double minArea = 1e9;
    for (const auto& t : mesh.triangles) minArea = std::min(minArea, triangleArea(mesh, t));
    EXPECT_GE(minArea, 1e-10);
  });
}

TEST(Mannequin, HeightAndFloor) {
  forEachBody([](const Body& b) {
    const Mannequin& m = b.mannequin;
    EXPECT_NEAR(m.height, b.targetHeight, 0.01);
    double minY = 1e9, maxY = -1e9;
    for (const Vec3d& v : m.mesh.vertices) {
      minY = std::min(minY, v.y());
      maxY = std::max(maxY, v.y());
    }
    EXPECT_NEAR(maxY, m.height, 1e-6);
    EXPECT_GE(minY, 0.0);
    EXPECT_LE(minY, 1e-4);
  });
}

TEST(Mannequin, FacesPlusZ) {
  forEachBody([](const Body& b) {
    const Mannequin& m = b.mannequin;
    EXPECT_GT(landmark(m, "chest").z(), m.regions.cylinders.at(BodyRegion::Torso).origin.z() + 0.05);
  });
}

TEST(Mannequin, MirrorSymmetric) {
  forEachBody([](const Body& b) {
    const auto& vertices = b.mannequin.mesh.vertices;
    const VertexGrid grid(vertices);
    int unmatched = 0;
    for (const Vec3d& v : vertices) unmatched += !grid.anyWithin({-v.x(), v.y(), v.z()}, 0.002);
    EXPECT_EQ(unmatched, 0) << "vertices without a mirror partner within 2 mm";
  });
}

TEST(Mannequin, PoseAngles) {
  forEachBody([](const Body& b) {
    const Mannequin& m = b.mannequin;
    for (const char* side : {"left", "right"}) {
      SCOPED_TRACE(side);
      const std::string s = side;
      const double arm = frontalAngleDegrees(landmark(m, "wrist_" + s) - landmark(m, "shoulder_joint_" + s));
      EXPECT_NEAR(arm, 45.0, 2.0);
      const double leg = frontalAngleDegrees(landmark(m, "ankle_" + s) - landmark(m, "hip_joint_" + s));
      EXPECT_NEAR(leg, 5.0, 1.0);
    }
  });
}

TEST(Mannequin, EggHead) {
  forEachBody([](const Body& b) {
    const auto& vertices = b.mannequin.mesh.vertices;
    ASSERT_FALSE(b.headVertices.empty());
    // Every vertex of the head proper is on the egg now (no cavities tucked under it): the 3634 vertices
    // with head weight >= 0.999, the Head region's vertices.
    EXPECT_EQ(b.headVertices.size(), 3634u);
    EXPECT_EQ(b.headVertices, b.regionVertices.at(BodyRegion::Head));

    double worst = 0;
    for (const std::uint32_t i : b.headVertices) {
      ASSERT_LT(i, vertices.size());
      worst = std::max(worst, (vertices[i] - b.egg.project(vertices[i])).norm());
    }
    EXPECT_LE(worst, 0.001) << "farthest head vertex from the egg, m";

    // Every triangle wholly on the egg faces outward: the eye sockets, mouth and nostrils lie flat on it.
    std::vector<bool> onEgg(vertices.size(), false);
    for (const std::uint32_t i : b.headVertices) onEgg[i] = true;
    int checked = 0, inward = 0;
    for (const auto& t : b.mannequin.mesh.triangles) {
      if (!(onEgg[t[0]] && onEgg[t[1]] && onEgg[t[2]])) continue;
      const Vec3d& a = vertices[t[0]];
      const Vec3d& bb = vertices[t[1]];
      const Vec3d& c = vertices[t[2]];
      const Vec3d normal = (bb - a).cross(c - a);
      ++checked;
      inward += !(normal.dot((a + bb + c) / 3.0 - b.egg.centre) > 0.0);
    }
    EXPECT_GT(checked, 0);
    EXPECT_EQ(inward, 0) << "egg-surface triangles whose normal points into the egg, of " << checked;
  });
}

TEST(Mannequin, SmoothFeet) {
  forEachBody([](const Body& b) {
    const Section s = crossSection(b.mannequin.mesh, 0.01);
    EXPECT_EQ(s.loopsPositiveX, 1);
    EXPECT_EQ(s.loopsNegativeX, 1);
    EXPECT_EQ(s.loopsStraddlingX, 0);
    EXPECT_TRUE(s.everyPointHasTwoSegments);
  });
}

TEST(Mannequin, RegionsContainTheirParts) {
  forEachBody([](const Body& b) {
    const auto& vertices = b.mannequin.mesh.vertices;
    for (const auto& [region, name] : kRegionNames) {
      SCOPED_TRACE(name);
      const RegionCylinder& cylinder = b.mannequin.regions.cylinders.at(region);
      const Vec3d axis = cylinder.axis.normalized();  // written with 6 decimals
      const auto& indices = b.regionVertices.at(region);
      ASSERT_FALSE(indices.empty());
      double worst = 0;
      for (const std::uint32_t i : indices) {
        ASSERT_LT(i, vertices.size());
        const Vec3d v = vertices[i] - cylinder.origin;
        worst = std::max(worst, (v - v.dot(axis) * axis).norm());
      }
      EXPECT_LE(worst, cylinder.radius + 1e-6) << "farthest vertex from the axis, m";
    }
  });
}

// Sleeves are placed on the arm cylinders, so they must hug the upper arm: run from the shoulder joint
// toward the elbow, with a radius of an upper arm.
TEST(Mannequin, ArmCylindersHugTheUpperArm) {
  forEachBody([](const Body& b) {
    const Mannequin& m = b.mannequin;
    const std::pair<BodyRegion, const char*> arms[] = {{BodyRegion::ArmLeft, "left"}, {BodyRegion::ArmRight, "right"}};
    for (const auto& [region, side] : arms) {
      SCOPED_TRACE(side);
      const std::string s = side;
      const RegionCylinder& c = m.regions.cylinders.at(region);
      EXPECT_GE(c.radius, 0.035);
      EXPECT_LE(c.radius, 0.08);
      const Vec3d shoulder = landmark(m, "shoulder_joint_" + s);
      const Vec3d upperArm = (landmark(m, "elbow_" + s) - shoulder).normalized();
      const double angle = std::acos(std::clamp(c.axis.normalized().dot(upperArm), -1.0, 1.0)) * 180.0 / kPi;
      EXPECT_LE(angle, 2.0) << "degrees between the axis and shoulder joint -> elbow";
      EXPECT_LE((c.origin - shoulder).norm(), 1e-6);
    }
  });
}

TEST(Mannequin, GirthsInAdultRanges) {
  const double maleChest = body(BodyType::Male).mannequin.girths.at("chest");
  EXPECT_GE(maleChest, 0.90);
  EXPECT_LE(maleChest, 1.10);
  const double femaleChest = body(BodyType::Female).mannequin.girths.at("chest");
  EXPECT_GE(femaleChest, 0.80);
  EXPECT_LE(femaleChest, 1.00);
}

TEST(Mannequin, CollisionFieldInsideOutside) {
  forEachBody([](const Body& b) {
    const Mannequin& m = b.mannequin;
    const CollisionField field = bakeCollisionField(std::span<const TriangleMesh>(&m.mesh, 1));
    const Vec3d chest = landmark(m, "chest");
    EXPECT_LT(field.sample(chest - Vec3d(0, 0, 0.05)), 0.0);
    EXPECT_GT(field.sample(chest + Vec3d(0, 0, 0.10)), 0.0);
  });
}

TEST(MannequinLoader, MissingDirectoryNamesThePath) {
  const std::string message = loadFailure(BodyType::Male, "/no/such/dir");
  EXPECT_TRUE(contains(message, (fs::path("/no/such/dir") / "male.obj").string())) << message;
  EXPECT_TRUE(contains(message, "cannot open")) << message;
}

TEST(MannequinLoader, NonTriangleFaceNamesTheLine) {
  TempDir dir;
  writeText(dir.path() / "male.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\nf 1 2 3 4\n");
  fs::copy_file(assetDir() / "male.json", dir.path() / "male.json");
  const std::string message = loadFailure(BodyType::Male, dir.path());
  EXPECT_TRUE(contains(message, (dir.path() / "male.obj").string() + ":5:")) << message;
  EXPECT_TRUE(contains(message, "face is not a triangle")) << message;
}

TEST(MannequinLoader, IndexOutOfRangeNamesTheLine) {
  TempDir dir;
  writeText(dir.path() / "male.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 99\n");
  fs::copy_file(assetDir() / "male.json", dir.path() / "male.json");
  const std::string message = loadFailure(BodyType::Male, dir.path());
  EXPECT_TRUE(contains(message, (dir.path() / "male.obj").string() + ":4:")) << message;
  EXPECT_TRUE(contains(message, "vertex index out of range")) << message;
}

TEST(MannequinLoader, UnsupportedFormatIsRejected) {
  TempDir dir;
  fs::copy_file(assetDir() / "male.obj", dir.path() / "male.obj");
  json doc = readJson(assetDir() / "male.json");
  doc["format"] = 2;
  writeText(dir.path() / "male.json", doc.dump(1));
  const std::string message = loadFailure(BodyType::Male, dir.path());
  EXPECT_TRUE(contains(message, (dir.path() / "male.json").string())) << message;
  EXPECT_TRUE(contains(message, "unsupported format 2")) << message;
}

TEST(MannequinLoader, MissingRegionIsRejected) {
  TempDir dir;
  fs::copy_file(assetDir() / "male.obj", dir.path() / "male.obj");
  json doc = readJson(assetDir() / "male.json");
  doc.at("regions").erase("Waist");
  writeText(dir.path() / "male.json", doc.dump(1));
  const std::string message = loadFailure(BodyType::Male, dir.path());
  EXPECT_TRUE(contains(message, (dir.path() / "male.json").string())) << message;
  EXPECT_TRUE(contains(message, "missing region 'Waist'")) << message;
}

// Copies the committed male mannequin into `dir`, with `edit` applied to its JSON description.
template <class Edit>
void writeEditedMale(const fs::path& dir, Edit edit) {
  fs::copy_file(assetDir() / "male.obj", dir / "male.obj");
  json doc = readJson(assetDir() / "male.json");
  edit(doc);
  writeText(dir / "male.json", doc.dump(1));
}

TEST(MannequinLoader, NonIntegerFormatIsRejected) {
  for (const json& format : {json(1.5), json("1")}) {
    SCOPED_TRACE(format.dump());
    TempDir dir;
    writeEditedMale(dir.path(), [&](json& doc) { doc["format"] = format; });
    const std::string message = loadFailure(BodyType::Male, dir.path());
    EXPECT_TRUE(contains(message, (dir.path() / "male.json").string())) << message;
    EXPECT_TRUE(contains(message, "format must be an integer")) << message;
  }
}

TEST(MannequinLoader, MissingLandmarkIsRejected) {
  // Every landmark the builder writes is required; the tee drapes and the tests rely on them.
  for (const char* key : {"chest", "hps_left", "elbow_right", "ankle_left", "crotch"}) {
    SCOPED_TRACE(key);
    TempDir dir;
    writeEditedMale(dir.path(), [&](json& doc) { doc.at("landmarks").erase(key); });
    const std::string message = loadFailure(BodyType::Male, dir.path());
    EXPECT_TRUE(contains(message, (dir.path() / "male.json").string() + ": missing landmark '" + key + "'"))
        << message;
  }
}

TEST(MannequinLoader, MissingGirthIsRejected) {
  for (const char* key : {"chest", "waist", "hip", "neck"}) {
    SCOPED_TRACE(key);
    TempDir dir;
    writeEditedMale(dir.path(), [&](json& doc) { doc.at("girths").erase(key); });
    const std::string message = loadFailure(BodyType::Male, dir.path());
    EXPECT_TRUE(contains(message, (dir.path() / "male.json").string() + ": missing girth '" + key + "'"))
        << message;
  }
}

// Changes the working directory for the scope, and back.
class WorkingDirectory {
 public:
  explicit WorkingDirectory(const fs::path& dir) : previous_(fs::current_path()) { fs::current_path(dir); }
  ~WorkingDirectory() {
    std::error_code ignored;
    fs::current_path(previous_, ignored);
  }
  WorkingDirectory(const WorkingDirectory&) = delete;
  WorkingDirectory& operator=(const WorkingDirectory&) = delete;

 private:
  fs::path previous_;
};

TEST(MannequinLoader, ErrorsNameTheAbsolutePath) {
  TempDir dir;
  writeEditedMale(dir.path(), [](json& doc) { doc.at("girths").erase("waist"); });
  WorkingDirectory cwd(dir.path().parent_path());
  const fs::path relative = dir.path().filename();
  ASSERT_TRUE(relative.is_relative());

  const std::string jsonError = loadFailure(BodyType::Male, relative);
  EXPECT_EQ(jsonError.rfind((fs::current_path() / relative / "male.json").string() + ": missing girth 'waist'", 0),
            0u)
      << jsonError;

  const std::string objError = loadFailure(BodyType::Male, relative / "nowhere");
  EXPECT_EQ(objError.rfind((fs::current_path() / relative / "nowhere" / "male.obj").string() + ": cannot open", 0),
            0u)
      << objError;
}

TEST(MannequinLoader, TruncatedJsonNamesTheFile) {
  TempDir dir;
  fs::copy_file(assetDir() / "male.obj", dir.path() / "male.obj");
  std::ifstream in(assetDir() / "male.json");
  std::ostringstream whole;
  whole << in.rdbuf();
  writeText(dir.path() / "male.json", whole.str().substr(0, whole.str().size() / 2));
  const std::string message = loadFailure(BodyType::Male, dir.path());
  EXPECT_TRUE(contains(message, (dir.path() / "male.json").string())) << message;
}

TEST(MannequinLoader, TruncatedObjNamesTheFile) {
  std::ifstream in(assetDir() / "male.obj", std::ios::binary);
  std::ostringstream whole;
  whole << in.rdbuf();
  const std::string obj = whole.str();
  const std::size_t firstFace = obj.find("\nf ");
  ASSERT_NE(firstFace, std::string::npos);
  const std::size_t midFaces = obj.find('\n', firstFace + (obj.size() - firstFace) / 2);
  ASSERT_NE(midFaces, std::string::npos);

  const std::pair<const char*, std::string> cuts[] = {
      {"empty file", ""},
      {"cut at a line boundary partway through the faces", obj.substr(0, midFaces + 1)},
      {"cut inside the last index", obj.substr(0, obj.size() - 3)}};
  for (const auto& [what, text] : cuts) {
    SCOPED_TRACE(what);
    TempDir dir;
    writeText(dir.path() / "male.obj", text);
    fs::copy_file(assetDir() / "male.json", dir.path() / "male.json");
    const std::string message = loadFailure(BodyType::Male, dir.path());
    EXPECT_TRUE(contains(message, (dir.path() / "male.obj").string())) << message;
    EXPECT_TRUE(contains(message, "not a closed manifold")) << message;
  }
}
