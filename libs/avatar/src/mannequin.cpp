#include "drape/avatar/mannequin.hpp"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "obj_reader.hpp"

namespace drape::avatar {

namespace {

namespace fs = std::filesystem;
using json = nlohmann::json;

constexpr int kFormat = 1;

// Keys of "regions" in <body>.json; they are the BodyRegion enumerator names.
constexpr std::pair<BodyRegion, const char*> kRegionNames[] = {
    {BodyRegion::Torso, "Torso"},       {BodyRegion::Neck, "Neck"},         {BodyRegion::Head, "Head"},
    {BodyRegion::ArmLeft, "ArmLeft"},   {BodyRegion::ArmRight, "ArmRight"}, {BodyRegion::LegLeft, "LegLeft"},
    {BodyRegion::LegRight, "LegRight"}, {BodyRegion::Waist, "Waist"}};

// Every landmark and girth tools/mannequin/build_mannequins.py writes; the drape tests, the hem target and
// the loader's tests rely on them, so a description without one is rejected.
constexpr const char* kLandmarkNames[] = {
    "neck_base", "hps_left", "hps_right", "shoulder_left", "shoulder_right", "chest", "waist", "hip", "crotch",
    "shoulder_joint_left", "shoulder_joint_right", "elbow_left", "elbow_right", "wrist_left", "wrist_right",
    "hip_joint_left", "hip_joint_right", "ankle_left", "ankle_right"};
constexpr const char* kGirthNames[] = {"chest", "waist", "hip", "neck"};

[[noreturn]] void fail(const fs::path& file, const std::string& reason) {
  throw std::runtime_error(file.string() + ": " + reason);
}

Vec3d toVec3(const json& j, const fs::path& file, const std::string& what) {
  if (!j.is_array() || j.size() != 3) fail(file, what + " must be an array of 3 numbers");
  return {j[0].get<double>(), j[1].get<double>(), j[2].get<double>()};
}

json readJson(const fs::path& file) {
  std::ifstream in(file);
  if (!in) fail(file, "cannot open");
  try {
    return json::parse(in);
  } catch (const json::exception& e) {
    fail(file, e.what());
  }
}

void readDescription(const fs::path& file, Mannequin& m) {
  const json doc = readJson(file);
  try {
    const json& formatValue = doc.at("format");
    if (!formatValue.is_number_integer()) fail(file, "format must be an integer, not " + formatValue.dump());
    const auto format = formatValue.get<std::int64_t>();
    if (format != kFormat) fail(file, "unsupported format " + std::to_string(format) + " (expected 1)");

    m.height = doc.at("height").get<double>();
    const json& girths = doc.at("girths");
    if (!girths.is_object()) fail(file, "girths must be an object");
    for (const auto& item : girths.items()) m.girths[item.key()] = item.value().get<double>();
    for (const char* name : kGirthNames) {
      if (!m.girths.contains(name)) fail(file, std::string("missing girth '") + name + "'");
    }
    const json& landmarks = doc.at("landmarks");
    if (!landmarks.is_object()) fail(file, "landmarks must be an object");
    for (const auto& item : landmarks.items()) {
      m.landmarks.points[item.key()] = toVec3(item.value(), file, "landmark '" + item.key() + "'");
    }
    for (const char* name : kLandmarkNames) {
      if (!m.landmarks.points.contains(name)) fail(file, std::string("missing landmark '") + name + "'");
    }

    const json& regions = doc.at("regions");
    if (!regions.is_object()) fail(file, "regions must be an object");
    for (const auto& [region, name] : kRegionNames) {
      const auto it = regions.find(name);
      if (it == regions.end()) fail(file, std::string("missing region '") + name + "'");
      RegionCylinder cylinder;
      cylinder.origin = toVec3(it->at("origin"), file, std::string("region '") + name + "' origin");
      cylinder.axis = toVec3(it->at("axis"), file, std::string("region '") + name + "' axis");
      cylinder.e0 = toVec3(it->at("e0"), file, std::string("region '") + name + "' e0");
      cylinder.radius = it->at("radius").get<double>();
      m.regions.cylinders[region] = cylinder;
    }
  } catch (const json::exception& e) {
    fail(file, e.what());
  }
}

}  // namespace

std::string_view bodyName(BodyType type) {
  switch (type) {
    case BodyType::Male: return "male";
    case BodyType::Female: return "female";
  }
  throw std::invalid_argument("unknown BodyType");
}

Mannequin loadMannequin(BodyType type) { return loadMannequin(type, fs::path(DRAPE_ASSET_DIR) / "mannequins"); }

Mannequin loadMannequin(BodyType type, const fs::path& mannequinDir) {
  const std::string name(bodyName(type));
  // Errors name the files by absolute path (symlinks are not resolved), whatever the working directory.
  const fs::path dir = fs::absolute(mannequinDir);
  Mannequin m;
  const fs::path objPath = dir / (name + ".obj");
  m.mesh = readObjTriangles(objPath);
  // An empty or truncated OBJ parses as a smaller mesh; the committed meshes are closed (false for no triangles).
  if (!isClosedManifold(m.mesh)) fail(objPath, "mesh is not a closed manifold");
  readDescription(dir / (name + ".json"), m);
  return m;
}

}  // namespace drape::avatar
