#include "drape/avatar/mannequin.hpp"

#include <nlohmann/json.hpp>

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
    const int format = doc.at("format").get<int>();
    if (format != kFormat) fail(file, "unsupported format " + std::to_string(format) + " (expected 1)");

    m.height = doc.at("height").get<double>();
    for (const auto& item : doc.at("girths").items()) m.girths[item.key()] = item.value().get<double>();
    for (const auto& item : doc.at("landmarks").items()) {
      m.landmarks.points[item.key()] = toVec3(item.value(), file, "landmark '" + item.key() + "'");
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
  Mannequin m;
  m.mesh = readObjTriangles(mannequinDir / (name + ".obj"));
  readDescription(mannequinDir / (name + ".json"), m);
  return m;
}

}  // namespace drape::avatar
