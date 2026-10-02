#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "drape/core/fabric.hpp"
#include "drape/core/pattern.hpp"

namespace drape {

enum class BaseBody { Female, Male };
enum class Measurement { Height, Bust, Underbust, Waist, Hip, ShoulderWidth, ArmLength, Inseam, Neck, HeadCircumference };

struct ChartRef {
  Id chart;
  std::string size;
  bool operator==(const ChartRef&) const = default;
};

struct AvatarParams {
  BaseBody base = BaseBody::Female;
  std::map<Measurement, double> targets;
  std::optional<ChartRef> chartRef;
  std::vector<double> solvedWeights;
  bool operator==(const AvatarParams&) const = default;
};

struct SizeChart {
  Id id;
  std::string name;
  BaseBody base = BaseBody::Female;
  std::vector<std::string> sizes;
  std::map<Measurement, std::vector<double>> values;
  bool operator==(const SizeChart&) const = default;
};

enum class OutfitSlot { Top, Bottom, Headwear };
enum class SimQuality { Draft, Standard, Fine };

struct GarmentInstance {
  Id id;
  std::string generatorId;
  int generatorVersion = 1;
  OutfitSlot slot = OutfitSlot::Top;
  std::string size = "M";
  std::map<std::string, std::map<std::string, double>> specOverrides;  // size -> POM key -> value (m)
  Pattern pattern;
  std::map<std::string, Id> fabricBySlot;
  std::optional<Color> garmentColor;
  std::map<Id, Color> pieceColors;
  SimQuality quality = SimQuality::Standard;
  bool operator==(const GarmentInstance&) const = default;
};

enum class LightingPreset { StudioSoft, StudioContrast, OvercastOutdoor };

struct Camera {
  Vec3d position{0, 1.2, 3};
  Vec3d target{0, 1.1, 0};
  double fovYDegrees = 35;
  bool operator==(const Camera&) const = default;
};

struct Scene {
  LightingPreset lighting = LightingPreset::StudioSoft;
  Camera camera;
  bool operator==(const Scene&) const = default;
};

enum class DisplayUnits { Centimeters, Inches };

struct ProjectMetadata {
  std::string name;
  std::string created;   // ISO-8601 UTC
  std::string modified;  // ISO-8601 UTC
  bool operator==(const ProjectMetadata&) const = default;
};

struct Project {
  ProjectMetadata meta;
  DisplayUnits units = DisplayUnits::Centimeters;
  AvatarParams avatar;
  std::vector<SizeChart> sizeCharts;
  std::vector<GarmentInstance> garments;
  std::vector<Fabric> customFabrics;
  Scene scene;
  bool operator==(const Project&) const = default;
};

}  // namespace drape
