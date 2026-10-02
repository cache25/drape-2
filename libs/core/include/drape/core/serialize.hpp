#pragma once

#include <nlohmann/json.hpp>

#include "drape/core/project.hpp"

// Eigen vectors serialize as JSON arrays.
NLOHMANN_JSON_NAMESPACE_BEGIN
template <int N>
struct adl_serializer<Eigen::Matrix<double, N, 1>> {
  template <typename BasicJsonType>
  static void to_json(BasicJsonType& j, const Eigen::Matrix<double, N, 1>& v) {
    j = BasicJsonType::array();
    for (int i = 0; i < N; ++i) j.push_back(v[i]);
  }
  template <typename BasicJsonType>
  static void from_json(const BasicJsonType& j, Eigen::Matrix<double, N, 1>& v) {
    if (!j.is_array() || j.size() != static_cast<std::size_t>(N)) {
      throw detail::type_error::create(302, "expected an array of " + std::to_string(N) + " numbers", &j);
    }
    for (int i = 0; i < N; ++i) v[i] = j.at(static_cast<std::size_t>(i)).template get<double>();
  }
};
NLOHMANN_JSON_NAMESPACE_END

namespace drape {

using json = nlohmann::json;

#define DRAPE_DECLARE_JSON(T)          \
  void to_json(json& j, const T& v);   \
  void from_json(const json& j, T& v);

DRAPE_DECLARE_JSON(BodyRegion)
DRAPE_DECLARE_JSON(SeamKind)
DRAPE_DECLARE_JSON(InternalLineKind)
DRAPE_DECLARE_JSON(TrimKind)
DRAPE_DECLARE_JSON(BaseBody)
DRAPE_DECLARE_JSON(Measurement)
DRAPE_DECLARE_JSON(OutfitSlot)
DRAPE_DECLARE_JSON(SimQuality)
DRAPE_DECLARE_JSON(LightingPreset)
DRAPE_DECLARE_JSON(DisplayUnits)

DRAPE_DECLARE_JSON(RegionCylinder)
DRAPE_DECLARE_JSON(BodyRegions)
DRAPE_DECLARE_JSON(Outline)
DRAPE_DECLARE_JSON(Placement)
DRAPE_DECLARE_JSON(LayerRelation)
DRAPE_DECLARE_JSON(ElasticSpec)
DRAPE_DECLARE_JSON(PatternPiece)
DRAPE_DECLARE_JSON(EdgeRef)
DRAPE_DECLARE_JSON(Seam)
DRAPE_DECLARE_JSON(Color)
DRAPE_DECLARE_JSON(InternalLine)
DRAPE_DECLARE_JSON(Trim)
DRAPE_DECLARE_JSON(Pattern)
DRAPE_DECLARE_JSON(FabricPhysical)
DRAPE_DECLARE_JSON(FabricVisual)
DRAPE_DECLARE_JSON(Fabric)
DRAPE_DECLARE_JSON(ChartRef)
DRAPE_DECLARE_JSON(AvatarParams)
DRAPE_DECLARE_JSON(SizeChart)
DRAPE_DECLARE_JSON(GarmentInstance)
DRAPE_DECLARE_JSON(Camera)
DRAPE_DECLARE_JSON(Scene)
DRAPE_DECLARE_JSON(ProjectMetadata)
DRAPE_DECLARE_JSON(Project)

#undef DRAPE_DECLARE_JSON

}  // namespace drape
