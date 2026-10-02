#include "drape/core/serialize.hpp"

#include <string>
#include <utility>

namespace drape {

namespace {

// Enum <-> string. Unknown strings throw json::type_error (302) so bad files are rejected, not guessed.
template <class E, std::size_t N>
void enumToJson(json& j, E e, const std::pair<E, const char*> (&table)[N]) {
  for (const auto& [value, name] : table) {
    if (value == e) {
      j = name;
      return;
    }
  }
  j = nullptr;
}

template <class E, std::size_t N>
void enumFromJson(const json& j, E& e, const std::pair<E, const char*> (&table)[N]) {
  const auto& s = j.get_ref<const std::string&>();
  for (const auto& [value, name] : table) {
    if (s == name) {
      e = value;
      return;
    }
  }
  throw json::type_error::create(302, "unknown value '" + s + "'", &j);
}

constexpr std::pair<BodyRegion, const char*> kBodyRegion[] = {
    {BodyRegion::Torso, "torso"},     {BodyRegion::Neck, "neck"},         {BodyRegion::Head, "head"},
    {BodyRegion::ArmLeft, "armLeft"}, {BodyRegion::ArmRight, "armRight"}, {BodyRegion::LegLeft, "legLeft"},
    {BodyRegion::LegRight, "legRight"}, {BodyRegion::Waist, "waist"}};
constexpr std::pair<SeamKind, const char*> kSeamKind[] = {
    {SeamKind::Normal, "normal"}, {SeamKind::ClosedZipper, "closedZipper"}, {SeamKind::ClosedFly, "closedFly"}};
constexpr std::pair<InternalLineKind, const char*> kLineKind[] = {{InternalLineKind::Fold, "fold"},
                                                                  {InternalLineKind::Quilting, "quilting"},
                                                                  {InternalLineKind::Topstitch, "topstitch"}};
constexpr std::pair<TrimKind, const char*> kTrimKind[] = {
    {TrimKind::Rivet, "rivet"}, {TrimKind::Button, "button"}, {TrimKind::ZipperStrip, "zipperStrip"}};
constexpr std::pair<BaseBody, const char*> kBaseBody[] = {{BaseBody::Female, "female"}, {BaseBody::Male, "male"}};
constexpr std::pair<Measurement, const char*> kMeasurement[] = {
    {Measurement::Height, "height"},
    {Measurement::Bust, "bust"},
    {Measurement::Underbust, "underbust"},
    {Measurement::Waist, "waist"},
    {Measurement::Hip, "hip"},
    {Measurement::ShoulderWidth, "shoulderWidth"},
    {Measurement::ArmLength, "armLength"},
    {Measurement::Inseam, "inseam"},
    {Measurement::Neck, "neck"},
    {Measurement::HeadCircumference, "headCircumference"}};
constexpr std::pair<OutfitSlot, const char*> kOutfitSlot[] = {
    {OutfitSlot::Top, "top"}, {OutfitSlot::Bottom, "bottom"}, {OutfitSlot::Headwear, "headwear"}};
constexpr std::pair<SimQuality, const char*> kSimQuality[] = {
    {SimQuality::Draft, "draft"}, {SimQuality::Standard, "standard"}, {SimQuality::Fine, "fine"}};
constexpr std::pair<LightingPreset, const char*> kLighting[] = {{LightingPreset::StudioSoft, "studioSoft"},
                                                                {LightingPreset::StudioContrast, "studioContrast"},
                                                                {LightingPreset::OvercastOutdoor, "overcastOutdoor"}};
constexpr std::pair<DisplayUnits, const char*> kUnits[] = {{DisplayUnits::Centimeters, "centimeters"},
                                                           {DisplayUnits::Inches, "inches"}};

// Maps keyed by an enum become JSON objects keyed by the enum's string.
template <class K, class V>
json enumMapToJson(const std::map<K, V>& m) {
  json out = json::object();
  for (const auto& [k, v] : m) out[json(k).template get<std::string>()] = v;
  return out;
}

template <class K, class V>
std::map<K, V> enumMapFromJson(const json& j) {
  if (!j.is_object()) throw json::type_error::create(302, "expected an object", &j);
  std::map<K, V> out;
  for (const auto& [key, value] : j.items()) out[json(key).template get<K>()] = value.template get<V>();
  return out;
}

template <class T>
void optionalToJson(json& j, const char* key, const std::optional<T>& v) {
  if (v) j[key] = *v;
}

template <class T>
void optionalFromJson(const json& j, const char* key, std::optional<T>& v) {
  if (j.contains(key)) {
    v = j.at(key).template get<T>();
  } else {
    v.reset();
  }
}

}  // namespace

#define DRAPE_ENUM_JSON(T, TABLE)                                  \
  void to_json(json& j, const T& v) { enumToJson(j, v, TABLE); }   \
  void from_json(const json& j, T& v) { enumFromJson(j, v, TABLE); }

DRAPE_ENUM_JSON(BodyRegion, kBodyRegion)
DRAPE_ENUM_JSON(SeamKind, kSeamKind)
DRAPE_ENUM_JSON(InternalLineKind, kLineKind)
DRAPE_ENUM_JSON(TrimKind, kTrimKind)
DRAPE_ENUM_JSON(BaseBody, kBaseBody)
DRAPE_ENUM_JSON(Measurement, kMeasurement)
DRAPE_ENUM_JSON(OutfitSlot, kOutfitSlot)
DRAPE_ENUM_JSON(SimQuality, kSimQuality)
DRAPE_ENUM_JSON(LightingPreset, kLighting)
DRAPE_ENUM_JSON(DisplayUnits, kUnits)
#undef DRAPE_ENUM_JSON

void to_json(json& j, const RegionCylinder& v) {
  j = {{"origin", v.origin}, {"axis", v.axis}, {"e0", v.e0}, {"radius", v.radius}};
}
void from_json(const json& j, RegionCylinder& v) {
  j.at("origin").get_to(v.origin);
  j.at("axis").get_to(v.axis);
  j.at("e0").get_to(v.e0);
  j.at("radius").get_to(v.radius);
}

void to_json(json& j, const BodyRegions& v) { j = {{"cylinders", enumMapToJson(v.cylinders)}}; }
void from_json(const json& j, BodyRegions& v) {
  v.cylinders = enumMapFromJson<BodyRegion, RegionCylinder>(j.at("cylinders"));
}

void to_json(json& j, const Outline& v) {
  j = json::array();
  for (std::size_t i = 0; i < v.size(); ++i) {
    const auto& s = v.segment(i);
    j.push_back({{"name", v.name(i)}, {"p", {s.p0, s.p1, s.p2, s.p3}}});
  }
}
void from_json(const json& j, Outline& v) {
  if (!j.is_array()) throw json::type_error::create(302, "expected an array of segments", &j);
  Outline out;
  for (const auto& seg : j) {
    const auto& p = seg.at("p");
    if (!p.is_array() || p.size() != 4) throw json::type_error::create(302, "expected 4 control points", &p);
    out.add(seg.at("name").get<std::string>(),
            CubicBezier{p.at(0).get<Vec2d>(), p.at(1).get<Vec2d>(), p.at(2).get<Vec2d>(), p.at(3).get<Vec2d>()});
  }
  v = std::move(out);
}

void to_json(json& j, const Placement& v) {
  j = {{"region", v.region},   {"centerAngle", v.centerAngle}, {"axialAtOrigin", v.axialAtOrigin},
       {"axialSign", v.axialSign}, {"offset", v.offset}};
  optionalToJson(j, "wrapRadius", v.wrapRadius);
}
void from_json(const json& j, Placement& v) {
  j.at("region").get_to(v.region);
  j.at("centerAngle").get_to(v.centerAngle);
  j.at("axialAtOrigin").get_to(v.axialAtOrigin);
  j.at("axialSign").get_to(v.axialSign);
  j.at("offset").get_to(v.offset);
  optionalFromJson(j, "wrapRadius", v.wrapRadius);
}

void to_json(json& j, const LayerRelation& v) { j = {{"basePiece", v.basePiece}, {"order", v.order}}; }
void from_json(const json& j, LayerRelation& v) {
  j.at("basePiece").get_to(v.basePiece);
  j.at("order").get_to(v.order);
}

void to_json(json& j, const ElasticSpec& v) { j = {{"factor", v.factor}, {"direction", v.direction}}; }
void from_json(const json& j, ElasticSpec& v) {
  j.at("factor").get_to(v.factor);
  j.at("direction").get_to(v.direction);
}

void to_json(json& j, const PatternPiece& v) {
  j = {{"id", v.id},
       {"name", v.name},
       {"outline", v.outline},
       {"grainline", v.grainline},
       {"fabricSlot", v.fabricSlot},
       {"placement", v.placement},
       {"mirrored", v.mirrored}};
  optionalToJson(j, "layer", v.layer);
  optionalToJson(j, "elastic", v.elastic);
}
void from_json(const json& j, PatternPiece& v) {
  j.at("id").get_to(v.id);
  j.at("name").get_to(v.name);
  j.at("outline").get_to(v.outline);
  j.at("grainline").get_to(v.grainline);
  j.at("fabricSlot").get_to(v.fabricSlot);
  j.at("placement").get_to(v.placement);
  j.at("mirrored").get_to(v.mirrored);
  optionalFromJson(j, "layer", v.layer);
  optionalFromJson(j, "elastic", v.elastic);
}

void to_json(json& j, const EdgeRef& v) {
  j = {{"piece", v.piece}, {"start", v.start}, {"end", v.end}, {"reversed", v.reversed}};
}
void from_json(const json& j, EdgeRef& v) {
  j.at("piece").get_to(v.piece);
  j.at("start").get_to(v.start);
  j.at("end").get_to(v.end);
  j.at("reversed").get_to(v.reversed);
}

void to_json(json& j, const Seam& v) {
  j = {{"id", v.id}, {"a", v.a}, {"b", v.b}, {"ease", v.ease}, {"kind", v.kind}};
}
void from_json(const json& j, Seam& v) {
  j.at("id").get_to(v.id);
  j.at("a").get_to(v.a);
  j.at("b").get_to(v.b);
  j.at("ease").get_to(v.ease);
  j.at("kind").get_to(v.kind);
}

void to_json(json& j, const Color& v) { j = json::array({v.r, v.g, v.b}); }
void from_json(const json& j, Color& v) {
  if (!j.is_array() || j.size() != 3) throw json::type_error::create(302, "expected an array of 3 numbers", &j);
  v.r = j.at(0).get<float>();
  v.g = j.at(1).get<float>();
  v.b = j.at(2).get<float>();
}

void to_json(json& j, const InternalLine& v) {
  j = {{"piece", v.piece},
       {"polyline", v.polyline},
       {"kind", v.kind},
       {"foldAngle", v.foldAngle},
       {"stiffnessMultiplier", v.stiffnessMultiplier},
       {"topstitchOffset", v.topstitchOffset},
       {"stitchesPerInch", v.stitchesPerInch},
       {"threadColor", v.threadColor},
       {"threadThickness", v.threadThickness}};
}
void from_json(const json& j, InternalLine& v) {
  j.at("piece").get_to(v.piece);
  j.at("polyline").get_to(v.polyline);
  j.at("kind").get_to(v.kind);
  j.at("foldAngle").get_to(v.foldAngle);
  j.at("stiffnessMultiplier").get_to(v.stiffnessMultiplier);
  j.at("topstitchOffset").get_to(v.topstitchOffset);
  j.at("stitchesPerInch").get_to(v.stitchesPerInch);
  j.at("threadColor").get_to(v.threadColor);
  j.at("threadThickness").get_to(v.threadThickness);
}

void to_json(json& j, const Trim& v) {
  j = {{"kind", v.kind}, {"piece", v.piece}, {"position", v.position},
       {"seam", v.seam}, {"size", v.size},   {"material", v.material}};
}
void from_json(const json& j, Trim& v) {
  j.at("kind").get_to(v.kind);
  j.at("piece").get_to(v.piece);
  j.at("position").get_to(v.position);
  j.at("seam").get_to(v.seam);
  j.at("size").get_to(v.size);
  j.at("material").get_to(v.material);
}

void to_json(json& j, const Pattern& v) {
  j = {{"pieces", v.pieces}, {"seams", v.seams}, {"internalLines", v.internalLines}, {"trims", v.trims}};
}
void from_json(const json& j, Pattern& v) {
  j.at("pieces").get_to(v.pieces);
  j.at("seams").get_to(v.seams);
  j.at("internalLines").get_to(v.internalLines);
  j.at("trims").get_to(v.trims);
}

void to_json(json& j, const FabricPhysical& v) {
  j = {{"stretchWarp", v.stretchWarp},
       {"stretchWeft", v.stretchWeft},
       {"stretchBias", v.stretchBias},
       {"strainLimitWarp", v.strainLimitWarp},
       {"strainLimitWeft", v.strainLimitWeft},
       {"bendWarp", v.bendWarp},
       {"bendWeft", v.bendWeft},
       {"weight", v.weight},
       {"thickness", v.thickness},
       {"friction", v.friction},
       {"damping", v.damping}};
}
void from_json(const json& j, FabricPhysical& v) {
  j.at("stretchWarp").get_to(v.stretchWarp);
  j.at("stretchWeft").get_to(v.stretchWeft);
  j.at("stretchBias").get_to(v.stretchBias);
  j.at("strainLimitWarp").get_to(v.strainLimitWarp);
  j.at("strainLimitWeft").get_to(v.strainLimitWeft);
  j.at("bendWarp").get_to(v.bendWarp);
  j.at("bendWeft").get_to(v.bendWeft);
  j.at("weight").get_to(v.weight);
  j.at("thickness").get_to(v.thickness);
  j.at("friction").get_to(v.friction);
  j.at("damping").get_to(v.damping);
}

void to_json(json& j, const FabricVisual& v) {
  j = {{"baseColorTexture", v.baseColorTexture},
       {"normalTexture", v.normalTexture},
       {"roughnessTexture", v.roughnessTexture},
       {"textureScale", v.textureScale},
       {"sheenColor", v.sheenColor},
       {"sheenRoughness", v.sheenRoughness},
       {"subsurfaceColor", v.subsurfaceColor},
       {"insideDarkening", v.insideDarkening}};
}
void from_json(const json& j, FabricVisual& v) {
  j.at("baseColorTexture").get_to(v.baseColorTexture);
  j.at("normalTexture").get_to(v.normalTexture);
  j.at("roughnessTexture").get_to(v.roughnessTexture);
  j.at("textureScale").get_to(v.textureScale);
  j.at("sheenColor").get_to(v.sheenColor);
  j.at("sheenRoughness").get_to(v.sheenRoughness);
  j.at("subsurfaceColor").get_to(v.subsurfaceColor);
  j.at("insideDarkening").get_to(v.insideDarkening);
}

void to_json(json& j, const Fabric& v) {
  j = {{"id", v.id}, {"name", v.name}, {"physical", v.physical}, {"visual", v.visual}};
}
void from_json(const json& j, Fabric& v) {
  j.at("id").get_to(v.id);
  j.at("name").get_to(v.name);
  j.at("physical").get_to(v.physical);
  j.at("visual").get_to(v.visual);
}

void to_json(json& j, const ChartRef& v) { j = {{"chart", v.chart}, {"size", v.size}}; }
void from_json(const json& j, ChartRef& v) {
  j.at("chart").get_to(v.chart);
  j.at("size").get_to(v.size);
}

void to_json(json& j, const AvatarParams& v) {
  j = {{"base", v.base}, {"targets", enumMapToJson(v.targets)}, {"solvedWeights", v.solvedWeights}};
  optionalToJson(j, "chartRef", v.chartRef);
}
void from_json(const json& j, AvatarParams& v) {
  j.at("base").get_to(v.base);
  v.targets = enumMapFromJson<Measurement, double>(j.at("targets"));
  j.at("solvedWeights").get_to(v.solvedWeights);
  optionalFromJson(j, "chartRef", v.chartRef);
}

void to_json(json& j, const SizeChart& v) {
  j = {{"id", v.id},       {"name", v.name}, {"base", v.base},
       {"sizes", v.sizes}, {"values", enumMapToJson(v.values)}};
}
void from_json(const json& j, SizeChart& v) {
  j.at("id").get_to(v.id);
  j.at("name").get_to(v.name);
  j.at("base").get_to(v.base);
  j.at("sizes").get_to(v.sizes);
  v.values = enumMapFromJson<Measurement, std::vector<double>>(j.at("values"));
}

void to_json(json& j, const GarmentInstance& v) {
  j = {{"id", v.id},
       {"generatorId", v.generatorId},
       {"generatorVersion", v.generatorVersion},
       {"slot", v.slot},
       {"size", v.size},
       {"specOverrides", v.specOverrides},
       {"pattern", v.pattern},
       {"fabricBySlot", v.fabricBySlot},
       {"pieceColors", v.pieceColors},
       {"quality", v.quality}};
  optionalToJson(j, "garmentColor", v.garmentColor);
}
void from_json(const json& j, GarmentInstance& v) {
  j.at("id").get_to(v.id);
  j.at("generatorId").get_to(v.generatorId);
  j.at("generatorVersion").get_to(v.generatorVersion);
  j.at("slot").get_to(v.slot);
  j.at("size").get_to(v.size);
  j.at("specOverrides").get_to(v.specOverrides);
  j.at("pattern").get_to(v.pattern);
  j.at("fabricBySlot").get_to(v.fabricBySlot);
  j.at("pieceColors").get_to(v.pieceColors);
  j.at("quality").get_to(v.quality);
  optionalFromJson(j, "garmentColor", v.garmentColor);
}

void to_json(json& j, const Camera& v) {
  j = {{"position", v.position}, {"target", v.target}, {"fovYDegrees", v.fovYDegrees}};
}
void from_json(const json& j, Camera& v) {
  j.at("position").get_to(v.position);
  j.at("target").get_to(v.target);
  j.at("fovYDegrees").get_to(v.fovYDegrees);
}

void to_json(json& j, const Scene& v) { j = {{"lighting", v.lighting}, {"camera", v.camera}}; }
void from_json(const json& j, Scene& v) {
  j.at("lighting").get_to(v.lighting);
  j.at("camera").get_to(v.camera);
}

void to_json(json& j, const ProjectMetadata& v) {
  j = {{"name", v.name}, {"created", v.created}, {"modified", v.modified}};
}
void from_json(const json& j, ProjectMetadata& v) {
  j.at("name").get_to(v.name);
  j.at("created").get_to(v.created);
  j.at("modified").get_to(v.modified);
}

void to_json(json& j, const Project& v) {
  j = {{"meta", v.meta},
       {"units", v.units},
       {"avatar", v.avatar},
       {"sizeCharts", v.sizeCharts},
       {"garments", v.garments},
       {"customFabrics", v.customFabrics},
       {"scene", v.scene}};
}
void from_json(const json& j, Project& v) {
  j.at("meta").get_to(v.meta);
  j.at("units").get_to(v.units);
  j.at("avatar").get_to(v.avatar);
  j.at("sizeCharts").get_to(v.sizeCharts);
  j.at("garments").get_to(v.garments);
  j.at("customFabrics").get_to(v.customFabrics);
  j.at("scene").get_to(v.scene);
}

}  // namespace drape
