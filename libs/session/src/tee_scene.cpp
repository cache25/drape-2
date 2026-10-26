#include "drape/session/tee_scene.hpp"

#include <map>
#include <span>
#include <utility>

#include "drape/avatar/collision_bake.hpp"
#include "drape/core/test_fabrics.hpp"
#include "drape/garment/placement.hpp"
#include "drape/garment/sim_mesh_builder.hpp"
#include "drape/garment/spec_table.hpp"
#include "drape/garment/test_tee.hpp"
#include "drape/sim/scene_builder.hpp"

namespace drape::session {

const garment::GarmentGenerator& teeGenerator(avatar::BodyType body) {
  static const garment::TestTeeGenerator mens;
  static const garment::WomensTestTeeGenerator womens;
  if (body == avatar::BodyType::Female) return womens;
  return mens;
}

std::unique_ptr<const TeeScene> makeTeeScene(avatar::BodyType body, SimQuality quality, const std::string& size,
                                             const std::optional<std::filesystem::path>& mannequinDir) {
  auto ts = std::make_unique<TeeScene>();
  ts->bodyType = body;
  ts->quality = quality;
  ts->mannequin = mannequinDir ? avatar::loadMannequin(body, *mannequinDir) : avatar::loadMannequin(body);
  ts->field = avatar::bakeCollisionField(std::span<const TriangleMesh>(&ts->mannequin.mesh, 1));
  const garment::GarmentGenerator& generator = teeGenerator(body);
  ts->pattern = generator.generate(garment::resolveSpec(generator.specTable(), size));
  ts->mesh = garment::buildSimMesh(ts->pattern, garment::particleDistance(quality));
  auto placed = garment::placePattern(ts->pattern, ts->mesh, ts->mannequin.regions);
  static const std::map<std::string, FabricPhysical> fabrics{{"body", testdata::testJersey()},
                                                             {"rib", testdata::testRib()}};
  ts->scene.body = &ts->field;
  ts->scene.garments.push_back(sim::makeSimGarment("tee", ts->pattern, ts->mesh, std::move(placed), fabrics));
  return ts;
}

}  // namespace drape::session
