// drape_dump: drapes the test tee on the test body (the Phase A pipeline) and writes body and garment to OBJ.
#include <algorithm>
#include <cstdio>
#include <exception>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drape/avatar/collision_bake.hpp"
#include "drape/avatar/test_body.hpp"
#include "drape/core/test_fabrics.hpp"
#include "drape/garment/placement.hpp"
#include "drape/garment/sim_mesh_builder.hpp"
#include "drape/garment/spec_table.hpp"
#include "drape/garment/test_tee.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "drape/sim/drape_runner.hpp"
#include "drape/sim/scene_builder.hpp"
#include "obj_writer.hpp"

namespace {

constexpr const char* kUsage =
    "usage: drape_dump --out <file.obj> [--size XS|S|M|L|XL|XXL] [--quality draft|standard|fine]\n";

struct Args {
  std::string out;
  std::string size = "M";
  drape::SimQuality quality = drape::SimQuality::Draft;
};

bool parse(int argc, char** argv, Args& a) {
  const std::map<std::string, drape::SimQuality> qualities{{"draft", drape::SimQuality::Draft},
                                                           {"standard", drape::SimQuality::Standard},
                                                           {"fine", drape::SimQuality::Fine}};
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (i + 1 >= argc) return false;
    const std::string value = argv[++i];
    if (key == "--out") {
      a.out = value;
    } else if (key == "--size") {
      const auto& sizes = drape::garment::kStandardSizes;
      if (std::find(sizes.begin(), sizes.end(), value) == sizes.end()) return false;
      a.size = value;
    } else if (key == "--quality") {
      const auto it = qualities.find(value);
      if (it == qualities.end()) return false;
      a.quality = it->second;
    } else {
      return false;
    }
  }
  return !a.out.empty();
}

}  // namespace

int main(int argc, char** argv) {
  using namespace drape;
  Args args;
  if (!parse(argc, argv, args)) {
    std::fputs(kUsage, stderr);
    return 2;
  }
  try {
    const avatar::TestBody body = avatar::makeTestBody();
    const CollisionField field = avatar::bakeCollisionField(body.components);
    garment::TestTeeGenerator generator;
    const Pattern pattern = generator.generate(garment::resolveSpec(generator.specTable(), args.size));
    SimMesh mesh = garment::buildSimMesh(pattern, garment::particleDistance(args.quality));
    auto placed = garment::placePattern(pattern, mesh, body.regions);
    const std::map<std::string, FabricPhysical> fabrics{{"body", testdata::testJersey()}, {"rib", testdata::testRib()}};

    sim::SimScene scene;
    scene.body = &field;
    scene.garments.push_back(sim::makeSimGarment("tee", pattern, mesh, std::move(placed), fabrics));
    sim::CpuSolver solver;
    solver.build(scene);
    sim::DrapeOptions options;
    options.substeps = sim::substepsFor(args.quality);
    options.iterations = sim::iterationsFor(args.quality);
    const sim::DrapeResult result = sim::drapeToRest(solver, {"tee"}, options);
    const sim::SimStats stats = solver.stats();

    std::vector<Vec3f> x(solver.particleCount());
    solver.readPositions(x);
    std::vector<std::pair<std::string, TriangleMesh>> objects;
    for (std::size_t c = 0; c < body.components.size(); ++c)
      objects.emplace_back("body_" + std::to_string(c), body.components[c]);
    TriangleMesh tee;
    for (const auto& p : x) tee.vertices.push_back(p.cast<double>());
    tee.triangles = mesh.triangles;
    objects.emplace_back("tee", std::move(tee));
    tools::writeObj(args.out, objects);

    std::printf("settled=%d failed=%d seconds=%.2f retries=%d seam_gap_mm=%.2f penetration_mm=%.2f particles=%zu\n",
                result.settled ? 1 : 0, result.failed ? 1 : 0, result.simulatedSeconds, result.retries,
                result.assemblyEndSeamGap * 1000.0, std::max(0.0, stats.maxBodyPenetration) * 1000.0, x.size());
    return result.settled && !result.failed ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "drape_dump: %s\n", e.what());
    return 1;
  }
}
