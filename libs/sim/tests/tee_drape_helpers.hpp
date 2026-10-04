#pragma once

#include <gtest/gtest.h>

#include <cstdio>
#include <map>
#include <string>

#include "drape/avatar/collision_bake.hpp"
#include "drape/avatar/test_body.hpp"
#include "drape/core/test_fabrics.hpp"
#include "drape/garment/placement.hpp"
#include "drape/garment/sim_mesh_builder.hpp"
#include "drape/garment/test_tee.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "drape/sim/drape_runner.hpp"
#include "drape/sim/scene_builder.hpp"
#include "sim_test_utils.hpp"

namespace drape::sim::testing {

inline const std::map<std::string, FabricPhysical>& teeFabrics() {
  static const std::map<std::string, FabricPhysical> f{{"body", testdata::testJersey()}, {"rib", testdata::testRib()}};
  return f;
}

inline Pattern teePattern(const std::string& size) {
  garment::TestTeeGenerator g;
  return g.generate(garment::resolveSpec(g.specTable(), size));
}

struct BodyAndField {
  avatar::TestBody body;
  CollisionField field;
};

inline const BodyAndField& testBody() {
  static const BodyAndField b = [] {
    BodyAndField out;
    out.body = avatar::makeTestBody();
    out.field = avatar::bakeCollisionField(out.body.components);
    return out;
  }();
  return b;
}

// The Task 16 pipeline: bake the test body, generate the tee, mesh, place, build and drape to rest. The hem is the
// front and back vertices on the pattern's y = 0 edge.
inline void expectTeeDrapes(const std::string& size, SimQuality q) {
  const Pattern pattern = teePattern(size);
  SimMesh mesh = garment::buildSimMesh(pattern, garment::particleDistance(q));
  auto placed = garment::placePattern(pattern, mesh, testBody().body.regions);
  SimScene scene;
  scene.body = &testBody().field;
  scene.garments.push_back(makeSimGarment("tee", pattern, mesh, placed, teeFabrics()));
  CpuSolver solver;
  solver.build(scene);
  DrapeOptions o;
  o.substeps = substepsFor(q);
  o.iterations = iterationsFor(q);
  const DrapeResult r = drapeToRest(solver, {"tee"}, o);
  const auto st = solver.stats();
  const auto x = positions(solver);
  double hem = 0;
  int hemCount = 0;
  for (std::uint32_t v = 0; v < mesh.rest.size(); ++v) {
    const auto& piece = pattern.pieces[mesh.vertexPiece[v]].id;
    if ((piece == "front" || piece == "back") && mesh.rest[v].y() < 1e-9) {
      hem += x[v].y();
      ++hemCount;
    }
  }
  ASSERT_GT(hemCount, 0);
  hem /= hemCount;
  std::printf("tee %s q%d: settled=%d failed=%d t=%.2fs retries=%d seamGap=%.2fmm pen=%.2fmm hem=%.3fm particles=%zu\n",
              size.c_str(), static_cast<int>(q), r.settled, r.failed, r.simulatedSeconds, r.retries,
              r.assemblyEndSeamGap * 1000, st.maxBodyPenetration * 1000, hem, x.size());
  EXPECT_TRUE(r.settled) << size;
  EXPECT_FALSE(r.failed) << r.message;
  EXPECT_LT(r.assemblyEndSeamGap, 0.001) << size;
  EXPECT_LE(st.maxBodyPenetration, 0.002) << size;
  EXPECT_GE(hem, 0.65) << size;
  EXPECT_LE(hem, 0.85) << size;
  EXPECT_LE(r.simulatedSeconds, 15.0) << size;
}

}  // namespace drape::sim::testing
