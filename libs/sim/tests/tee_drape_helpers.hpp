#pragma once

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <map>
#include <string>

#include "drape/avatar/mannequin.hpp"
#include "drape/core/test_fabrics.hpp"
#include "drape/garment/sim_mesh_builder.hpp"
#include "drape/garment/spec_table.hpp"
#include "drape/sim/cpu_solver.hpp"
#include "drape/sim/drape_runner.hpp"
#include "drape/sim/scene_builder.hpp"
#include "drape/sim/solver_factory.hpp"
#include "drape/session/tee_scene.hpp"
#include "sim_test_utils.hpp"

namespace drape::sim::testing {

using session::teeGenerator;

inline const std::map<std::string, FabricPhysical>& teeFabrics() {
  static const std::map<std::string, FabricPhysical> f{{"body", testdata::testJersey()}, {"rib", testdata::testRib()}};
  return f;
}

inline Pattern teePattern(const std::string& size, avatar::BodyType body = avatar::BodyType::Male) {
  const garment::GarmentGenerator& g = teeGenerator(body);
  return g.generate(garment::resolveSpec(g.specTable(), size));
}

// The Task 16 pipeline on a mannequin: generate the tee, mesh, place, build and drape to rest. The hem is the
// front and back vertices on the pattern's y = 0 edge; its mean height must lie within 3 cm of the body-aware
// target below.
inline void expectTeeDrapes(const std::string& size, SimQuality q, avatar::BodyType body = avatar::BodyType::Male,
                            SolverBackend backend = SolverBackend::Cpu) {
  const auto ts = session::makeTeeScene(body, q, size);
  const auto& lm = ts->mannequin.landmarks.points;
  // L is the body length of the tee this body wears.
  const double bodyLength = garment::resolveSpec(teeGenerator(body).specTable(), size).at("body_length");
  // Hem target = mean HPS height - body length + take-up. The front does not fall vertically from the HPS: it
  // runs straight from the HPS over the chest apex, which uses up cloth length. take-up = |chest - hps_left|
  // (3D distance) - (hps_left.y - chest.y), the extra length of that slanted run over its height
  // (about 0.040 m male, 0.049 m female).
  const Vec3d& hps = lm.at("hps_left");
  const Vec3d& chest = lm.at("chest");
  const double takeUp = (chest - hps).norm() - (hps.y() - chest.y());
  const double hemTarget = 0.5 * (lm.at("hps_left").y() + lm.at("hps_right").y()) - bodyLength + takeUp;

  const Pattern& pattern = ts->pattern;
  const SimMesh& mesh = ts->mesh;
  const SimScene& scene = ts->scene;
  const auto solverPtr = makeSolver(backend);
  Solver& solver = *solverPtr;
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
  const std::string label = size + " " + std::string(avatar::bodyName(body)) + " " + std::string(backendName(backend));
  std::printf("tee %s q%d: settled=%d failed=%d t=%.2fs assembly=%.2fs retries=%d seamGap=%.2fmm pen=%.2fmm "
              "hem=%.3fm target=%.3fm hemErr=%+.1fmm particles=%zu\n",
              label.c_str(), static_cast<int>(q), r.settled, r.failed, r.simulatedSeconds, r.assemblySeconds,
              r.retries, r.assemblyEndSeamGap * 1000, st.maxBodyPenetration * 1000, hem, hemTarget,
              (hem - hemTarget) * 1000, x.size());
  EXPECT_TRUE(r.settled) << label;
  EXPECT_FALSE(r.failed) << label << ": " << r.message;
  EXPECT_LT(r.assemblyEndSeamGap, 0.001) << label;
  EXPECT_LE(st.maxBodyPenetration, 0.002) << label;
  EXPECT_LE(std::abs(hem - hemTarget), 0.03) << label << ": hem " << hem << " m, target " << hemTarget << " m";
  EXPECT_LE(r.simulatedSeconds, 15.0) << label;
}

}  // namespace drape::sim::testing
