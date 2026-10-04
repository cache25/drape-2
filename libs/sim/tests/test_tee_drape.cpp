#include "tee_drape_helpers.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

TEST(SceneBuilder, ResolvesFabricPerPiece) {
  const Pattern p = teePattern("M");
  SimMesh mesh = garment::buildSimMesh(p, 0.02);
  std::vector<Vec3d> pos(mesh.rest.size(), Vec3d::Zero());
  const SimGarment g = makeSimGarment("tee", p, mesh, pos, teeFabrics());
  ASSERT_EQ(g.pieceFabric.size(), 5u);
  EXPECT_EQ(g.pieceFabric[0], testdata::testJersey());  // front
  EXPECT_EQ(g.pieceFabric[4], testdata::testRib());     // neck_rib
  EXPECT_EQ(g.id, "tee");
}

TEST(SceneBuilder, MissingSlotThrows) {
  const Pattern p = teePattern("M");
  SimMesh mesh = garment::buildSimMesh(p, 0.02);
  std::vector<Vec3d> pos(mesh.rest.size(), Vec3d::Zero());
  try {
    makeSimGarment("tee", p, mesh, pos, {{"body", testdata::testJersey()}});
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("rib"), std::string::npos) << e.what();
  }
}

TEST(TeeDrape, DraftOnTestBody) { expectTeeDrapes("M", SimQuality::Draft); }
