#pragma once

#include <vector>

#include "drape/core/body_regions.hpp"
#include "drape/core/triangle_mesh.hpp"

namespace drape::avatar {

// Static stand-in body for Phase A: torso, neck, head and A-pose arms as separate closed components.
struct TestBody {
  std::vector<TriangleMesh> components;
  BodyRegions regions;
};

TestBody makeTestBody();

}  // namespace drape::avatar
