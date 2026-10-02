#pragma once

#include "drape/core/fabric.hpp"

// Fixed fabrics for tests and developer tools (SI units). Not the product presets (Phase D).
namespace drape::testdata {

FabricPhysical testJersey();
FabricPhysical testRib();
FabricPhysical testStiff();
FabricPhysical testAniso();

}  // namespace drape::testdata
