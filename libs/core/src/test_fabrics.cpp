#include "drape/core/test_fabrics.hpp"

namespace drape::testdata {

FabricPhysical testJersey() { return {400, 200, 100, 0.5, 0.8, 2e-6, 1.5e-6, 0.160, 0.0006, 0.3, 0.3}; }
FabricPhysical testRib() { return {300, 100, 80, 0.8, 1.0, 3e-6, 2e-6, 0.280, 0.0010, 0.3, 0.3}; }
FabricPhysical testStiff() { return {20000, 20000, 5000, 0.02, 0.02, 100e-6, 100e-6, 0.450, 0.0008, 0.3, 1.0}; }
FabricPhysical testAniso() { return {20000, 20000, 5000, 0.02, 0.02, 100e-6, 40e-6, 0.450, 0.0008, 0.3, 1.0}; }

}  // namespace drape::testdata
