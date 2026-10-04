#include "tee_drape_helpers.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

TEST(TeeDrapeSlow, AllSizesDraft) {
  for (const auto& size : garment::kStandardSizes) expectTeeDrapes(size, SimQuality::Draft);
}

TEST(TeeDrapeSlow, StandardOnTestBody) { expectTeeDrapes("M", SimQuality::Standard); }
