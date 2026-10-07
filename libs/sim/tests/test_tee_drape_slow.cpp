#include "tee_drape_helpers.hpp"

using namespace drape;
using namespace drape::sim;
using namespace drape::sim::testing;

TEST(TeeDrapeSlow, AllSizesDraftMale) {
  for (const auto& size : garment::kStandardSizes) expectTeeDrapes(size, SimQuality::Draft, avatar::BodyType::Male);
}

TEST(TeeDrapeSlow, AllSizesDraftFemale) {
  for (const auto& size : garment::kStandardSizes) expectTeeDrapes(size, SimQuality::Draft, avatar::BodyType::Female);
}

TEST(TeeDrapeSlow, StandardOnMaleMannequin) { expectTeeDrapes("M", SimQuality::Standard, avatar::BodyType::Male); }
