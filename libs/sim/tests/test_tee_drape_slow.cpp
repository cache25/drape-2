#include "backends.hpp"
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

// The tee drape checks on every GPU backend this machine has (spec Phase B 7.2).
TEST(TeeDrape, GpuMaleM) {
  if (gpuBackends().empty()) GTEST_SKIP() << "no GPU backend available";
  for (auto b : gpuBackends()) expectTeeDrapes("M", SimQuality::Draft, avatar::BodyType::Male, b);
}

TEST(TeeDrape, GpuFemaleM) {
  if (gpuBackends().empty()) GTEST_SKIP() << "no GPU backend available";
  for (auto b : gpuBackends()) expectTeeDrapes("M", SimQuality::Draft, avatar::BodyType::Female, b);
}
