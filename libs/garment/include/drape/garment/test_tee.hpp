#pragma once

#include "drape/garment/generator.hpp"

namespace drape::garment {

// Internal physics-baseline tee (not shipped in the library): front, back, two set-in sleeves, neck rib.
class TestTeeGenerator final : public GarmentGenerator {
 public:
  std::string id() const override { return "test_tee"; }
  int version() const override { return 1; }
  OutfitSlot slot() const override { return OutfitSlot::Top; }
  SpecTable specTable() const override;
  Pattern generate(const std::map<std::string, double>& spec) const override;
};

}  // namespace drape::garment
