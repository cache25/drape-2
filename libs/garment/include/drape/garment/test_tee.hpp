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

// Women's cut of the test tee: the same pieces, seams and generate() code with a women's basic crew-neck POM
// table (narrower shoulders and neck, shorter body, armhole and sleeve). Drape's female mannequin wears it; the
// men's shoulders would land 6 cm past hers and drop the sleeves onto her 45 degree arms.
class WomensTestTeeGenerator final : public GarmentGenerator {
 public:
  std::string id() const override { return "test_tee_womens"; }
  int version() const override { return 1; }
  OutfitSlot slot() const override { return OutfitSlot::Top; }
  SpecTable specTable() const override;
  Pattern generate(const std::map<std::string, double>& spec) const override;
};

}  // namespace drape::garment
