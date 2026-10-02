#pragma once

#include <map>
#include <stdexcept>
#include <string>

#include "drape/core/pattern.hpp"
#include "drape/core/project.hpp"
#include "drape/garment/spec_table.hpp"

namespace drape::garment {

class GeneratorError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// A library garment: spec values for one size in, a body-independent Pattern out (spec 7.1).
class GarmentGenerator {
 public:
  virtual ~GarmentGenerator() = default;
  virtual std::string id() const = 0;
  virtual int version() const = 0;
  virtual OutfitSlot slot() const = 0;
  virtual SpecTable specTable() const = 0;
  virtual Pattern generate(const std::map<std::string, double>& spec) const = 0;  // throws GeneratorError
};

}  // namespace drape::garment
