#pragma once

#include <map>
#include <string>
#include <vector>

namespace drape::garment {

inline const std::vector<std::string> kStandardSizes{"XS", "S", "M", "L", "XL", "XXL"};

struct PomSpec {
  std::string key;
  std::string label;
  double base = 0;          // value at the base size (m)
  double gradePerSize = 0;  // change per size step (m)
};

struct SpecTable {
  std::string baseSize = "M";
  std::vector<PomSpec> poms;
};

using SpecOverrides = std::map<std::string, std::map<std::string, double>>;  // size -> POM key -> value (m)

// value = base + gradePerSize * (index(size) - index(baseSize)) unless overridden.
// Unknown size -> std::invalid_argument.
std::map<std::string, double> resolveSpec(const SpecTable& table, const std::string& size,
                                          const SpecOverrides& overrides = {});

}  // namespace drape::garment
