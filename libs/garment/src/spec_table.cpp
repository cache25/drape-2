#include "drape/garment/spec_table.hpp"

#include <algorithm>
#include <stdexcept>

namespace drape::garment {

namespace {
int sizeIndex(const std::string& size) {
  const auto it = std::find(kStandardSizes.begin(), kStandardSizes.end(), size);
  if (it == kStandardSizes.end()) throw std::invalid_argument("unknown size '" + size + "'");
  return static_cast<int>(it - kStandardSizes.begin());
}
}  // namespace

std::map<std::string, double> resolveSpec(const SpecTable& table, const std::string& size,
                                          const SpecOverrides& overrides) {
  const int steps = sizeIndex(size) - sizeIndex(table.baseSize);
  std::map<std::string, double> out;
  for (const auto& pom : table.poms) out[pom.key] = pom.base + pom.gradePerSize * steps;
  if (auto it = overrides.find(size); it != overrides.end()) {
    for (const auto& [key, value] : it->second) out[key] = value;
  }
  return out;
}

}  // namespace drape::garment
