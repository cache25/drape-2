#pragma once

#include <string>
#include <vector>

#include "drape/core/pattern.hpp"

namespace drape::garment {

struct ValidationIssue {
  std::string rule;
  std::string message;
};

// Structural pattern rules (spec 7.2). Empty result == valid.
std::vector<ValidationIssue> validatePattern(const Pattern& pattern);

}  // namespace drape::garment
