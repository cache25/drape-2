#include "drape/core/pattern.hpp"

#include <stdexcept>
#include <string>

namespace drape {

const PatternPiece* Pattern::findPiece(std::string_view id) const {
  for (const auto& p : pieces) {
    if (p.id == id) return &p;
  }
  return nullptr;
}

EdgeRef segmentEdge(const PatternPiece& piece, std::string_view first, std::string_view last, bool reversed) {
  const auto i = piece.outline.find(first);
  const auto k = piece.outline.find(last);
  if (!i || !k) {
    throw std::invalid_argument("piece '" + piece.id + "' has no segment '" + std::string(!i ? first : last) + "'");
  }
  if (*k < *i) {
    throw std::invalid_argument("piece '" + piece.id + "': segment '" + std::string(last) + "' comes before '" +
                                std::string(first) + "'");
  }
  return EdgeRef{piece.id, piece.outline.segmentStart(*i), piece.outline.segmentEnd(*k), reversed};
}

}  // namespace drape
