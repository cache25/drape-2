#include "drape/core/hash.hpp"

namespace drape {

std::uint64_t fnv1a64(std::string_view bytes) {
  std::uint64_t h = 0xcbf29ce484222325ull;
  for (unsigned char c : bytes) {
    h ^= c;
    h *= 0x100000001b3ull;
  }
  return h;
}

}  // namespace drape
