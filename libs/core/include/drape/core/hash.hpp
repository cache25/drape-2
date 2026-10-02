#pragma once

#include <cstdint>
#include <string_view>

namespace drape {

// 64-bit FNV-1a (offset basis 0xcbf29ce484222325, prime 0x100000001b3).
std::uint64_t fnv1a64(std::string_view bytes);

}  // namespace drape
