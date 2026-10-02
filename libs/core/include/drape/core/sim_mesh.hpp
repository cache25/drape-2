#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "drape/core/math.hpp"

namespace drape {

struct SeamPair {
  std::uint32_t a = 0, b = 0;  // global vertex indices on side A and side B
  std::uint32_t seam = 0;      // index into Pattern::seams
  std::uint32_t index = 0;     // 0..N along the seam
};

// Triangle mesh of a pattern in pattern space; vertices of all pieces are concatenated in Pattern order.
struct SimMesh {
  std::vector<Vec2d> rest;                              // pattern-space position (m)
  std::vector<std::uint32_t> vertexPiece;               // index into Pattern::pieces
  std::vector<std::array<std::uint32_t, 3>> triangles;  // CCW in pattern space
  std::vector<std::uint32_t> trianglePiece;
  std::vector<double> vertexArea;      // 1/3 of incident triangle rest areas (m²)
  std::vector<std::uint8_t> boundary;  // 1 if the vertex lies on its piece outline
  std::vector<Vec2d> pieceGrain;       // unit grainline per piece index
  std::vector<SeamPair> seamPairs;     // grouped by seam (Pattern order), ascending index
};

}  // namespace drape
