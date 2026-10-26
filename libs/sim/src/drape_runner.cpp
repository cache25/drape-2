#include "drape/sim/drape_runner.hpp"

#include "drape/sim/drape_session.hpp"

namespace drape::sim {

DrapeResult drapeToRest(Solver& solver, const std::vector<GarmentId>& garments, const DrapeOptions& o) {
  DrapeSession session(solver, garments, o);
  while (!session.finished()) session.advance();
  return session.result();
}

}  // namespace drape::sim
