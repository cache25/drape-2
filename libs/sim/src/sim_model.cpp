#include "sim_model.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

#include "bending.hpp"

namespace drape::sim {

std::size_t SimModel::garmentIndex(const GarmentId& id) const {
  for (std::size_t i = 0; i < garments.size(); ++i)
    if (garments[i].id == id) return i;
  throw std::out_of_range("unknown garment '" + id + "'");
}

ParticleRange SimModel::range(const GarmentId& id) const { return garments[garmentIndex(id)].range; }

double SimModel::hingeStiffness(std::size_t hgi) const {
  double B = 0;
  for (const auto& side : hingeSides[hgi]) {
    const auto& f = fabrics[side.fabric];
    B += 0.5 * edgeBendStiffness(bendingCoefficients(f.bendWarp, f.bendWeft), side.edgeDir, side.grain);
  }
  return B * hingeGeom[hgi];
}

double SimModel::seamTarget(const SeamSpring& sp) const {
  const auto& g = garments[sp.garment];
  if (g.phase != SimPhase::Assembling) return 0.0;
  return std::max(0.0, static_cast<double>(sp.startGap) - settings.assemblyClosingSpeed * g.assemblyTime);
}

std::uint32_t SimModel::slot(std::uint32_t row, std::uint32_t col) const {
  const auto begin = colIndex.begin() + rowStart[row];
  const auto end = colIndex.begin() + rowStart[row + 1];
  return static_cast<std::uint32_t>(std::lower_bound(begin, end, col) - colIndex.begin());
}

void SimModel::buildStructure() {
  const std::size_t n = x.size();
  std::vector<std::vector<std::uint32_t>> cols(n);
  auto couple = [&](const auto& vs) {
    for (auto a : vs)
      for (auto b : vs) cols[a].push_back(b);
  };
  for (std::size_t i = 0; i < n; ++i) cols[i].push_back(static_cast<std::uint32_t>(i));
  for (const auto& t : stretch) couple(t.v);
  for (const auto& h : hinges) couple(h.v);
  for (const auto& s : seams) couple(std::array<std::uint32_t, 2>{s.a, s.b});
  rowStart.assign(n + 1, 0);
  colIndex.clear();
  for (std::size_t i = 0; i < n; ++i) {
    std::sort(cols[i].begin(), cols[i].end());
    cols[i].erase(std::unique(cols[i].begin(), cols[i].end()), cols[i].end());
    colIndex.insert(colIndex.end(), cols[i].begin(), cols[i].end());
    rowStart[i + 1] = static_cast<std::uint32_t>(colIndex.size());
  }
  diagSlot.resize(n);
  for (std::uint32_t i = 0; i < n; ++i) diagSlot[i] = slot(i, i);
  auto slotsFor = [&](const auto& vs, auto& out) {
    const int m = static_cast<int>(vs.size());
    for (int a = 0; a < m; ++a)
      for (int b = 0; b < m; ++b) out[a * m + b] = slot(vs[a], vs[b]);
  };
  stretchSlots.resize(stretch.size());
  for (std::size_t k = 0; k < stretch.size(); ++k) slotsFor(stretch[k].v, stretchSlots[k]);
  hingeSlots.resize(hinges.size());
  for (std::size_t k = 0; k < hinges.size(); ++k) slotsFor(hinges[k].v, hingeSlots[k]);
  seamSlots.resize(seams.size());
  for (std::size_t k = 0; k < seams.size(); ++k) slotsFor(std::array<std::uint32_t, 2>{seams[k].a, seams[k].b}, seamSlots[k]);
  contactForce.assign(n, 0.0);
}

SimModel SimModel::build(const SimScene& scene) {
  SimModel next;
  next.settings = scene.settings;
  next.body = scene.body;
  for (const auto& sg : scene.garments) {
    if (sg.initialPositions.size() != sg.mesh.rest.size()) {
      throw std::invalid_argument("garment '" + sg.id + "': one initial position per mesh vertex required");
    }
    if (sg.pieceFabric.size() != sg.mesh.pieceGrain.size()) {
      throw std::invalid_argument("garment '" + sg.id + "': one fabric per pattern piece required");
    }
    Garment g;
    g.id = sg.id;
    g.range = {static_cast<std::uint32_t>(next.x.size()), static_cast<std::uint32_t>(sg.mesh.rest.size())};
    g.fabricOffset = static_cast<std::uint32_t>(next.fabrics.size());
    g.pieceCount = static_cast<std::uint32_t>(sg.pieceFabric.size());
    const auto gi = static_cast<std::uint32_t>(next.garments.size());
    const std::uint32_t off = g.range.offset;
    next.fabrics.insert(next.fabrics.end(), sg.pieceFabric.begin(), sg.pieceFabric.end());
    for (std::size_t v = 0; v < sg.mesh.rest.size(); ++v) {
      next.x.push_back(sg.initialPositions[v].cast<float>());
      next.vertexArea.push_back(sg.mesh.vertexArea[v]);
      next.pFabric.push_back(g.fabricOffset + sg.mesh.vertexPiece[v]);
      next.pGarment.push_back(gi);
    }
    auto triArea = [&](std::uint32_t t) {
      const auto& tri = sg.mesh.triangles[t];
      const Vec2d a = sg.mesh.rest[tri[1]] - sg.mesh.rest[tri[0]];
      const Vec2d b = sg.mesh.rest[tri[2]] - sg.mesh.rest[tri[0]];
      return 0.5 * std::abs(a.x() * b.y() - a.y() * b.x());
    };
    // Stretch terms in each piece's grain frame.
    for (std::uint32_t t = 0; t < sg.mesh.triangles.size(); ++t) {
      const auto& tri = sg.mesh.triangles[t];
      const auto piece = sg.mesh.trianglePiece[t];
      const Vec2d u = sg.mesh.pieceGrain[piece];
      const Vec2d vdir(-u.y(), u.x());
      Vec2d rr[3];
      for (int k = 0; k < 3; ++k) rr[k] = Vec2d(sg.mesh.rest[tri[k]].dot(u), sg.mesh.rest[tri[k]].dot(vdir));
      Eigen::Matrix2d dm;
      dm << (rr[1] - rr[0]).x(), (rr[2] - rr[0]).x(), (rr[1] - rr[0]).y(), (rr[2] - rr[0]).y();
      StretchTerm st;
      for (int k = 0; k < 3; ++k) st.v[k] = off + tri[k];
      st.dmInv = dm.inverse();
      st.area = 0.5 * std::abs(dm.determinant());
      st.fabric = g.fabricOffset + piece;
      next.stretch.push_back(st);
    }
    // Hinges: one per interior edge shared by two triangles of the same piece.
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::uint32_t>> edgeTris;
    for (std::uint32_t t = 0; t < sg.mesh.triangles.size(); ++t) {
      const auto& tri = sg.mesh.triangles[t];
      for (int k = 0; k < 3; ++k) edgeTris[std::minmax(tri[k], tri[(k + 1) % 3])].push_back(t);
    }
    auto opposite = [&](std::uint32_t t, std::uint32_t e0, std::uint32_t e1) {
      for (auto vtx : sg.mesh.triangles[t])
        if (vtx != e0 && vtx != e1) return vtx;
      return e0;
    };
    for (const auto& [edge, tris] : edgeTris) {
      if (tris.size() != 2 || sg.mesh.trianglePiece[tris[0]] != sg.mesh.trianglePiece[tris[1]]) continue;
      const auto piece = sg.mesh.trianglePiece[tris[0]];
      const Vec2d d = sg.mesh.rest[edge.second] - sg.mesh.rest[edge.first];
      HingeTerm hg;
      hg.v = {off + opposite(tris[0], edge.first, edge.second), off + opposite(tris[1], edge.first, edge.second),
              off + edge.first, off + edge.second};
      next.hinges.push_back(hg);
      const HingeSide side{g.fabricOffset + piece, d, sg.mesh.pieceGrain[piece]};
      next.hingeSides.push_back({side, side});
      next.hingeGeom.push_back(d.squaredNorm() / (triArea(tris[0]) + triArea(tris[1])));
    }
    // Seam springs, plus seam hinges across consecutive pairs so seams do not act as free hinges.
    auto triangleWithEdge = [&](std::uint32_t e0, std::uint32_t e1) -> std::int64_t {
      auto it = edgeTris.find(std::minmax(e0, e1));
      return (it == edgeTris.end() || it->second.size() != 1) ? -1 : it->second[0];
    };
    for (std::size_t k = 0; k < sg.mesh.seamPairs.size(); ++k) {
      const auto& sp = sg.mesh.seamPairs[k];
      SeamSpring spring;
      spring.a = off + sp.a;
      spring.b = off + sp.b;
      spring.garment = gi;
      spring.startGap = static_cast<float>((sg.initialPositions[sp.a] - sg.initialPositions[sp.b]).norm());
      next.seams.push_back(spring);
      if (k + 1 >= sg.mesh.seamPairs.size() || sg.mesh.seamPairs[k + 1].seam != sp.seam) continue;
      const auto& nx = sg.mesh.seamPairs[k + 1];
      const auto tA = triangleWithEdge(sp.a, nx.a);
      const auto tB = triangleWithEdge(sp.b, nx.b);
      if (tA < 0 || tB < 0) continue;
      const auto ta = static_cast<std::uint32_t>(tA), tb = static_cast<std::uint32_t>(tB);
      const auto pieceA = sg.mesh.trianglePiece[ta], pieceB = sg.mesh.trianglePiece[tb];
      const Vec2d dA = sg.mesh.rest[nx.a] - sg.mesh.rest[sp.a];
      const Vec2d dB = sg.mesh.rest[nx.b] - sg.mesh.rest[sp.b];
      HingeTerm hg;
      hg.v = {off + opposite(ta, sp.a, nx.a), off + opposite(tb, sp.b, nx.b), off + sp.a, off + nx.a};
      next.hinges.push_back(hg);
      next.hingeSides.push_back({HingeSide{g.fabricOffset + pieceA, dA, sg.mesh.pieceGrain[pieceA]},
                                 HingeSide{g.fabricOffset + pieceB, dB, sg.mesh.pieceGrain[pieceB]}});
      next.hingeGeom.push_back(dA.squaredNorm() / (triArea(ta) + triArea(tb)));
    }
    next.garments.push_back(g);
  }
  const std::size_t n = next.x.size();
  next.v.assign(n, Vec3f::Zero());
  next.fExt.assign(n, Vec3d::Zero());
  next.pinned.assign(n, 0);
  next.mass.assign(n, 0.0);
  for (const auto& pin : scene.pins) {
    if (pin.particle >= n) throw std::out_of_range("pin particle out of range");
    next.pinned[pin.particle] = 1;
    next.pins.push_back({pin.particle, pin.position.cast<float>()});
    next.x[pin.particle] = pin.position.cast<float>();
  }
  for (const auto& f : scene.forces) {
    if (f.particle >= n) throw std::out_of_range("force particle out of range");
    next.fExt[f.particle] += f.force;
  }
  for (std::uint32_t i = 0; i < n; ++i) next.updateMass(i);
  for (std::size_t hgi = 0; hgi < next.hinges.size(); ++hgi) next.hinges[hgi].K = next.hingeStiffness(hgi);
  next.buildStructure();
  return next;
}

void SimModel::updateFabric(const GarmentId& id, const std::vector<FabricPhysical>& pieceFabric) {
  auto& g = garments[garmentIndex(id)];
  if (pieceFabric.size() != g.pieceCount) throw std::invalid_argument("one fabric per pattern piece required");
  std::copy(pieceFabric.begin(), pieceFabric.end(), fabrics.begin() + g.fabricOffset);
  for (std::uint32_t i = g.range.offset; i < g.range.offset + g.range.count; ++i) updateMass(i);
  for (std::size_t hgi = 0; hgi < hinges.size(); ++hgi) {
    const auto fi = hingeSides[hgi][0].fabric;
    if (fi >= g.fabricOffset && fi < g.fabricOffset + g.pieceCount) hinges[hgi].K = hingeStiffness(hgi);
  }
}

void SimModel::setPhase(const GarmentId& id, SimPhase phase) {
  const auto gi = garmentIndex(id);
  auto& g = garments[gi];
  g.phase = phase;
  if (phase == SimPhase::Assembling) {
    g.assemblyTime = 0;
    for (auto& sp : seams) {
      if (sp.garment == gi) sp.startGap = (x[sp.a] - x[sp.b]).norm();
    }
  }
}

SimStats SimModel::stats() const {
  SimStats st;
  double sumV2 = 0;
  std::size_t moving = 0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    if (!x[i].allFinite() || !v[i].allFinite()) st.hasNonFinite = true;
    if (pinned[i]) continue;
    const double v2 = v[i].cast<double>().squaredNorm();
    st.kineticEnergy += 0.5 * mass[i] * v2;
    sumV2 += v2;
    ++moving;
  }
  st.rmsSpeed = moving ? std::sqrt(sumV2 / static_cast<double>(moving)) : 0.0;
  for (const auto& sp : seams)
    st.maxSeamGap = std::max(st.maxSeamGap, static_cast<double>((x[sp.a] - x[sp.b]).norm()));
  if (body) {
    for (const auto& p : x) st.maxBodyPenetration = std::max(st.maxBodyPenetration, -body->sample(p.cast<double>()));
  }
  return st;
}

Snapshot SimModel::snapshot() const {
  Snapshot snap;
  snap.positions = x;
  snap.velocities = v;
  for (const auto& g : garments) {
    snap.phases.push_back(g.phase);
    snap.assemblyTime.push_back(g.assemblyTime);
  }
  for (const auto& sp : seams) snap.seamStartGap.push_back(sp.startGap);
  snap.contactForce = contactForce;
  return snap;
}

void SimModel::restore(const Snapshot& snap) {
  if (snap.positions.size() != x.size() || snap.phases.size() != garments.size()) {
    throw std::invalid_argument("snapshot does not match this scene");
  }
  x = snap.positions;
  v = snap.velocities;
  if (snap.contactForce.size() == contactForce.size()) {
    contactForce = snap.contactForce;
  } else {
    std::fill(contactForce.begin(), contactForce.end(), 0.0);
  }
  for (std::size_t i = 0; i < garments.size(); ++i) {
    garments[i].phase = snap.phases[i];
    garments[i].assemblyTime = snap.assemblyTime[i];
  }
  if (snap.seamStartGap.size() == seams.size()) {
    for (std::size_t k = 0; k < seams.size(); ++k) seams[k].startGap = snap.seamStartGap[k];
  }
}

}  // namespace drape::sim
