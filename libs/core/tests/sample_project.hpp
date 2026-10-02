#pragma once

#include "drape/core/project.hpp"

namespace drape::testing {

inline PatternPiece squarePiece(const Id& id, double size) {
  PatternPiece p;
  p.id = id;
  p.name = id;
  p.outline.add("bottom", CubicBezier::line({0, 0}, {size, 0}));
  p.outline.add("right", CubicBezier::line({size, 0}, {size, size}));
  p.outline.add("top", CubicBezier::line({size, size}, {0, size}));
  p.outline.add("left", CubicBezier::line({0, size}, {0, 0}));
  p.fabricSlot = "body";
  return p;
}

inline Project makeSampleProject() {
  Project p;
  p.meta = {"Sample", "2026-10-01T00:00:00Z", "2026-10-01T01:00:00Z"};
  p.units = DisplayUnits::Inches;
  p.avatar.base = BaseBody::Male;
  p.avatar.targets = {{Measurement::Height, 1.80}, {Measurement::Waist, 0.82}};
  p.avatar.chartRef = ChartRef{"generic_us_men", "M"};
  p.avatar.solvedWeights = {0.1, 0.5};

  SizeChart chart;
  chart.id = "brand";
  chart.name = "Brand chart";
  chart.base = BaseBody::Male;
  chart.sizes = {"S", "M"};
  chart.values = {{Measurement::Bust, {0.96, 1.02}}};
  p.sizeCharts.push_back(chart);

  GarmentInstance g;
  g.id = "g1";
  g.generatorId = "test_tee";
  g.generatorVersion = 1;
  g.slot = OutfitSlot::Top;
  g.size = "L";
  g.specOverrides = {{"L", {{"chest_width", 0.56}}}};
  PatternPiece a = squarePiece("a", 0.1);
  a.layer = LayerRelation{"b", 1};
  a.elastic = ElasticSpec{0.85, {1, 0}};
  a.mirrored = true;
  PatternPiece b = squarePiece("b", 0.1);
  b.placement = Placement{BodyRegion::Neck, -1.5, 0.2, -1, 0.0, 0.08};
  g.pattern.pieces = {a, b};
  g.pattern.seams.push_back(Seam{"s1", {"a", 0.1, 0.2, false}, {"b", 0.3, 0.4, true}, 0.001, SeamKind::ClosedZipper});
  auto line = [](const Id& piece, std::vector<Vec2d> pts, InternalLineKind kind) {
    InternalLine l;
    l.piece = piece;
    l.polyline = std::move(pts);
    l.kind = kind;
    return l;
  };
  InternalLine fold = line("a", {{0, 0.05}, {0.1, 0.05}}, InternalLineKind::Fold);
  fold.foldAngle = 3.14;
  InternalLine quilt = line("a", {{0.05, 0}, {0.05, 0.1}}, InternalLineKind::Quilting);
  InternalLine stitch = line("b", {{0.01, 0}, {0.01, 0.1}}, InternalLineKind::Topstitch);
  stitch.topstitchOffset = 0.006;
  stitch.stitchesPerInch = 8;
  stitch.threadColor = {0.9f, 0.7f, 0.2f};
  stitch.threadThickness = 0.0004;
  g.pattern.internalLines = {fold, quilt, stitch};
  g.pattern.trims.push_back(Trim{TrimKind::Rivet, "a", {0.02, 0.03}, "", 0.008, "copper"});
  g.fabricBySlot = {{"body", "custom_denim"}};
  g.garmentColor = Color{0.1f, 0.1f, 0.1f};
  g.pieceColors = {{"b", Color{1.0f, 0.0f, 0.0f}}};
  g.quality = SimQuality::Fine;
  p.garments.push_back(g);

  Fabric f;
  f.id = "custom_denim";
  f.name = "Custom denim";
  f.physical = {20000, 15000, 4000, 0.02, 0.03, 30e-6, 20e-6, 0.458, 0.0009, 0.4, 0.5};
  f.visual.baseColorTexture = "denim_base.png";
  f.visual.textureScale = 0.02;
  p.customFabrics.push_back(f);

  p.scene.lighting = LightingPreset::StudioContrast;
  p.scene.camera.fovYDegrees = 40;
  return p;
}

}  // namespace drape::testing
