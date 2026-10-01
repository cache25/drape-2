# Drape Phase A — Foundations Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build Drape's portable foundations so the internal test tee drapes headlessly on a static test body. That covers the data model, `.drape` files, pattern validation, the test tee generator, the sim mesh builder, placement, the test body with its collision field, and the CPU reference cloth solver.

**Architecture:** Four CMake libraries (`drape_core`, `drape_garment`, `drape_avatar`, `drape_sim`) plus a developer CLI. All of it is plain C++20 that builds and tests on Linux, with no GPU, Qt or Mac code. The solver is XPBD with small substeps and one constraint iteration per substep. Constraints are solved by graph-colored Gauss–Seidel, and particle state is kept in `float` so the Phase B Metal solver can match it.

**Tech Stack:** C++20, CMake 3.28 + Ninja, GoogleTest v1.17.0, Eigen 3.4 (system package), nlohmann/json v3.12.0, miniz 3.1.0, CDT 1.4.5. Python 3 + matplotlib are used only for the developer OBJ preview.

**Spec:** `docs/superpowers/specs/2026-10-01-drape-milestone-1-design.md`. This plan implements Phase A of Section 17. Read Sections 3, 4, 5, 7.1–7.3, 11.1 and 12.1–12.2 alongside it.

## Global Constraints

- C++20; CMake ≥ 3.28 with presets; Ninja. Our targets compile with `-Wall -Wextra -Wpedantic -Werror` on GCC 13 and Clang 18. Third-party headers are included as SYSTEM, so their warnings never fail the build.
- **Pinned dependencies** via FetchContent (`GIT_SHALLOW TRUE`, `SYSTEM`): nlohmann/json `v3.12.0`, miniz `3.1.0`, GoogleTest `v1.17.0`, CDT `1.4.5` (`SOURCE_SUBDIR CDT`, header-only). Eigen 3.4 comes from `find_package(Eigen3 REQUIRED NO_MODULE)`: apt `libeigen3-dev` on Linux, Homebrew `eigen` on Mac. No other third-party code. Never SMPL; never Shewchuk's Triangle.
- **Module dependencies:**
  - `drape_core` → Eigen, nlohmann_json, miniz
  - `drape_garment` → core, CDT
  - `drape_avatar` → core
  - `drape_sim` → core only (sim *tests* may also link garment and avatar)
- **Namespaces:** `drape` (core types), `drape::garment`, `drape::avatar`, `drape::sim`. Public headers live in `libs/<module>/include/drape/<module>/`.
- **Units:** SI internally (m, kg, s, N, N·m, kg/m²).
- **3D space:** right-handed, Y up, feet at y = 0, body facing +Z.
- **Pattern space:** 2D meters, pieces drawn as seen from outside the garment, outlines counter-clockwise. Edge positions are arc length in meters from the outline's start.
- **Quality settings:** particle distance Draft 0.020 m, Standard 0.010 m, Fine 0.005 m. Substeps per frame 10 / 20 / 30. Frame dt = 1/60 s.
- **Thresholds** (spec 5.7, 5.8, 12.3):
  - Assembling ends when max seam gap < 0.001 m, or after 3 s simulated.
  - Settled when RMS speed < 0.002 m/s for 60 consecutive frames.
  - Body penetration ≤ 0.002 m.
  - Snapshot every 30 frames; at most 3 retries, doubling substeps each time.
  - Failure message, exactly: `Drape couldn't settle this garment. Try Draft quality or a different fabric.`
- Particle positions and velocities are stored and integrated as `float` (`Vec3f`). Rest data may be computed in `double` and stored as `float`.
- `.drape` format version `1`; app version string `"0.1.0-dev"`.
- **ctest labels:** `slow` (Standard-quality and multi-size drapes) and `perf` (timing assertions, run only in the `release` preset). Quick loop: `ctest --preset dev -LE "slow|perf"`; full Debug run: `ctest --preset dev -LE perf`.
- **Not in Phase A** (scheduled for Phase D): air drag (spec 5.6), contact-impulse accumulation for the Pressure fit map (spec 5.5), self-collision and its tunneling guard, which also brings the clamp-share failure criterion of spec 5.8 (spec 5.5), elastic, fold lines, layer order and mouse dragging.
- Every commit message ends with these two lines:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01E6tbT2CPKJE1VpJ175tbiL
  ```

## Review Focus

1. **Thin pieces and very short edges.** A 2 cm rib meshed at Draft, where the piece height equals the particle distance, must still give a valid mesh: every triangle area > 0, no NaN in rest data, and the solver steps cleanly. Tests: Task 7 Step 1 and Task 11 Step 1.
2. **Cloth that starts inside the body** (large body, small offset) must be pushed out within a few frames without a stability failure. Test: Task 14 Step 1.
3. **Points outside the collision field's grid** are treated as far outside (+band) and never read out of bounds. Test: Task 9 Step 1.
4. **A hand-edited or corrupt `project.json`** with null or wrong-typed fields raises `FileError(SchemaMismatch)` with a message naming the JSON path, never a crash. Test: Task 4 Step 1.
5. **Spec-table overrides that make the tee impossible** (zero or negative values, bicep too wide for the armhole) raise `GeneratorError` naming the POM, and nothing is simulated. Test: Task 6 Step 1.

---

## File Structure

```
drape/
  CMakeLists.txt                 root: project, options, add_subdirectory(libs) + tools
  CMakePresets.json              presets: dev (Debug), release (Release); matching test presets
  cmake/DrapeDependencies.cmake  FetchContent + find_package for all dependencies
  cmake/DrapeHelpers.cmake       drape_add_library(), drape_add_tests()
  README.md                      developer setup and commands
  libs/core/      include/drape/core/  math.hpp units.hpp version.hpp bezier.hpp outline.hpp polygon.hpp
                                       body_regions.hpp pattern.hpp fabric.hpp project.hpp serialize.hpp
                                       drape_file.hpp hash.hpp triangle_mesh.hpp collision_field.hpp
                                       sim_mesh.hpp test_fabrics.hpp
                  src/ tests/
  libs/garment/   include/drape/garment/  shapes.hpp validate.hpp spec_table.hpp generator.hpp test_tee.hpp
                                          sim_mesh_builder.hpp placement.hpp
                  src/ tests/
  libs/avatar/    include/drape/avatar/   primitives.hpp test_body.hpp collision_bake.hpp
                  src/ tests/
  libs/sim/       include/drape/sim/      solver.hpp cpu_solver.hpp drape_runner.hpp scene_builder.hpp
                  src/  coloring.* stretch.* bending.* seams.* collision.* cpu_solver.cpp drape_runner.cpp scene_builder.cpp
                  tests/
  tools/drape_dump/  main.cpp obj_writer.hpp obj_writer.cpp
  tools/render_obj.py
```

Each `libs/<module>/CMakeLists.txt` declares its library and tests through the helpers. Test sources include `src/` headers directly when they unit-test internals (`coloring`, `stretch`, `bending`).

---

### Task 1: Repository scaffold, build system and dependencies

**Files:**
- Create: `CMakeLists.txt`, `CMakePresets.json`, `cmake/DrapeDependencies.cmake`, `cmake/DrapeHelpers.cmake`, `.gitignore` (`build/`, `*.obj`, `*.png` under `out/`), `README.md`
- Create: `libs/CMakeLists.txt`, `libs/core/CMakeLists.txt`, `libs/core/include/drape/core/{math.hpp,units.hpp,version.hpp}`, `libs/core/src/version.cpp`
- Test: `libs/core/tests/test_units.cpp`

**Interfaces:**
- Produces:
  - `math.hpp`: `using Vec2d = Eigen::Vector2d; using Vec3d = Eigen::Vector3d; using Vec3f = Eigen::Vector3f; using Mat2d = Eigen::Matrix2d; using Mat3d = Eigen::Matrix3d;` plus `inline constexpr double kPi = 3.14159265358979323846; inline constexpr double kGravity = 9.81;`
  - `units.hpp` (namespace `drape::units`, all `constexpr double f(double)`): `mmToM` (×0.001), `cmToM` (×0.01), `inToM` (×0.0254), `gsmToKgPerM2` (×0.001), `microNewtonMeterToNewtonMeter` (×1e-6), `percentToFraction` (×0.01), `ozPerSqYdToGsm` (×33.9057)
  - `version.hpp`: `const char* appVersion();` returning `"0.1.0-dev"`
  - CMake helpers: `drape_add_library(<name> SOURCES ... [PUBLIC_DEPS ...] [PRIVATE_DEPS ...])` and `drape_add_tests(<name> SOURCES ... DEPS ... [LABELS ...])`. The second creates an executable linked to `GTest::gtest_main` and registers it with `gtest_discover_tests(... PROPERTIES LABELS ...)`.

- [ ] **Step 1: Write the failing test** `libs/core/tests/test_units.cpp`

```cpp
TEST(Units, Conversions) {
  EXPECT_DOUBLE_EQ(drape::units::inToM(1.0), 0.0254);
  EXPECT_DOUBLE_EQ(drape::units::gsmToKgPerM2(160.0), 0.160);
  EXPECT_DOUBLE_EQ(drape::units::microNewtonMeterToNewtonMeter(40.0), 40e-6);
  EXPECT_NEAR(drape::units::ozPerSqYdToGsm(13.5), 457.727, 1e-3);
}
TEST(Version, IsDev) { EXPECT_STREQ(drape::appVersion(), "0.1.0-dev"); }
TEST(Math, EigenAliases) { drape::Vec3d v(1, 2, 2); EXPECT_DOUBLE_EQ(v.norm(), 3.0); }
```

- [ ] **Step 2: Write the build files**

`cmake/DrapeDependencies.cmake`. This exact block was verified to configure and build on this toolchain:

```cmake
include(FetchContent)
set(FETCHCONTENT_QUIET ON)
FetchContent_Declare(nlohmann_json GIT_REPOSITORY https://github.com/nlohmann/json GIT_TAG v3.12.0 GIT_SHALLOW TRUE SYSTEM)
FetchContent_Declare(miniz GIT_REPOSITORY https://github.com/richgel999/miniz GIT_TAG 3.1.0 GIT_SHALLOW TRUE SYSTEM)
FetchContent_Declare(googletest GIT_REPOSITORY https://github.com/google/googletest GIT_TAG v1.17.0 GIT_SHALLOW TRUE SYSTEM)
FetchContent_Declare(cdt GIT_REPOSITORY https://github.com/artem-ogre/CDT GIT_TAG 1.4.5 GIT_SHALLOW TRUE SOURCE_SUBDIR CDT SYSTEM)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_FUZZERS OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nlohmann_json miniz googletest cdt)
find_package(Eigen3 REQUIRED NO_MODULE)
if(Eigen3_VERSION VERSION_LESS 3.4)
  message(FATAL_ERROR "Eigen >= 3.4 required")
endif()
include(GoogleTest)
```

Root `CMakeLists.txt`: `project(drape VERSION 0.1.0 LANGUAGES C CXX)`, `CMAKE_CXX_STANDARD 20` (required, no extensions), `enable_testing()`, include both cmake files, `add_subdirectory(libs)`; `add_subdirectory(tools/drape_dump)` only if that directory exists.

`drape_core` links `PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json` and `PRIVATE miniz`, and sets `target_compile_definitions(drape_core PUBLIC JSON_DIAGNOSTICS=1)` so JSON errors name the failing path.

`CMakePresets.json`:
- Configure presets `dev` (binaryDir `build/dev`, Debug) and `release` (binaryDir `build/release`, Release), both with generator Ninja.
- Build presets and test presets with the same names. Test presets set `output.outputOnFailure: true`.

`README.md` dev setup:

```
sudo apt-get install -y cmake ninja-build g++ libeigen3-dev python3-matplotlib
cmake --preset dev && cmake --build --preset dev && ctest --preset dev
```

- [ ] **Step 3: Configure, build and run tests**

Run: `cmake --preset dev && cmake --build --preset dev && ctest --preset dev`
Expected: configure succeeds; 3 tests PASS.

- [ ] **Step 4: Commit** (`build: scaffold CMake project with pinned dependencies`)

---

### Task 2: Bézier, outline and polygon geometry

**Files:**
- Create: `libs/core/include/drape/core/{bezier.hpp,outline.hpp,polygon.hpp}`, `libs/core/src/{bezier.cpp,outline.cpp,polygon.cpp}`
- Test: `libs/core/tests/{test_bezier.cpp,test_outline.cpp,test_polygon.cpp}`

**Interfaces:**
- Produces (namespace `drape`):
  ```cpp
  struct CubicBezier {
    Vec2d p0, p1, p2, p3;
    Vec2d point(double t) const;
    Vec2d derivative(double t) const;
    CubicBezier reversed() const;            // point(t) == original.point(1-t)
    CubicBezier mirroredX() const;           // (x,y) -> (-x,y) on every control point
    static CubicBezier line(const Vec2d& a, const Vec2d& b);  // controls at 1/3 and 2/3
    bool operator==(const CubicBezier&) const = default;
  };
  class ArcLengthTable {                     // 256 samples, cumulative chord length
   public:
    explicit ArcLengthTable(const CubicBezier& c, int samples = 256);
    double length() const;
    double tAtLength(double s) const;        // binary search + linear interpolation, s clamped to [0, length]
  };
  double length(const CubicBezier& c);
  class Outline {
   public:
    void add(std::string name, const CubicBezier& segment);
    std::size_t size() const;
    const CubicBezier& segment(std::size_t i) const;
    const std::string& name(std::size_t i) const;
    std::optional<std::size_t> find(std::string_view name) const;
    bool isClosed(double tol = 1e-9) const;   // each p3 == next p0 and last p3 == first p0
    double perimeter() const;
    double segmentStart(std::size_t i) const; // arc length at segment start
    double segmentEnd(std::size_t i) const;
    Vec2d pointAt(double s) const;            // s clamped to [0, perimeter]; s == perimeter -> start point
    Vec2d tangentAt(double s) const;          // unit
    std::vector<Vec2d> polyline(double maxSpacing) const;  // closed (last != first), contains every segment endpoint
    bool operator==(const Outline& o) const;  // compares segments and names
  };
  double signedArea(std::span<const Vec2d> closedPolyline);
  bool pointInPolygon(const Vec2d& p, std::span<const Vec2d> closedPolyline);
  double distancePointSegment(const Vec2d& p, const Vec2d& a, const Vec2d& b);
  bool segmentsIntersect(const Vec2d& a, const Vec2d& b, const Vec2d& c, const Vec2d& d);  // includes touching
  bool isSimplePolygon(std::span<const Vec2d> closedPolyline);  // no intersections between non-adjacent edges
  ```

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(Bezier, LineLengthIsExact) { EXPECT_NEAR(drape::length(drape::CubicBezier::line({0,0},{3,4})), 5.0, 1e-12); }
TEST(Bezier, QuarterCircleLength) {
  const double k = 0.5522847498;
  drape::CubicBezier q{{1,0},{1,k},{k,1},{0,1}};
  EXPECT_NEAR(drape::length(q), drape::kPi / 2, 1e-3);
}
TEST(Bezier, TAtLengthInverts) {  // for s in {0, .25L, .5L, L}: ArcLengthTable(sub-curve [0, tAtLength(s)]) length ≈ s (1e-6)
}
TEST(Bezier, ReversedAndMirrored) {  // reversed().point(0.3) == point(0.7); mirroredX().point(0.3) == (-x, y)
}
TEST(Outline, SquarePerimeterAndPoints) {
  // unit square CCW from (0,0), segments "bottom","right","top","left"
  EXPECT_DOUBLE_EQ(sq.perimeter(), 4.0);
  EXPECT_TRUE(sq.pointAt(1.5).isApprox(drape::Vec2d(1, 0.5)));
  EXPECT_TRUE(sq.tangentAt(1.5).isApprox(drape::Vec2d(0, 1)));
  EXPECT_TRUE(sq.pointAt(4.0).isApprox(drape::Vec2d(0, 0)));
  EXPECT_DOUBLE_EQ(sq.segmentStart(2), 2.0);
  EXPECT_EQ(sq.find("top"), 2u);
  EXPECT_TRUE(sq.isClosed());
}
TEST(Outline, OpenOutlineDetected) {  // only 3 sides -> isClosed() false
}
TEST(Outline, PolylineSpacing) {  // polyline(0.3): contains 4 corners, every edge <= 0.3 + 1e-12, first != last
}
TEST(Polygon, SignedAreaOrientation) {  // CCW unit square +1, CW -1
}
TEST(Polygon, PointInPolygon) {  // (0.5,0.5) inside; (1.5,0.5) outside
}
TEST(Polygon, DistancePointSegment) {  // (0,1) to (-1,0)-(1,0) == 1; (2,0) to same segment == 1
}
TEST(Polygon, BowtieIsNotSimple) {  // {(0,0),(1,1),(1,0),(0,1)} false; unit square true
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R "Bezier|Outline|Polygon"`
Expected: build fails (headers missing).

- [ ] **Step 3: Implement** the three headers and sources.

`Outline` keeps one `ArcLengthTable` per segment and the cumulative starts, all updated in `add`. `polyline` emits, for each segment, `n = max(1, ceil(len/maxSpacing))` points at equal arc length, leaving out the segment's end point (the next segment starts there).

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R "Bezier|Outline|Polygon"`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(core): Bezier, outline and polygon geometry`)

---

### Task 3: Data model and JSON serialization

**Files:**
- Create: `libs/core/include/drape/core/{body_regions.hpp,pattern.hpp,fabric.hpp,project.hpp,serialize.hpp,test_fabrics.hpp}`, `libs/core/src/{pattern.cpp,serialize.cpp,test_fabrics.cpp}`
- Test: `libs/core/tests/{sample_project.hpp,test_serialize.cpp,test_pattern.cpp}`

**Interfaces:**
- Consumes: `Vec2d/Vec3d` (Task 1), `CubicBezier`, `Outline` (Task 2).
- Produces (namespace `drape`). Every struct declares `bool operator==(const T&) const = default;`.
  ```cpp
  using Id = std::string;
  // body_regions.hpp
  enum class BodyRegion { Torso, Neck, Head, ArmLeft, ArmRight, LegLeft, LegRight, Waist };
  struct RegionCylinder { Vec3d origin; Vec3d axis; Vec3d e0; double radius = 0; Vec3d e1() const { return axis.cross(e0); } };
  struct BodyRegions { std::map<BodyRegion, RegionCylinder> cylinders; };
  // Region origins are anchors: Torso and Neck at neck-base height on the body axis, arms at the shoulder joints.
  // pattern.hpp
  struct Placement { BodyRegion region = BodyRegion::Torso; double centerAngle = 0; double axialAtOrigin = 0;
                     int axialSign = 1; double offset = 0.06; std::optional<double> wrapRadius; };
  struct LayerRelation { Id basePiece; int order = 1; };
  struct ElasticSpec { double factor = 1; Vec2d direction{1, 0}; };
  struct PatternPiece { Id id; std::string name; Outline outline; Vec2d grainline{0, 1}; std::string fabricSlot;
                        Placement placement; std::optional<LayerRelation> layer; std::optional<ElasticSpec> elastic; bool mirrored = false; };
  struct EdgeRef { Id piece; double start = 0; double end = 0; bool reversed = false; };  // arc length, start < end
  enum class SeamKind { Normal, ClosedZipper, ClosedFly };
  struct Seam { Id id; EdgeRef a, b; double ease = 0; SeamKind kind = SeamKind::Normal; };
  enum class InternalLineKind { Fold, Quilting, Topstitch };
  struct Color { float r = 1, g = 1, b = 1; };   // sRGB, 0..1
  struct InternalLine { Id piece; std::vector<Vec2d> polyline; InternalLineKind kind = InternalLineKind::Fold;
                        double foldAngle = 0; double stiffnessMultiplier = 1; double topstitchOffset = 0;
                        double stitchesPerInch = 0; Color threadColor; double threadThickness = 0; };
  enum class TrimKind { Rivet, Button, ZipperStrip };
  struct Trim { TrimKind kind = TrimKind::Rivet; Id piece; Vec2d position{0, 0}; Id seam; double size = 0; std::string material; };
  struct Pattern { std::vector<PatternPiece> pieces; std::vector<Seam> seams; std::vector<InternalLine> internalLines; std::vector<Trim> trims;
                   const PatternPiece* findPiece(std::string_view id) const; };
  // Edge covering the contiguous named segments [first..last] of `piece`. Throws std::invalid_argument for unknown names or last before first.
  EdgeRef segmentEdge(const PatternPiece& piece, std::string_view first, std::string_view last, bool reversed);
  // fabric.hpp (SI units, see spec 6.1)
  struct FabricPhysical { double stretchWarp = 0, stretchWeft = 0, stretchBias = 0;   // N/m
                          double strainLimitWarp = 0, strainLimitWeft = 0;            // fraction (0.05 = 5%)
                          double bendWarp = 0, bendWeft = 0;                          // N·m
                          double weight = 0;                                          // kg/m²
                          double thickness = 0;                                       // m
                          double friction = 0, damping = 0; };                        // 0..1
  struct FabricVisual { std::string baseColorTexture, normalTexture, roughnessTexture; double textureScale = 0.01;
                        Color sheenColor; double sheenRoughness = 0.5; Color subsurfaceColor; double insideDarkening = 0.2; };
  struct Fabric { Id id; std::string name; FabricPhysical physical; FabricVisual visual; };
  // project.hpp
  enum class BaseBody { Female, Male };
  enum class Measurement { Height, Bust, Underbust, Waist, Hip, ShoulderWidth, ArmLength, Inseam, Neck, HeadCircumference };
  struct ChartRef { Id chart; std::string size; };
  struct AvatarParams { BaseBody base = BaseBody::Female; std::map<Measurement, double> targets; std::optional<ChartRef> chartRef; std::vector<double> solvedWeights; };
  struct SizeChart { Id id; std::string name; BaseBody base = BaseBody::Female; std::vector<std::string> sizes; std::map<Measurement, std::vector<double>> values; };
  enum class OutfitSlot { Top, Bottom, Headwear };
  enum class SimQuality { Draft, Standard, Fine };
  struct GarmentInstance { Id id; std::string generatorId; int generatorVersion = 1; OutfitSlot slot = OutfitSlot::Top; std::string size = "M";
                           std::map<std::string, std::map<std::string, double>> specOverrides;   // size -> POM key -> value (m)
                           Pattern pattern; std::map<std::string, Id> fabricBySlot; std::optional<Color> garmentColor;
                           std::map<Id, Color> pieceColors; SimQuality quality = SimQuality::Standard; };
  enum class LightingPreset { StudioSoft, StudioContrast, OvercastOutdoor };
  struct Camera { Vec3d position{0, 1.2, 3}; Vec3d target{0, 1.1, 0}; double fovYDegrees = 35; };
  struct Scene { LightingPreset lighting = LightingPreset::StudioSoft; Camera camera; };
  enum class DisplayUnits { Centimeters, Inches };
  struct ProjectMetadata { std::string name; std::string created; std::string modified; };   // ISO-8601 UTC strings
  struct Project { ProjectMetadata meta; DisplayUnits units = DisplayUnits::Centimeters; AvatarParams avatar;
                   std::vector<SizeChart> sizeCharts; std::vector<GarmentInstance> garments; std::vector<Fabric> customFabrics; Scene scene; };
  // serialize.hpp: void to_json(nlohmann::json&, const T&) and void from_json(const nlohmann::json&, T&) for every type above.
  // test_fabrics.hpp (namespace drape::testdata): FabricPhysical testJersey(), testRib(), testStiff(), testAniso();
  ```

**JSON rules:**
- Keys are the field names in camelCase.
- Enums are lower-camel strings via `NLOHMANN_JSON_SERIALIZE_ENUM` (`"female"`, `"armLeft"`, `"closedZipper"`, `"studioSoft"`).
- `Vec2d`/`Vec3d`/`Color` are arrays.
- An empty `std::optional` omits its key.
- Maps keyed by an enum (`targets`, `values`, `cylinders`) become JSON objects keyed by the enum string.
- `Outline` is `[{"name":..., "p":[[x,y]×4]}, ...]`.
- `from_json` uses `.at()`, so a missing key throws.

**Test fabric values (SI):**

| Fabric | stretch warp / weft / bias | strain limit warp / weft | bend warp / weft | weight | thickness | friction | damping |
|---|---|---|---|---|---|---|---|
| testJersey | 400 / 200 / 100 | 0.5 / 0.8 | 2e-6 / 1.5e-6 | 0.160 | 0.0006 | 0.3 | 0.3 |
| testRib | 300 / 100 / 80 | 0.8 / 1.0 | 3e-6 / 2e-6 | 0.280 | 0.0010 | 0.3 | 0.3 |
| testStiff | 20000 / 20000 / 5000 | 0.02 / 0.02 | 100e-6 / 100e-6 | 0.450 | 0.0008 | 0.3 | 1.0 |
| testAniso | as testStiff | as testStiff | 100e-6 / 40e-6 | 0.450 | 0.0008 | 0.3 | 1.0 |

- [ ] **Step 1: Write the failing tests**

```cpp
// sample_project.hpp: drape::Project makeSampleProject();
//   two rectangle pieces (one with layer + elastic set, one placement with wrapRadius set), one seam,
//   one internal line of each kind, one trim, one custom fabric, one size chart, avatar with chartRef and two
//   targets, garment with garmentColor and one pieceColor.
TEST(Serialize, ProjectRoundTrip) {
  auto p = makeSampleProject();
  nlohmann::json j = p;
  EXPECT_EQ(nlohmann::json::parse(j.dump()).get<drape::Project>(), p);
}
TEST(Serialize, OptionalFieldsOmitted) {  // piece without layer -> json object has no "layer" key
}
TEST(Serialize, EnumsAsStrings) {  // Female -> "female", ArmLeft -> "armLeft", ClosedZipper -> "closedZipper"
}
TEST(Serialize, MissingKeyThrows) {  // erase "seams" from a pattern's json -> get<Pattern>() throws nlohmann::json::exception
}
TEST(Pattern, SegmentEdgeSpansNamedSegments) {
  // unit square piece: segmentEdge(sq, "right", "top", false) -> {start 1.0, end 3.0, reversed false}; unknown name throws
}
TEST(Regions, E1IsAxisCrossE0) {  // axis +Y, e0 +Z -> e1 == +X
}
TEST(TestFabrics, AreSI) { EXPECT_DOUBLE_EQ(drape::testdata::testJersey().weight, 0.160);
                           EXPECT_DOUBLE_EQ(drape::testdata::testAniso().bendWeft, 40e-6); }
```

The square piece fixture here is built directly from `CubicBezier::line`; Task 5 adds a shared `rectanglePiece` helper.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R "Serialize|Pattern|Regions|TestFabrics"`
Expected: build fails.

- [ ] **Step 3: Implement** the headers, `segmentEdge`, the serializers and the test fabrics.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R "Serialize|Pattern|Regions|TestFabrics"`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(core): data model and JSON serialization`)

---

### Task 4: `.drape` project files

**Files:**
- Create: `libs/core/include/drape/core/{drape_file.hpp,hash.hpp}`, `libs/core/src/{drape_file.cpp,hash.cpp}`
- Test: `libs/core/tests/test_drape_file.cpp`

**Interfaces:**
- Consumes: `Project` + serializers (Task 3), `appVersion()` (Task 1).
- Produces (namespace `drape`):
  ```cpp
  inline constexpr int kDrapeFormatVersion = 1;
  enum class FileErrorCode { Io, NotAZip, MissingEntry, InvalidJson, SchemaMismatch, NewerFormat };
  class FileError : public std::runtime_error { public: FileError(FileErrorCode code, const std::string& message); FileErrorCode code() const; };
  struct CacheEntry { std::uint64_t key = 0; std::vector<Vec3f> positions; bool operator==(const CacheEntry&) const = default; };
  struct DrapeFile { Project project; std::map<Id, CacheEntry> cache; std::vector<std::uint8_t> thumbnailPng; bool operator==(const DrapeFile&) const = default; };
  void saveDrapeFile(const std::filesystem::path& path, const DrapeFile& file);   // throws FileError(Io)
  DrapeFile loadDrapeFile(const std::filesystem::path& path);                     // throws FileError
  nlohmann::json migrateProjectJson(nlohmann::json j, int fromVersion);           // v1: identity; >1 throws NewerFormat
  std::uint64_t fnv1a64(std::string_view bytes);                                  // hash.hpp; offset 0xcbf29ce484222325, prime 0x100000001b3
  std::uint64_t garmentCacheKey(const Project& project, const GarmentInstance& garment);
  ```

**Container layout (ZIP via miniz), per spec 11.1:**

| Entry | Contents |
|---|---|
| `manifest.json` | `{"formatVersion":1,"appVersion":"0.1.0-dev","created":meta.created,"modified":meta.modified}` |
| `project.json` | The project as JSON |
| `cache/<garment-id>.bin` | One entry per cache entry; binary layout below |
| `thumbnail.png` | Only when `thumbnailPng` is non-empty |

**Cache binary layout (little-endian):** magic `"DRPC"`, `u32` version = 1, `u64` key, `u32` count, then count × 3 `float32`. An entry with a bad magic, version or length is skipped silently; that garment simply re-drapes.

**`garmentCacheKey`:** `fnv1a64` of a canonical JSON dump of `{"pattern": garment.pattern, "fabricBySlot": garment.fabricBySlot, "fabrics": [the project.customFabrics whose ids appear in fabricBySlot, in id order], "quality": garment.quality, "avatar": project.avatar}`.

**Atomic save:** write to `<path>.tmp` in the same directory, then `std::filesystem::rename` over the destination.

**Error messages:**

| Code | Message |
|---|---|
| NotAZip | `"<path> is not a Drape project file"` |
| MissingEntry | `"<path> is missing <entry>"` |
| InvalidJson | `"<entry> in <path> is not valid JSON: <detail>"` |
| SchemaMismatch | `"<entry> in <path> has unexpected content: <nlohmann what()>"` (includes the JSON path because of `JSON_DIAGNOSTICS`) |
| NewerFormat | `"<path> was saved by a newer version of Drape (format <n>); this version reads format 1"` |
| Io | `"could not write <path>: <detail>"` |

- [ ] **Step 1: Write the failing tests** (use a `std::filesystem::temp_directory_path()` subfolder per test)

```cpp
TEST(DrapeFile, RoundTrip) {  // project + cache {"g1": {key 42, 3 positions}} + 4 thumbnail bytes -> load == saved
}
TEST(DrapeFile, ManifestContents) {  // read manifest.json with miniz: formatVersion == 1, appVersion == "0.1.0-dev"
}
TEST(DrapeFile, NewerFormatRefused) {  // manifest formatVersion 2 -> FileError NewerFormat, what() contains "newer version of Drape"
}
TEST(DrapeFile, NotAZip) {  // plain text file -> NotAZip
}
TEST(DrapeFile, MissingProjectJson) {  // zip with only manifest -> MissingEntry, what() contains "project.json"
}
TEST(DrapeFile, InvalidJson) {  // project.json = "{" -> InvalidJson
}
TEST(DrapeFile, NullFieldIsSchemaMismatch) {  // Review Focus 4: valid project with "units": null
  // -> SchemaMismatch, what() contains "project.json" and "/units"
}
TEST(DrapeFile, TruncatedFileIsNotAZip) {  // save valid file, truncate to half its size -> NotAZip
}
TEST(DrapeFile, CorruptCacheEntryIgnored) {  // cache/g1.bin with magic "XXXX" -> load succeeds, cache.count("g1") == 0
}
TEST(DrapeFile, CacheKeyTracksInputs) {  // key stable for equal inputs; changes when pattern, quality or avatar target changes
}
TEST(DrapeFile, SaveIsAtomic) {  // save twice to the same path -> second content loads; no "*.tmp" left in the folder
}
TEST(DrapeFile, MigrationIdentityAndNewer) {  // migrateProjectJson(j,1) == j; migrateProjectJson(j,2) throws NewerFormat
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R DrapeFile`
Expected: build fails.

- [ ] **Step 3: Implement** `drape_file.cpp` and `hash.cpp`. Load order:
  1. Open the ZIP (failure → NotAZip).
  2. Read the manifest (missing → MissingEntry; bad JSON → InvalidJson).
  3. Check `formatVersion` (> 1 → NewerFormat).
  4. Read `project.json`, migrate it, then `get<Project>()`. Wrap `nlohmann::json::exception` as SchemaMismatch.
  5. Read the cache entries and the thumbnail.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R DrapeFile`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(core): .drape container with manifest, cache and errors`)

---

### Task 5: Pattern validation and shape helpers

**Files:**
- Create: `libs/garment/CMakeLists.txt`, `libs/garment/include/drape/garment/{shapes.hpp,validate.hpp}`, `libs/garment/src/{shapes.cpp,validate.cpp}`
- Modify: `libs/CMakeLists.txt` (add `garment`)
- Test: `libs/garment/tests/test_validate.cpp`

**Interfaces:**
- Consumes: `Pattern`, `PatternPiece`, `Seam`, `EdgeRef`, `segmentEdge` (Task 3); `Outline::polyline`, `signedArea`, `isSimplePolygon` (Task 2).
- Produces (namespace `drape::garment`):
  ```cpp
  // CCW from (0,0); segments "bottom","right","top","left"; default Placement.
  PatternPiece rectanglePiece(const Id& id, double width, double height,
                              const Vec2d& grainline = {0, 1}, const std::string& fabricSlot = "body");
  struct ValidationIssue { std::string rule; std::string message; };
  std::vector<ValidationIssue> validatePattern(const Pattern& pattern);   // empty == valid
  ```

**Rules** (spec 7.2). Every message names the piece id and/or seam id.

| Rule id | Fails when |
|---|---|
| `outline-closed` | `!outline.isClosed()` |
| `outline-simple` | `!isSimplePolygon(outline.polyline(0.002))` |
| `outline-ccw` | `signedArea(polyline) <= 0` |
| `grainline` | `grainline.norm() < 1e-9` |
| `fabric-slot` | `fabricSlot.empty()` |
| `seam-piece` | an edge names a piece that doesn't exist |
| `seam-range` | not `0 <= start < end <= perimeter + 1e-9` |
| `seam-overlap` | two seam edges on the same piece overlap by more than 1e-9 |
| `seam-twisted` | `a.reversed == b.reversed` (pieces facing outward always join edges with opposite traversal) |
| `seam-length` | `abs(lenA - lenB) > 0.002 + abs(ease)` |

Elastic ratios join this rule in Phase D.

- [ ] **Step 1: Write the failing tests** (fixture `twoSquaresSewn()`: two `rectanglePiece`s of 0.1 × 0.1; seam `s1` joining `sq1` "right" (not reversed) to `sq2` "left" (reversed))

```cpp
TEST(Validate, TwoSquaresSewnIsValid) { EXPECT_TRUE(validatePattern(twoSquaresSewn()).empty()); }
// one test per rule: mutate the fixture and expect exactly that rule id among the issues
TEST(Validate, OpenOutline) {}        // drop the "left" segment -> "outline-closed"
TEST(Validate, SelfIntersecting) {}   // bowtie outline -> "outline-simple"
TEST(Validate, Clockwise) {}          // reversed segment order -> "outline-ccw"
TEST(Validate, ZeroGrainline) {}      // -> "grainline"
TEST(Validate, EmptyFabricSlot) {}    // -> "fabric-slot"
TEST(Validate, UnknownPiece) {}       // b.piece = "nope" -> "seam-piece", message contains "s1"
TEST(Validate, RangeOutside) {}       // a.end = 5.0 -> "seam-range"
TEST(Validate, Overlap) {}            // second seam on sq1 covering "right" again -> "seam-overlap"
TEST(Validate, Twisted) {}            // b.reversed = false -> "seam-twisted"
TEST(Validate, LengthMismatch) {}     // sq2 height 0.103 -> "seam-length"; with ease 0.003 -> no issue
TEST(Validate, EdgeEndingAtPerimeterIsValid) {}  // edge over "left" (end == perimeter exactly) -> no "seam-range"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R Validate`
Expected: build fails.

- [ ] **Step 3: Implement** `shapes.cpp` and `validate.cpp`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R Validate`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(garment): pattern validation rules`)

---

### Task 6: Spec tables, generator interface and the test tee

**Files:**
- Create: `libs/garment/include/drape/garment/{spec_table.hpp,generator.hpp,test_tee.hpp}`, `libs/garment/src/{spec_table.cpp,test_tee.cpp}`
- Test: `libs/garment/tests/{test_spec_table.cpp,test_test_tee.cpp}`

**Interfaces:**
- Consumes: `Pattern` and related types, `segmentEdge` (Task 3); `CubicBezier`, `Outline`, `length` (Task 2); `validatePattern` (Task 5).
- Produces (namespace `drape::garment`):
  ```cpp
  inline const std::vector<std::string> kStandardSizes{"XS", "S", "M", "L", "XL", "XXL"};
  struct PomSpec { std::string key; std::string label; double base = 0; double gradePerSize = 0; };
  struct SpecTable { std::string baseSize = "M"; std::vector<PomSpec> poms; };
  using SpecOverrides = std::map<std::string, std::map<std::string, double>>;   // size -> POM key -> value (m)
  // value = base + gradePerSize * (index(size) - index(baseSize)) unless overridden. Unknown size -> std::invalid_argument.
  std::map<std::string, double> resolveSpec(const SpecTable& table, const std::string& size, const SpecOverrides& overrides = {});
  class GeneratorError : public std::runtime_error { using std::runtime_error::runtime_error; };
  class GarmentGenerator {
   public:
    virtual ~GarmentGenerator() = default;
    virtual std::string id() const = 0;
    virtual int version() const = 0;
    virtual OutfitSlot slot() const = 0;
    virtual SpecTable specTable() const = 0;
    virtual Pattern generate(const std::map<std::string, double>& spec) const = 0;   // throws GeneratorError
  };
  class TestTeeGenerator final : public GarmentGenerator { /* id "test_tee", version 1, slot Top */ };
  ```

**Test tee spec table (meters).** The label is the key with spaces.

| key | M | grade/size | | key | M | grade/size |
|---|---|---|---|---|---|---|
| chest_width | 0.530 | 0.025 | | neck_width | 0.180 | 0.005 |
| body_length | 0.720 | 0.020 | | front_neck_drop | 0.090 | 0.003 |
| shoulder_width | 0.460 | 0.020 | | back_neck_drop | 0.020 | 0 |
| sleeve_length | 0.210 | 0.010 | | armhole_depth | 0.250 | 0.010 |
| sleeve_opening | 0.170 | 0.008 | | shoulder_drop | 0.045 | 0 |
| bicep_width | 0.190 | 0.008 | | neck_rib_height | 0.020 | 0 |

**Drafting.** Here C = chest_width, L = body_length, S = shoulder_width, AD = armhole_depth, SD = shoulder_drop, NW = neck_width, ND = front_neck_drop (front) or back_neck_drop (back), SL = sleeve_length, W = bicep_width, SO = sleeve_opening, H = neck_rib_height. Pattern y = 0 at the hem; x = 0 on the center line.

- **Front and back** (identical except ND). Right-half points:
  - HPS = (NW/2, L)
  - SP = (S/2, L−SD)
  - AB = (C/2, L−AD)
  - CF/CB = (0, L−ND)
  - Armhole Bézier SP→AB: P1 = (S/2, L−SD−0.5·(AD−SD)), P2 = (S/2 + 0.5·(C/2−S/2), L−AD).
  - Neck Bézier CF→HPS: P1 = (0.55·NW/2, L−ND), P2 = (NW/2, L−0.45·ND).
  - Outline, CCW from (−C/2, 0), with segment names: `hem` (→(C/2,0)), `side_r` (→AB), `armhole_r` (AB→SP, the reversed armhole Bézier), `shoulder_r` (→HPS), `neck_r` (HPS→CF, reversed neck), `neck_l` (CF→mirrored HPS, the mirrored neck), `shoulder_l`, `armhole_l`, `side_l`. The `_l` segments are `mirroredX()` copies traversed so that the outline stays CCW.
- **Sleeve.** Flat measurements are half the piece width, so the bicep half-width is W and the hem half-width is SO. U = (W, SL−CH), T = (0, SL).
  - Cap Bézier U→T: P1 = (0.75·W, SL−0.45·CH), P2 = (0.40·W, SL).
  - Outline CCW from (−SO, 0): `hem` (→(SO,0)), `underarm_r` (→U), `cap_r` (U→T), `cap_l` (T→mirrored U), `underarm_l` (→(−SO,0)).
  - Cap height CH is found by bisection on [0.02, SL−0.02] until `length(cap_r) == length(front armhole_r)` within 1e-4. If no root is bracketed: `GeneratorError("test_tee: bicep_width is too large for the armhole; no sleeve cap fits")`.
- **Neck rib.** A rectangle CCW from (0,0) with segments `bottom`, `right`, `top`, `left`. Length Lr = front neckline length (`neck_r`+`neck_l`) + back neckline length; height H.
- **Pieces:**

  | Piece | Fabric slot | Grainline |
  |---|---|---|
  | `front` | body | (0,1) |
  | `back` | body | (0,1) |
  | `sleeve_l` | body | (0,1) |
  | `sleeve_r` | body | (0,1) |
  | `neck_rib` | rib | (0,1) |

  "_l"/"_r" on sleeves means the wearer's left/right. Pattern +x is the wearer's left on the front and the wearer's right on the back.
- **Placement** (`region, centerAngle, axialAtOrigin, axialSign, offset, wrapRadius`):

  | Piece | Placement |
  |---|---|
  | front | `Torso, 0, −L, +1, 0.06` |
  | back | `Torso, π, −L, +1, 0.06` |
  | sleeve_l | `ArmLeft, 0, SL, −1, 0.06` |
  | sleeve_r | `ArmRight, 0, SL, −1, 0.06` |
  | neck_rib | `Neck, −π/2, 0, +1, 0, wrapRadius = Lr/(2π)` |

- **Seams.** Edge A is not reversed; edge B is reversed. Edges come from `segmentEdge`, and the rib's bottom edge is split by arc length.

  | Seam id | A | B |
  |---|---|---|
  | shoulder_wl | front.shoulder_r | back.shoulder_l |
  | shoulder_wr | front.shoulder_l | back.shoulder_r |
  | side_wl | front.side_r | back.side_l |
  | side_wr | front.side_l | back.side_r |
  | armhole_front_wl | front.armhole_r | sleeve_l.cap_l |
  | armhole_back_wl | back.armhole_l | sleeve_l.cap_r |
  | armhole_front_wr | front.armhole_l | sleeve_r.cap_r |
  | armhole_back_wr | back.armhole_r | sleeve_r.cap_l |
  | underarm_l | sleeve_l.underarm_r | sleeve_l.underarm_l |
  | underarm_r | sleeve_r.underarm_r | sleeve_r.underarm_l |
  | neck_front | neck_rib.bottom [0, Lf] | front.neck_r..neck_l |
  | neck_back | neck_rib.bottom [Lf, Lr] | back.neck_r..neck_l |
  | rib_close | neck_rib.right | neck_rib.left |

- **Spec checks before drafting.** Every POM must be > 0, and AD > SD + 0.02. Otherwise throw `GeneratorError("test_tee: '<key>' must be > 0")` or `GeneratorError("test_tee: 'armhole_depth' must exceed 'shoulder_drop' by at least 0.02 m")`.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(SpecTable, GradesFromBase) {
  auto t = TestTeeGenerator{}.specTable();
  EXPECT_NEAR(resolveSpec(t, "XS").at("chest_width"), 0.480, 1e-12);
  EXPECT_NEAR(resolveSpec(t, "XXL").at("chest_width"), 0.605, 1e-12);
}
TEST(SpecTable, OverrideWins) {  // {"M":{"chest_width":0.55}} -> M 0.55, L still 0.555
}
TEST(SpecTable, UnknownSizeThrows) {  // "XXXL" -> std::invalid_argument
}
TEST(TestTee, ValidForAllSizes) {  // for each kStandardSizes: validatePattern(generate(resolveSpec(t, s))).empty()
}
TEST(TestTee, PomsReproduceSpec) {
  // for each size, measured from outline points, each within 0.001 of the spec:
  // chest = front width at y = L-AD; body length = front HPS.y - hem y; shoulder = distance between back SP points;
  // sleeve length = cap top y - hem y; sleeve opening = sleeve hem width / 2; bicep = sleeve width at U / 2;
  // neck width = distance between front HPS points
}
TEST(TestTee, HasThirteenNamedSeams) {  // ids == the 13 ids in the seam table
}
TEST(TestTee, CapMatchesArmhole) {  // |len(sleeve_l cap_l) - len(front armhole_r)| < 1e-4 for every size
}
TEST(TestTee, PiecesSlotsAndGrain) {  // ids, fabric slots and grainlines as in the piece table
}
TEST(TestTee, InfeasibleSpecThrows) {  // Review Focus 5
  // M with bicep_width 0.30 -> GeneratorError, what() contains "bicep_width";
  // chest_width 0 -> contains "chest_width"; sleeve_length -0.1 -> contains "sleeve_length"
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R "SpecTable|TestTee"`
Expected: build fails.

- [ ] **Step 3: Implement** `spec_table.cpp` and `test_tee.cpp` following the drafting above.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R "SpecTable|TestTee"`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(garment): spec tables and test tee generator`)

---

### Task 7: Sim mesh builder

**Files:**
- Create: `libs/core/include/drape/core/sim_mesh.hpp`, `libs/garment/include/drape/garment/sim_mesh_builder.hpp`, `libs/garment/src/sim_mesh_builder.cpp`
- Test: `libs/garment/tests/test_sim_mesh.cpp`

**Interfaces:**
- Consumes: `Pattern`, `SimQuality` (Task 3); `Outline`, polygon helpers (Task 2); `validatePattern`, `rectanglePiece` (Task 5); `TestTeeGenerator` (Task 6); CDT.
- Produces:
  ```cpp
  namespace drape {   // core/sim_mesh.hpp
  struct SeamPair { std::uint32_t a = 0, b = 0; std::uint32_t seam = 0; std::uint32_t index = 0; };   // index: 0..N along the seam
  struct SimMesh {
    std::vector<Vec2d> rest;                                 // pattern-space position (m)
    std::vector<std::uint32_t> vertexPiece;                  // index into Pattern::pieces
    std::vector<std::array<std::uint32_t, 3>> triangles;     // CCW in pattern space, global vertex indices
    std::vector<std::uint32_t> trianglePiece;
    std::vector<double> vertexArea;                          // 1/3 of incident triangle rest areas (m²)
    std::vector<std::uint8_t> boundary;                      // 1 if the vertex lies on its piece outline
    std::vector<Vec2d> pieceGrain;                           // unit grainline per piece index
    std::vector<SeamPair> seamPairs;                         // grouped by seam (Pattern order), ascending index
  };
  }
  namespace drape::garment {
  double particleDistance(SimQuality q);   // Draft 0.020, Standard 0.010, Fine 0.005
  class MeshError : public std::runtime_error { using std::runtime_error::runtime_error; };
  SimMesh buildSimMesh(const Pattern& pattern, double particleDistance);   // throws MeshError listing validation issue rule ids
  }
  ```

**Algorithm** (spec 5.3). Pieces are processed in Pattern order and their vertices concatenated. With h = particle distance:
1. **Breakpoints** (arc length on the piece outline): 0, every corner (a segment junction whose unit tangents differ by > 10°), and every seam-edge start and end on this piece. Sort and dedupe at 1e-9.
2. **Seam edges:** sample each edge as a whole with N = max(1, ⌈max(lenA, lenB)/h⌉) equal arc-length steps. Side A runs from start to end. Side B runs in its traversal direction (end to start when reversed), so A's i-th point pairs with B's i-th point. Corners strictly inside a seam edge are not preserved.
3. **Free spans** between breakpoints: n = max(1, round(len/h)) equal steps.
4. **Interior lattice.** Use the grain frame u = grainline (normalized) and v = (−u.y, u.x), with the origin at pattern (0,0). Lattice points are (i + 0.5·(j mod 2))·h·u + j·(√3/2)·h·v. Keep points that are inside the boundary polygon and at least 0.5·h from every boundary edge.
5. **Triangulate** with CDT: run `CDT::RemoveDuplicatesAndRemapEdges`, insert vertices, constrain the boundary edges, then `eraseOuterTriangles()`.
6. **Smooth:** 3 iterations of Laplacian smoothing (move each vertex to the mean of its 1-ring), non-boundary vertices only.
7. **Orient** every triangle CCW in pattern space. Throw `MeshError` if any triangle area < 1e-12.
8. **Fill** `vertexArea`, `boundary` and `pieceGrain`, and append `seamPairs` (seam index in Pattern order, i = 0..N).

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(SimMesh, ParticleDistances) {  // 0.020 / 0.010 / 0.005
}
TEST(SimMesh, SeamPairsAreOneToOne) {
  // two 0.2x0.1 rectangles, sq1.right <-> sq2.left (B reversed), h = 0.01
  // -> 11 seam pairs, index 0..10; rest.y of a_i == rest.y of b_i within 1e-9
}
TEST(SimMesh, AreaIsPreserved) {  // 0.2x0.1 rectangle: sum of triangle areas == 0.02 within 1e-9
}
TEST(SimMesh, VertexAreaSumsToTotal) {  // sum(vertexArea) == sum(triangle areas) within 1e-12
}
TEST(SimMesh, TrianglesAreWellShaped) {
  // every triangle's minimum angle >= 15 deg for: 0.2x0.1 rectangle at 0.01; 4-segment Bézier circle r=0.1 at 0.01;
  // test tee size M at 0.020, 0.010 and 0.005
}
TEST(SimMesh, LatticeFollowsGrain) {
  // 0.3x0.3 rectangle, h = 0.01, grain (1,0) and grain rotated 30 deg: >= 80% of interior edges
  // (both endpoints non-boundary) lie within 2 deg of 0/60/120 deg from the grain
}
TEST(SimMesh, CornersAreVertices) {  // tee front at 0.02: every outline corner (>10 deg) appears exactly in rest
}
TEST(SimMesh, ThinPieceIsValid) {  // Review Focus 1: 0.5x0.02 rectangle at h = 0.02 -> all areas > 1e-12, all rest finite
}
TEST(SimMesh, InvalidPatternThrows) {  // open outline -> MeshError, what() contains "outline-closed"
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R SimMesh`
Expected: build fails.

- [ ] **Step 3: Implement** `sim_mesh.hpp` and `sim_mesh_builder.cpp` per the algorithm.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R SimMesh`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(garment): pattern to sim mesh with grain-aligned lattice`)

---

### Task 8: Triangle meshes, primitives and the test body

**Files:**
- Create: `libs/core/include/drape/core/triangle_mesh.hpp`, `libs/core/src/triangle_mesh.cpp`, `libs/avatar/CMakeLists.txt`, `libs/avatar/include/drape/avatar/{primitives.hpp,test_body.hpp}`, `libs/avatar/src/{primitives.cpp,test_body.cpp}`
- Modify: `libs/CMakeLists.txt` (add `avatar`)
- Test: `libs/avatar/tests/test_primitives.cpp`

**Interfaces:**
- Consumes: `Vec3d`, `BodyRegions`, `RegionCylinder`, `BodyRegion` (Tasks 1, 3).
- Produces:
  ```cpp
  namespace drape {   // core/triangle_mesh.hpp
  struct TriangleMesh { std::vector<Vec3d> vertices; std::vector<std::array<std::uint32_t, 3>> triangles; };  // CCW seen from outside
  double signedVolume(const TriangleMesh& m);
  bool isClosedManifold(const TriangleMesh& m);   // every undirected edge used by exactly two triangles, in opposite directions
  }
  namespace drape::avatar {
  TriangleMesh icosphere(const Vec3d& center, double radius, int subdivisions);
  TriangleMesh capsule(const Vec3d& a, const Vec3d& b, double radius, int segments = 32, int rings = 8);
  // Vertical (Y axis) elliptic cylinder from yBottom to yTop with semi-axes rx (X) and rz (Z),
  // closed by half-ellipsoid caps of height capHeight.
  TriangleMesh ellipticCapsule(double yBottom, double yTop, double rx, double rz, double capHeight, int segments = 48, int rings = 12);
  struct TestBody { std::vector<TriangleMesh> components; BodyRegions regions; };
  TestBody makeTestBody();
  }
  ```

**Test body** (meters; d_L = (sin45°, −cos45°, 0), d_R = (−sin45°, −cos45°, 0)):

| Component | Geometry |
|---|---|
| Torso | `ellipticCapsule(0.85, 1.38, 0.17, 0.11, 0.08)` (spans y 0.77–1.46) |
| Neck | `capsule((0,1.40,0), (0,1.58,0), 0.055)` |
| Head | `icosphere((0,1.68,0), 0.10, 3)` |
| Left arm | `capsule((0.19,1.37,0), (0.19,1.37,0) + 0.55·d_L, 0.045)` |
| Right arm | `capsule((−0.19,1.37,0), (−0.19,1.37,0) + 0.55·d_R, 0.045)` |

| Region | origin | axis | e0 | radius |
|---|---|---|---|---|
| Torso | (0, 1.47, 0) | +Y | +Z | 0.17 |
| Neck | (0, 1.47, 0) | +Y | +Z | 0.055 |
| ArmLeft | (0.19, 1.37, 0) | d_L | (cos45°, sin45°, 0) | 0.045 |
| ArmRight | (−0.19, 1.37, 0) | d_R | (−cos45°, sin45°, 0) | 0.045 |

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(Primitives, AllAreClosedAndOutward) {  // icosphere, capsule, ellipticCapsule: isClosedManifold && signedVolume > 0
}
TEST(Primitives, VolumesMatchAnalytic) {
  // icosphere r=0.1 subdiv 4 within 0.5% of 4/3*pi*r^3; capsule within 1% of pi*r^2*len + 4/3*pi*r^3;
  // ellipticCapsule within 1% of pi*rx*rz*(yTop-yBottom) + 4/3*pi*rx*rz*capHeight
}
TEST(TestBody, ComponentsAndRegions) {
  // 5 closed components; regions Torso, Neck, ArmLeft, ArmRight with the table values;
  // every region's e1() is unit length and perpendicular to axis and e0
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R "Primitives|TestBody"`
Expected: build fails.

- [ ] **Step 3: Implement** the primitives (icosphere by midpoint subdivision projected to the radius; capsules as UV-sphere halves joined by a cylinder) and `makeTestBody`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R "Primitives|TestBody"`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(avatar): primitives and static test body`)

---

### Task 9: Collision field and its bake

**Files:**
- Create: `libs/core/include/drape/core/collision_field.hpp`, `libs/core/src/collision_field.cpp`, `libs/avatar/include/drape/avatar/collision_bake.hpp`, `libs/avatar/src/collision_bake.cpp`
- Test: `libs/avatar/tests/test_collision_field.cpp`

**Interfaces:**
- Consumes: `TriangleMesh`, `isClosedManifold` (Task 8); `icosphere`, `makeTestBody` (Task 8).
- Produces:
  ```cpp
  namespace drape {   // core/collision_field.hpp
  struct CollisionField {
    Vec3d origin{0, 0, 0}; double cellSize = 0.004; std::array<int, 3> dims{0, 0, 0}; double band = 0.02;
    std::vector<float> values;           // samples at origin + (i,j,k)*cellSize, x fastest; signed distance clamped to [-band, band]
    double sample(const Vec3d& p) const;  // trilinear; any p outside the sampled box -> +band
    Vec3d gradient(const Vec3d& p) const; // central differences of sample() with step cellSize, normalized; zero if |g| < 1e-9
  };
  }
  namespace drape::avatar {
  struct BakeOptions { double cellSize = 0.004; double padding = 0.05; double band = 0.02; };
  // Union of closed components (min of their fields). Throws std::invalid_argument if a component is not a closed manifold.
  CollisionField bakeCollisionField(std::span<const TriangleMesh> components, const BakeOptions& options = {});
  }
  ```

**Bake algorithm.** The grid covers the union bounding box ± padding, with dims = ⌈extent/cellSize⌉ + 1. Each component is baked separately:
1. For each triangle, visit the samples inside its bounding box ± band. Compute the exact point–triangle distance (Ericson, *Real-Time Collision Detection* §5.1.5) and keep the minimum.
2. Each kept sample takes its sign from the angle-weighted pseudonormal of the closest feature (Bærentzen & Aanæs 2005): the face normal for a face, the sum of the adjacent face normals for an edge, and Σ angle × face normal for a vertex.
3. Samples never reached (outside the band) get their sign by a 6-connected flood fill from the grid boundary through unreached samples: reached by the fill → +band, otherwise → −band.
4. Reached samples store clamp(sign·distance, −band, band).

The final value of each sample is the minimum across components.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(CollisionField, SphereDistanceAccuracy) {
  // icosphere r=0.1 subdiv 4, default options; 2000 points (mt19937 seed 7) with |p|-0.1 in [-0.015, 0.015]
  // -> |sample(p) - (|p|-0.1)| <= 0.001
}
TEST(CollisionField, SignInsideOutside) {  // sample(origin) == -0.02; sample((0.15,0,0)) == +0.02
}
TEST(CollisionField, GradientPointsOutward) {  // 200 points 0.005 outside: angle(gradient, radial) <= 5 deg
}
TEST(CollisionField, UnionIsMin) {
  // spheres r=0.1 at (+-0.08,0,0): sample(origin) < 0; at (0,0,0.12): positive and within 0.001 of the distance to the nearer sphere
}
TEST(CollisionField, OutsideGridIsFarOutside) {  // Review Focus 3
  // sample((10,10,10)) == band; gradient((10,10,10)) == zero; points 1e-6 beyond each face of the box -> band
}
TEST(CollisionField, RejectsOpenMesh) {  // icosphere minus one triangle -> std::invalid_argument
}
TEST(CollisionField, BakeTestBodyUnderTwoSeconds) {  // LABELS perf; Release: steady_clock time < 2.0 s
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R CollisionField -LE perf`
Expected: build fails.

- [ ] **Step 3: Implement** `collision_field.cpp` and `collision_bake.cpp` per the algorithm.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R CollisionField -LE perf && cmake --preset release && cmake --build --preset release && ctest --preset release -L perf`
Expected: all PASS, including the perf test in Release.

- [ ] **Step 5: Commit** (`feat(avatar): signed collision field bake`)

---

### Task 10: Initial placement

**Files:**
- Create: `libs/garment/include/drape/garment/placement.hpp`, `libs/garment/src/placement.cpp`
- Test: `libs/garment/tests/test_placement.cpp` (links `drape_avatar` for `makeTestBody`)

**Interfaces:**
- Consumes: `Pattern`, `Placement`, `BodyRegions` (Task 3); `SimMesh` (Task 7); `TestTeeGenerator` (Task 6); `makeTestBody` (Task 8).
- Produces (namespace `drape::garment`):
  ```cpp
  class PlacementError : public std::runtime_error { using std::runtime_error::runtime_error; };
  // One world position per sim vertex. Throws PlacementError naming the region when a piece's region is missing.
  std::vector<Vec3d> placePattern(const Pattern& pattern, const SimMesh& mesh, const BodyRegions& regions);
  ```

**Wrap formula** for a vertex at pattern (x, y) on a piece with placement P over cylinder C:
- R = P.wrapRadius if set, else C.radius + P.offset
- θ = P.centerAngle + P.axialSign · x / R
- a = P.axialAtOrigin + P.axialSign · y
- position = C.origin + a·C.axis + R·(cos θ · C.e0 + sin θ · C.e1())

With e1 = axis × e0, this mapping keeps every piece's outside face pointing away from the axis.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(Placement, FrontWrapsTorsoCylinder) {
  // tee M on the test body: every front vertex (x, y) is 0.23 (0.17+0.06) from the torso axis, sits at angle
  // x/0.23 from e0 toward e1, and has height 1.47 - L + y (all 1e-9)
}
TEST(Placement, OutsideFacesPointOutward) {
  // every tee triangle (CCW in pattern space) has a world normal with positive dot against the radial
  // direction from its region axis at the triangle centroid
}
TEST(Placement, SeamEndsStartClose) {  // every seam: both endpoint pairs closer than 0.25 m
}
TEST(Placement, RibRingCloses) {  // rib_close seam pairs start within 0.001 m of each other
}
TEST(Placement, MissingRegionThrows) {  // regions without ArmLeft -> PlacementError, what() contains "ArmLeft"
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R Placement`
Expected: build fails.

- [ ] **Step 3: Implement** `placePattern`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R Placement`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(garment): cylinder-wrap initial placement`)

---

### Task 11: Solver interface and CPU solver core (integration, pins, forces, stretch)

**Files:**
- Create: `libs/sim/CMakeLists.txt`, `libs/sim/include/drape/sim/{solver.hpp,cpu_solver.hpp}`, `libs/sim/src/{coloring.hpp,coloring.cpp,stretch.hpp,stretch.cpp,cpu_solver.cpp}`
- Modify: `libs/CMakeLists.txt` (add `sim`)
- Test: `libs/sim/tests/{sim_test_utils.hpp,test_coloring.cpp,test_stretch.cpp,test_cpu_solver.cpp}` (tests link `drape_garment`)

**Interfaces:**
- Consumes: `SimMesh` (Task 7), `FabricPhysical`, `SimQuality`, `testJersey` (Task 3), `CollisionField` (Task 9; only stored in this task).
- Produces (namespace `drape::sim`):
  ```cpp
  using GarmentId = Id;
  enum class SimPhase { Assembling, Settling };   // phase after build(): Settling
  struct SimSettings { Vec3d gravity{0, -kGravity, 0}; double collisionOffset = 0.003; double assemblyClosingSpeed = 0.25;
                       double assemblyExtraDamping = 10.0; double dampingScale = 5.0; };
  struct SimGarment { GarmentId id; SimMesh mesh; std::vector<Vec3d> initialPositions; std::vector<FabricPhysical> pieceFabric; };  // pieceFabric per piece index
  struct Pin { std::uint32_t particle = 0; Vec3d position{0, 0, 0}; };           // global particle index
  struct ExternalForce { std::uint32_t particle = 0; Vec3d force{0, 0, 0}; };    // newtons
  struct SimScene { std::vector<SimGarment> garments; const CollisionField* body = nullptr; std::vector<Pin> pins;
                    std::vector<ExternalForce> forces; SimSettings settings; };
  struct StepInput { double dt = 1.0 / 60.0; int substeps = 20; };
  struct SimStats { bool hasNonFinite = false; double kineticEnergy = 0; double rmsSpeed = 0; double maxSeamGap = 0; double maxBodyPenetration = 0; };
  struct ParticleRange { std::uint32_t offset = 0; std::uint32_t count = 0; };
  struct Snapshot { std::vector<Vec3f> positions, velocities; std::vector<SimPhase> phases; std::vector<double> assemblyTime; std::vector<float> seamStartGap; };
  class Solver {   // spec 5.2, plus range() and particleCount()
   public:
    virtual ~Solver() = default;
    virtual void build(const SimScene& scene) = 0;
    virtual void updateFabric(const GarmentId& g, const std::vector<FabricPhysical>& pieceFabric) = 0;
    virtual void setPhase(const GarmentId& g, SimPhase phase) = 0;
    virtual void step(const StepInput& in) = 0;
    virtual void readPositions(std::span<Vec3f> out) const = 0;   // out.size() == particleCount()
    virtual SimStats stats() const = 0;                           // valid right after build()
    virtual Snapshot snapshot() const = 0;
    virtual void restore(const Snapshot& s) = 0;
    virtual ParticleRange range(const GarmentId& g) const = 0;
    virtual std::size_t particleCount() const = 0;
  };
  int substepsFor(SimQuality q);   // Draft 10, Standard 20, Fine 30
  class CpuSolver final : public Solver { /* overrides all of the above */ };
  ```
  `updateFabric` takes the per-piece list, matching `SimGarment::pieceFabric`. Spec 5.2's single-fabric signature is generalized because a garment has several fabric slots.
- Internal (`src/`):
  - `greedyColor(std::span<const std::array<std::uint32_t,4>> items, std::uint32_t particleCount) -> std::vector<std::vector<std::uint32_t>>`. Unused slots are `UINT32_MAX`; first-fit in item order; deterministic.
  - `template<class S> void evalStretch(const std::array<Eigen::Matrix<S,3,1>,3>& x, const Eigen::Matrix<S,2,2>& dmInv, int which /*0 warp, 1 weft, 2 bias*/, S& C, std::array<Eigen::Matrix<S,3,1>,3>& grad)`. The solver uses it with `float`; tests use it with `double`.

**Particle setup:**
- Particles are the garments' vertices in scene order.
- Mass = vertexArea × weight of the vertex's piece fabric. w = 1/m; pinned particles have w = 0.
- Initial velocities are 0.

**Stretch element setup:**
- Per triangle, u = piece grain and v = (−u.y, u.x). Rest coordinates r_k = (rest_k·u, rest_k·v), Dm = [r1−r0, r2−r0], A = |det Dm|/2.
- Store Dm⁻¹, A, kWarp/kWeft/kBias and the strain limits.
- Then color the triangles (3 particles each).

**Per frame:** h = dt/substeps. Each substep runs:
1. **Predict** (particles with w > 0): v += h·(gravity if the garment is Settling, else 0) + h·w·f_ext; x_prev = x; x += h·v.
2. **Stretch**, color by color. For each triangle, solve warp, then weft, then bias. Each solve is a scalar XPBD update with fresh λ:
   - α̃ = α/h², Δλ = −C / (Σ w_k|∇_k C|² + α̃), x_k += w_k ∇_k C Δλ. Skip when the denominator < 1e-20.
   - Compliance α = 1/(k·A) with k = kWarp, kWeft or kBias.
   - Constraints (spec 5.4): f_u = F·u, f_v = F·v, where F = Ds·Dm⁻¹ and Ds = [x1−x0, x2−x0]. C_warp = |f_u|−1, C_weft = |f_v|−1, C_bias = n_u·n_v with n = f/|f|.
   - Gradients through f_c = (x1−x0)·D[0][c] + (x2−x0)·D[1][c], where D = Dm⁻¹:
     - ∂|f|/∂f = n.
     - ∂C_bias/∂f_u = (n_v − c·n_u)/|f_u| and ∂C_bias/∂f_v = (n_u − c·n_v)/|f_v|, where c = n_u·n_v.
     - ∇x0 = −(∇x1 + ∇x2).
   - **Strain limiting:** then, if |f_u| > 1 + limitWarp, apply C = |f_u| − (1 + limitWarp) with α = 0. Same for the weft.
3. *(Tasks 12–14 insert bending, seams, seam bending and body collision here, in that order.)*
4. **Pins:** x = pin position.
5. **Velocity:** v = (x − x_prev)/h, then v *= max(0, 1 − c·h), where c = dampingScale·fabric.damping, plus assemblyExtraDamping while Assembling.

**Stats:**
- `hasNonFinite`: any NaN/Inf in x or v.
- `kineticEnergy` = Σ ½ m|v|²; `rmsSpeed` = sqrt(mean |v|²). Both over particles with w > 0.
- `maxSeamGap` and `maxBodyPenetration` are 0 until Tasks 13 and 14.

**Test helpers** (`sim_test_utils.hpp`):
- `SimGarment flatGarment(const GarmentId& id, PatternPiece piece, double h, const FabricPhysical& f, const Vec3d& origin = {0,0,0})`: positions are origin + (rest.x, 0, rest.y).
- `std::vector<std::uint32_t> verticesWhere(const SimMesh&, std::function<bool(std::uint32_t)>)`.
- `int runUntilStill(Solver&, StepInput in, double rms, int holdFrames, double maxSeconds)`: returns the frames run, or −1 on timeout.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(Coloring, NoConflicts) {  // 500 random 3- and 4-particle items over 200 particles: each item in exactly one color; no color shares a particle
}
TEST(Stretch, GradientsMatchFiniteDifference) {  // double; random deformed triangle; warp, weft, bias: |analytic - central FD(1e-6)| < 1e-6
}
TEST(CpuSolver, FreeFall) {
  // 0.1x0.1 rectangle, h 0.02, testJersey with damping 0, no body; 30 frames at 10 substeps:
  // every particle dy == -1.22625 within 0.5%, |dx|,|dz| < 1e-6
}
TEST(CpuSolver, AssemblingDisablesGravity) {  // setPhase(Assembling): 30 frames -> max displacement < 1e-6
}
TEST(CpuSolver, PinsHold) {  // pinned particle within 1e-7 of its pin after 60 frames with gravity
}
TEST(CpuSolver, StripStretchWarp) {
  // 0.200x0.050 rectangle, grain (1,0), h 0.005, testJersey with damping 1.0, gravity 0, Settling.
  // Pin vertices with rest.x <= 1e-9. Load vertices with rest.x >= 0.2-1e-9 with 0.5 N total along +X, split by
  // tributary edge length. Run until rms < 1e-5 for 60 frames (max 30 s).
  // Strain = least-squares slope of world x vs rest x over rest.x in [0.04, 0.16], minus 1 -> within 5% of 0.025.
}
TEST(CpuSolver, StripStretchWeft) {  // same with grain (0,1): within 5% of 0.05
}
TEST(CpuSolver, UpdateFabricChangesStiffness) {  // warp strip: updateFabric with stretchWarp 800, re-settle -> within 5% of 0.0125
}
TEST(CpuSolver, SnapshotRestoreIsDeterministic) {  // snapshot at frame 10; +20 frames -> P; restore; +20 frames -> bitwise equal to P
}
TEST(CpuSolver, ThinPieceSteps) {  // Review Focus 1: 0.5x0.02 at h 0.02, gravity, 60 frames -> !hasNonFinite
}
TEST(CpuSolver, RangesAndCounts) {  // two garments: ranges contiguous in scene order; particleCount == sum
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R "Coloring|Stretch|CpuSolver"`
Expected: build fails.

- [ ] **Step 3: Implement** `solver.hpp`, `cpu_solver.hpp/.cpp`, `coloring.*` and `stretch.*` per the algorithm.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R "Coloring|Stretch|CpuSolver"`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(sim): CPU XPBD solver core with anisotropic stretch`)

---

### Task 12: Bending

**Files:**
- Create: `libs/sim/src/{bending.hpp,bending.cpp}`
- Modify: `libs/sim/src/cpu_solver.cpp` (build hinges; solve after stretch)
- Test: `libs/sim/tests/test_bending.cpp`

**Interfaces:**
- Consumes: `CpuSolver` internals (Task 11), `testStiff`, `testAniso` (Task 3).
- Produces (internal):
  ```cpp
  struct BendCoefficients { double p = 0, q = 0; };
  // p = (11*Bwarp - 3*Bweft)/8, q = (9*Bweft - Bwarp)/8, each clamped to >= 0.05*min(Bwarp, Bweft)   (spec 5.4)
  BendCoefficients bendingCoefficients(double bendWarp, double bendWeft);
  // B(phi) = p*sin^2(phi) + q*cos^2(phi), phi = angle between the rest edge direction and the piece grain
  double edgeBendStiffness(const BendCoefficients& c, const Vec2d& restEdgeDir, const Vec2d& grain);
  // x = {x1, x2, x3, x4}: x1, x2 are the vertices opposite the hinge in triangles A and B; x3, x4 are the hinge edge.
  // theta = atan2((n1 x n2) . e_hat, n1 . n2), with n1 = (x1-x3) x (x1-x4), n2 = (x2-x4) x (x2-x3), e_hat = (x4-x3)/|x4-x3|.
  // Gradients from Bridson, Marino & Fedkiw 2003, "Simulation of Clothing with Folds and Wrinkles", Sec. 4.
  template<class S> void evalDihedral(const std::array<Eigen::Matrix<S,3,1>,4>& x, S& theta, std::array<Eigen::Matrix<S,3,1>,4>& grad);
  ```

**Hinges:** one per interior edge (exactly two triangles of the same piece). Rest angle θ₀ = 0, and α = (A₁+A₂) / (2·B(φ)·|e|²), using rest areas and rest edge length (spec 5.4). Color the hinges (4 particles each) and solve them after stretch.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(Bending, Coefficients) {
  // (100e-6,100e-6) -> p == q == 100e-6; (100e-6,40e-6) -> p == 122.5e-6, q == 32.5e-6;
  // (100e-6,5e-6) -> q == 0.25e-6 (clamped)
}
TEST(Bending, GradientMatchesFiniteDifference) {  // double; random non-degenerate hinge; |analytic - central FD(1e-6)| < 1e-6
}
TEST(Bending, FlatSheetStaysFlat) {  // 0.2x0.2 testStiff, gravity 0, 60 frames -> max |y| < 1e-6
}
// Cantilever (spec 12.2 #2): strip length l+0.01, width 0.025, h 0.001; W = weight*9.81; l = 2*(B/W)^(1/3).
// Pin vertices with rest.x <= 0.01. x_clamp = mean rest.x of pinned vertices that have an unpinned neighbour.
// Tip = mean world position of vertices with rest.x >= length-1e-9. Substeps 30; run until rms < 1e-4 for 60 frames (max 10 s).
// theta = atan2(-tip.y, tip.x - x_clamp); overhang = mean tip rest.x - x_clamp;
// c = overhang * cbrt(cos(theta/2) / (8*tan(theta))); G = W*c^3; require |G - B|/B <= 0.10.
TEST(Bending, CantileverIsotropic) {}   // testStiff, grain (1,0), B = 100e-6
TEST(Bending, CantileverAnisoWarp) {}   // testAniso, grain (1,0), B = 100e-6
TEST(Bending, CantileverAnisoWeft) {}   // testAniso, grain (0,1), B = 40e-6
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R Bending`
Expected: build fails.

- [ ] **Step 3: Implement** `bending.*` and wire the hinges into `CpuSolver`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R "Bending|CpuSolver"`
Expected: all PASS. The earlier CpuSolver tests still pass.

- [ ] **Step 5: Commit** (`feat(sim): direction-dependent dihedral bending`)

---

### Task 13: Seams, seam bending and assembly

**Files:**
- Create: `libs/sim/src/{seams.hpp,seams.cpp}`
- Modify: `libs/sim/src/cpu_solver.cpp` (seam pairs, seam hinges, phase bookkeeping, `maxSeamGap`)
- Test: `libs/sim/tests/test_seams.cpp`

**Interfaces:**
- Consumes: `SimMesh::seamPairs` (Task 7); `evalDihedral`, `edgeBendStiffness` (Task 12); `rectanglePiece`, `buildSimMesh` (Tasks 5, 7).
- Produces: seam behavior inside `CpuSolver`. The public interface is unchanged; `SimStats::maxSeamGap` becomes live.

**Behavior** (spec 5.4, 5.7):
- **Seam distance:** one constraint per seam pair. C = |x_a − x_b| − L_target, one-sided (applied only when C > 0), α = 0. Skip when |x_a − x_b| < 1e-9.
- **Assembly bookkeeping:** `setPhase(g, Assembling)` records each pair's current gap as `seamStartGap` and resets the garment's `assemblyTime` to 0.
  - While Assembling: each substep adds h to `assemblyTime`, and L_target = max(0, seamStartGap − assemblyClosingSpeed·assemblyTime).
  - While Settling: L_target = 0.
- **Seam hinges:** for consecutive pairs k, k+1 of the same seam, find the triangle on side A containing edge (a_k, a_{k+1}), with opposite vertex pA, and the triangle on side B containing (b_k, b_{k+1}), with opposite vertex pB. The hinge is x = {pA, pB, a_k, a_{k+1}} with θ₀ = 0.
  - B_hinge = ½·(B_A(φ_A) + B_B(φ_B)), each side measured against its own piece grain.
  - α = (A_A + A_B) / (2·B_hinge·|e|²), with rest |e| from side A.
- **Solve order** after bending: seam distances (colored, 2 particles each), then seam hinges (colored, 4 particles each).
- **`maxSeamGap`** = max |x_a − x_b| over all pairs.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(Seams, GapStatAfterBuild) {  // two 0.05x0.05 squares, facing edges 0.1 apart, seam sq1.right<->sq2.left: stats().maxSeamGap == 0.1 (1e-6)
}
TEST(Seams, CloseAtClosingSpeed) {
  // same setup, testJersey, gravity 0, setPhase(Assembling), 10 substeps:
  // after 12 frames maxSeamGap in [0.047, 0.053]; after 30 frames < 0.001
}
TEST(Seams, HoldWhenSettling) {  // after closing: setPhase(Settling), gravity on, pin sq1's left edge, 120 frames -> maxSeamGap < 0.001
}
TEST(Seams, SeamDoesNotHinge) {
  // testStiff, h 0.002, pin rest.x <= 0.01 on the first piece:
  // (a) one 0.06x0.02 strip; (b) two 0.03x0.02 pieces placed touching and sewn along the 0.02 edge.
  // Settled tip drop ratio (b)/(a) in [0.8, 1.25]
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R Seams`
Expected: FAIL (`maxSeamGap` is 0; seams not enforced).

- [ ] **Step 3: Implement** `seams.*` and wire them into `CpuSolver`, including the `setPhase` bookkeeping and `Snapshot` fields.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R "Seams|Bending|CpuSolver"`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(sim): seams with bounded-speed assembly and seam bending`)

---

### Task 14: Body collision with friction

**Files:**
- Create: `libs/sim/src/{collision.hpp,collision.cpp}`
- Modify: `libs/sim/src/cpu_solver.cpp` (collision after seam hinges; `maxBodyPenetration`), `libs/avatar/include/drape/avatar/primitives.hpp`, `libs/avatar/src/primitives.cpp` (add `box`)
- Test: `libs/sim/tests/test_body_collision.cpp` (links `drape_avatar`), `libs/avatar/tests/test_primitives.cpp` (add a box test)

**Interfaces:**
- Consumes: `CollisionField::sample/gradient` (Task 9); `icosphere`, `bakeCollisionField` (Tasks 8, 9).
- Produces:
  - `TriangleMesh box(const Vec3d& center, const Vec3d& halfExtents, const Mat3d& rotation);` in `drape::avatar` (12 triangles, outward)
  - `SimStats::maxBodyPenetration` becomes live: max(0, max over particles of −sample(x)).

**Collision step** (spec 5.5), for each particle with w > 0 when `scene.body` is set:
1. φ = sample(x); r = collisionOffset + thickness/2 of the particle's piece fabric.
2. If φ < r: n = gradient(x). Skip if n is zero.
3. d = r − φ; x += d·n.
4. **Friction:** Δ = x − x_prev and Δt = Δ − (Δ·n)n. If |Δt| ≤ μ·d, then x −= Δt (static). Otherwise x −= Δt·(μ·d/|Δt|) (kinetic). μ = fabric friction.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(Primitives, BoxIsClosed) {  // isClosedManifold && signedVolume == 8*hx*hy*hz (1e-12)
}
// sphere fixture: icosphere(origin, 0.15, 4) baked with default options; r_c = 0.003 + 0.0006/2 = 0.0033 (testJersey)
TEST(BodyCollision, ParticlesRestOnSphere) {  // 0.004x0.004 piece (h 0.004) starting 0.05 above the top; 120 frames -> every particle phi in [r_c-0.0005, r_c+0.0005]
}
TEST(BodyCollision, SheetDrapesOverSphere) {
  // spec 12.2 #4: 0.6x0.6 sheet, h 0.01, testJersey, centred 0.05 above the top; run until rms < 0.002 for 60 frames (max 20 s)
  // -> !hasNonFinite, maxBodyPenetration <= 0.002, vertex nearest the sheet centre has y within 0.002 of 0.15 + r_c
}
TEST(BodyCollision, StartsInsideIsResolved) {  // Review Focus 2: same sheet with its centre 0.01 below the top; 60 frames -> !hasNonFinite, maxBodyPenetration <= 0.002
}
// slope fixture: box(center chosen so its top face passes through the origin, halfExtents (0.5,0.05,0.5), rotation 20 deg about Z);
// a 0.02x0.02 piece (h 0.01) laid 0.004 above the top face
TEST(BodyCollision, FrictionHoldsOnGentleSlope) {}    // friction 0.5: displacement along the slope after 120 frames < 0.002
TEST(BodyCollision, SlidesWhenSlippery) {}            // friction 0.2: displacement along the slope after 120 frames > 0.05
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R "BodyCollision|Primitives"`
Expected: build fails (`box` missing); collision tests fail.

- [ ] **Step 3: Implement** `box`, `collision.*` and the solver wiring.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R "BodyCollision|Primitives|Seams|Bending|CpuSolver"`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(sim): body collision with positional friction`)

---

### Task 15: Drape runner (phases and stability guard)

**Files:**
- Create: `libs/sim/include/drape/sim/drape_runner.hpp`, `libs/sim/src/drape_runner.cpp`
- Test: `libs/sim/tests/test_drape_runner.cpp`

**Interfaces:**
- Consumes: `Solver` and the types around it (Task 11).
- Produces (namespace `drape::sim`):
  ```cpp
  inline constexpr std::string_view kDrapeFailureMessage =
      "Drape couldn't settle this garment. Try Draft quality or a different fabric.";
  struct DrapeOptions { int substeps = 20; double frameDt = 1.0 / 60.0; double maxAssemblySeconds = 3.0; double seamGapDone = 0.001;
                        double settleRmsSpeed = 0.002; int settleHoldFrames = 60; double maxSimSeconds = 20.0;
                        int snapshotEvery = 30; int maxRetries = 3; int energyRiseFrames = 60; };
  struct DrapeResult { bool settled = false; bool failed = false; std::string message; double simulatedSeconds = 0;
                       int retries = 0; int framesRun = 0; double assemblyEndSeamGap = 0; };
  DrapeResult drapeToRest(Solver& solver, const std::vector<GarmentId>& garments, const DrapeOptions& options = {});
  ```

**Algorithm** (spec 5.7, 5.8):
1. **Start:** set every garment to Assembling. Take a frame-0 snapshot that records the solver snapshot, simulated time, runner phase and phase time.
2. **Each frame:**
   1. If time ≥ maxSimSeconds − 1e-9, return `settled = false, failed = false`. All time comparisons use this 1e-9 tolerance so accumulated float error in 1/60 steps doesn't shift a transition by a frame.
   2. Otherwise `step({frameDt, substeps})`, add frameDt to the time, then read stats.
   3. Count consecutive frames whose kineticEnergy is greater than the previous frame's (`riseFrames`). Reset the count on any phase change.
3. **Failure** means `hasNonFinite` or `riseFrames ≥ energyRiseFrames`. On failure:
   - Increment retries. If retries > maxRetries, return `failed = true`, `message = kDrapeFailureMessage`, `retries = maxRetries`.
   - Otherwise restore the last snapshot (solver state, time and phase), double the substeps, reset the counters and continue.
4. **Assembling → Settling** when maxSeamGap < seamGapDone or phase time ≥ maxAssemblySeconds − 1e-9. Record `assemblyEndSeamGap`, then set every garment to Settling.
5. **Settled:** in Settling, count consecutive frames with rmsSpeed < settleRmsSpeed. At settleHoldFrames, return `settled = true`.
6. **Snapshots:** after every `snapshotEvery`-th kept frame, take a new snapshot.

`framesRun` counts every `step` call, including rolled-back frames. `simulatedSeconds` is the kept simulated time.

- [ ] **Step 1: Write the failing tests** (a `ScriptedSolver : Solver` returns scripted stats per frame. It records the substeps passed to `step` and which snapshot was restored; its snapshots store the frame index in `positions[0].x()`.)

```cpp
TEST(Runner, SettlesAfterStillFrames) {  // gap 0, rms 0 -> Settling after frame 1; settled with framesRun == 61
}
TEST(Runner, AssemblyEndsAtTimeout) {  // gap always 0.01 -> Settling at frame 180; assemblyEndSeamGap == 0.01
}
TEST(Runner, RetriesOnNonFinite) {  // NaN once at frame 40 -> restored frame-30 snapshot; later steps use substeps 40; retries == 1; settles
}
TEST(Runner, FailsAfterThreeRetries) {  // NaN on frame 10 and on the 10th frame after every restore -> failed, retries == 3, message == kDrapeFailureMessage
}
TEST(Runner, RisingEnergyIsFailure) {  // kinetic energy strictly increasing for 60 frames -> retries == 1
}
TEST(Runner, StopsAtMaxSimSeconds) {  // rms always 1 -> settled false, failed false, |simulatedSeconds - 20| <= 1/60
}
TEST(Runner, HangingSheetSettlesSymmetric) {
  // real CpuSolver: 0.5x0.5 sheet, h 0.01, testJersey, positions (x, 0.5 + y, 0), top corners (0,1,0) and (0.5,1,0) pinned;
  // substeps 20 -> settled within 15 s; |centre-of-mass x - 0.25| <= 0.002; |y(bottom-left) - y(bottom-right)| <= 0.002
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R Runner`
Expected: build fails.

- [ ] **Step 3: Implement** `drape_runner.cpp`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R Runner`
Expected: all PASS.

- [ ] **Step 5: Commit** (`feat(sim): drape runner with phases and stability guard`)

---

### Task 16: Scene builder and the test tee drape

**Files:**
- Create: `libs/sim/include/drape/sim/scene_builder.hpp`, `libs/sim/src/scene_builder.cpp`
- Test: `libs/sim/tests/test_tee_drape.cpp` (links `drape_garment`, `drape_avatar`)

**Interfaces:**
- Consumes: everything above.
- Produces (namespace `drape::sim`):
  ```cpp
  // pieceFabric[i] = fabricBySlot.at(pattern.pieces[i].fabricSlot); a missing slot -> std::invalid_argument naming the slot
  SimGarment makeSimGarment(const GarmentId& id, const Pattern& pattern, SimMesh mesh, std::vector<Vec3d> positions,
                            const std::map<std::string, FabricPhysical>& fabricBySlot);
  ```

**Pipeline** (used by the tests and by Task 17):
1. `makeTestBody()` → `bakeCollisionField(body.components)`.
2. `TestTeeGenerator().generate(resolveSpec(specTable(), size))`.
3. `buildSimMesh(pattern, particleDistance(q))`.
4. `placePattern(pattern, mesh, body.regions)`.
5. `makeSimGarment("tee", ..., {{"body", testJersey()}, {"rib", testRib()}})`.
6. `CpuSolver::build({{garment}, &field})`.
7. `drapeToRest(solver, {"tee"}, {.substeps = substepsFor(q)})`.

The **hem** is the vertices of `front` and `back` with rest.y < 1e-9.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(SceneBuilder, ResolvesFabricPerPiece) {  // tee: pieceFabric[neck_rib] == testRib(), pieceFabric[front] == testJersey()
}
TEST(SceneBuilder, MissingSlotThrows) {  // no "rib" -> std::invalid_argument, what() contains "rib"
}
// Shared drape assertions: settled && !failed; assemblyEndSeamGap < 0.001; stats().maxBodyPenetration <= 0.002;
// mean hem world y in [0.65, 0.85]; simulatedSeconds <= 15
TEST(TeeDrape, DraftOnTestBody) {}       // size M, Draft
TEST(TeeDrape, AllSizesDraft) {}         // LABELS slow; XS..XXL, Draft
TEST(TeeDrape, StandardOnTestBody) {}    // LABELS slow; size M, Standard
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build --preset dev && ctest --preset dev -R "SceneBuilder|TeeDrape"`
Expected: build fails.

- [ ] **Step 3: Implement** `scene_builder.cpp`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `ctest --preset dev -R "SceneBuilder|TeeDrape"`
Expected: all PASS, including the `slow` ones.

If a drape assertion fails, use superpowers:systematic-debugging before changing any threshold. The thresholds come from the spec.

- [ ] **Step 5: Commit** (`feat(sim): scene builder; test tee drapes on the test body`)

---

### Task 17: Developer dump tool, preview render and hand-off

**Files:**
- Create: `tools/drape_dump/CMakeLists.txt`, `tools/drape_dump/{main.cpp,obj_writer.hpp,obj_writer.cpp}`, `tools/render_obj.py`
- Modify: `README.md` (tool usage)
- Test: ctest `DrapeDumpSmoke` (registered in `tools/drape_dump/CMakeLists.txt`)

**Interfaces:**
- Consumes: the Task 16 pipeline.
- Produces:
  - `void writeObj(const std::filesystem::path& path, std::span<const std::pair<std::string, TriangleMesh>> objects);` writes one `o <name>` block per object, with 1-based global vertex indices.
  - CLI `drape_dump --out <file.obj> [--size XS|S|M|L|XL|XXL] [--quality draft|standard|fine]` (defaults M, draft).
    - Writes objects `body_0`…`body_4` and `tee`.
    - Prints exactly one line: `settled=<0|1> failed=<0|1> seconds=<%.2f> retries=<n> seam_gap_mm=<%.2f> penetration_mm=<%.2f> particles=<n>`.
    - Exits 0 when settled and not failed, 1 otherwise. Bad arguments print `usage: drape_dump --out <file.obj> [--size XS|S|M|L|XL|XXL] [--quality draft|standard|fine]` and exit 2.
  - `python3 tools/render_obj.py <in.obj> <out.png>` draws a 1600×1000 px figure with two panels (front view from +Z, side view from +X). Body objects are light grey, the garment is blue, with Lambert shading from light direction (0.3, 0.6, 1.0) normalized. It uses matplotlib `Poly3DCollection`.

- [ ] **Step 1: Register the failing smoke test.** `add_test(NAME DrapeDumpSmoke COMMAND drape_dump --out ${CMAKE_BINARY_DIR}/smoke.obj)`

- [ ] **Step 2: Run it to verify it fails**

Run: `cmake --build --preset dev; ctest --preset dev -R DrapeDumpSmoke`
Expected: FAIL (target missing).

- [ ] **Step 3: Implement** `obj_writer.*`, `main.cpp` and `render_obj.py`.

- [ ] **Step 4: Verify the tool and look at the result**

Run: `./build/dev/tools/drape_dump/drape_dump --out out/tee_M.obj && python3 tools/render_obj.py out/tee_M.obj out/tee_M.png`
Expected:
- The summary line shows `settled=1 failed=0` and exit code 0.
- Open `out/tee_M.png` with the Read tool and confirm: the tee hangs from the shoulders, the sleeves surround the arms, the rib rings the neck, and there are no spikes or exploded triangles.
- Send the PNG to Ian.

- [ ] **Step 5: Full verification**

Run: `ctest --preset dev -LE perf && cmake --preset release && cmake --build --preset release && ctest --preset release -L perf`
Expected: every test PASSes, including `slow` in dev and `perf` in release.

- [ ] **Step 6: Commit** (`feat(tools): drape_dump CLI and OBJ preview`)

- [ ] **Step 7: Copy the repository to Ian's Mac.**
  1. `git bundle create /mnt/user-data/outputs/drape.bundle --all`.
  2. Commit the bundle into the connected Dev 2 folder.
  3. On the device, in `Dev 2/drape`, run `git init -q && git fetch -q ../drape.bundle main && git checkout -q -f -B main FETCH_HEAD`. The folder already holds the spec as an untracked file; `-f` checks out the identical committed copy over it.
  4. Confirm that `git log --oneline` there matches.
