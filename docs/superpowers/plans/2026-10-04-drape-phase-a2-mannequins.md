# Drape Phase A2 — Mannequins Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the primitive test body with male and female MakeHuman-based mannequins (egg head, toned build, smooth feet, 45° pose), and drape the test tee on both.

**Architecture:** A developer-run Python builder (`tools/mannequin/`, numpy only) turns pinned CC0 MakeHuman files into committed assets: triangle OBJ, landmark and region JSON, and test-only check JSON. A C++ loader in `drape_avatar` reads those assets. The collision bake, placement and solver are unchanged. Every garment-facing test and tool switches to the mannequins; the primitive test body stays for fast unit tests.

**Tech Stack:** C++20, CMake ≥ 3.28 + Ninja, GoogleTest, Eigen 3.4.x, nlohmann/json; Python ≥ 3.10 with numpy (builder and its unit tests only), matplotlib (preview only).

**Spec:** `docs/superpowers/specs/2026-10-04-drape-mannequins-design.md` (Phase A2), which amends `docs/superpowers/specs/2026-10-01-drape-milestone-1-design.md` Sections 8.1, 12.5 and 17.

## Global Constraints

- **Source:** `github.com/makehumancommunity/makehuman` at commit `a8bc2d54ff0ac92e78ff71431b1023eda42bf482`; assets are CC0. Downloads come from `https://raw.githubusercontent.com/makehumancommunity/makehuman/a8bc2d54ff0ac92e78ff71431b1023eda42bf482/<path>` and are checked against `tools/mannequin/manifest.sha256` (Task 2). No other source.
- **No network at build or test time:** builder outputs are committed. Only `build_mannequins.py` touches the network, and only with an empty cache.
- **Frame:** metres, right-handed, Y up, lowest vertex at y = 0, facing +Z, x = 0 on the symmetry plane, z = 0 midway between the ankle joints. The character's left is +x.
- **Targets:** male height 1.80 m, female 1.70 m (±1 cm in tests, ±1 mm in the builder). Arms 45° and legs 5° from vertical, measured in the frontal (x–y) projection. MakeHuman's forward swing of the arms is kept.
- **Build settings:** gender 1.0 (male) / 0.0 (female); age 0.5; muscle 0.75; weight 0.5; proportions 1.0; race 1/3 each; breast modifiers at MakeHuman defaults, so no breast targets apply.
- **Vertex order:** the OBJ holds only the `body` group's 13,380 vertices, in ascending `base.obj` index order, so Phase C targets map one-to-one.
- **Determinism:** the builder writes byte-identical outputs on every run. Floats are printed with 6 decimals; JSON uses sorted keys and an indent of 1.
- **Warnings:** our C++ targets compile with `-Wall -Wextra -Wpedantic -Werror` (existing `DRAPE_WARNINGS`).

## Review Focus

1. **Corrupted or tampered builder cache.** A cached file whose SHA-256 no longer matches the manifest must stop the build with an error naming the file, never silently build a different body (test in Task 2).
2. **Mannequin JSON from another builder version.** An asset with `"format"` ≠ 1 must be rejected with an error naming the file and the format, not half-parsed (test in Task 6).
3. **Mannequin JSON missing a region.** The loader must refuse an asset lacking any of the eight regions, naming the region, so placement never fails later with a confusing error (test in Task 6).
4. **A mannequin OBJ with a non-triangle face or a bad index.** It must fail with `file:line`, not crash or mis-index (test in Task 6).
5. **Assets directory not where the binary expects.** It must fail with an error naming the full path it tried (test in Task 6).

---

## File Structure

```
cmake/DrapeDependencies.cmake            Eigen pinned to 3.4.x (Task 1)
tools/mannequin/
  CMakeLists.txt                         ctest MannequinBuilderUnit (Task 2)
  manifest.sha256                        26 pinned source files + hashes (Task 2)
  makehuman.py                           fetch, parse OBJ/targets/rig, macro weights (Task 2)
  style.py                               egg head, toe smoothing (Task 3)
  pose.py                                skinning pose, frame, triangulation (Task 4)
  measure.py                             landmarks, regions, girths (Task 5)
  build_mannequins.py                    CLI: build both bodies, write assets (Task 5)
  test_makehuman.py, test_style.py, test_pose.py, test_measure.py
assets/mannequins/                       committed outputs (Task 5)
  male.obj female.obj male.json female.json male.checks.json female.checks.json LICENSE.md
libs/avatar/include/drape/avatar/mannequin.hpp   loader API (Task 6)
libs/avatar/src/mannequin.cpp, libs/avatar/src/obj_reader.{hpp,cpp}
libs/avatar/tests/test_mannequin.cpp, test_collision_perf.cpp (modified)
libs/sim/tests/tee_drape_helpers.hpp, test_tee_drape.cpp, test_tee_drape_slow.cpp (modified, Task 7)
tools/drape_dump/main.cpp, CMakeLists.txt; tools/render_obj.py (modified, Task 7)
docs/…/2026-10-01-drape-milestone-1-design.md, README.md (modified, Task 8)
```

---

### Task 1: Pin Eigen 3.4 and harden the sheet-settle test

Ian's Mac now has Homebrew's Eigen 5.0.1 (unversioned `eigen`) and 3.4.1 (`eigen@3`, keg-only). Against Eigen 5, `BodyCollision.DrapedSheetComesToRest` ended at 0.55 mm/s after 10 s; its limit is 0.5 mm/s. The mannequin drape tests must run there in Phase B.

**Files:**
- Modify: `cmake/DrapeDependencies.cmake:11-14`
- Modify: `libs/sim/tests/test_body_collision.cpp` (`DrapedSheetComesToRest`)
- Modify: `README.md` (Mac setup line)

**Interfaces:** none.

- [ ] **Step 1: Pin Eigen.** Replace the `find_package(Eigen3 …)` block in `cmake/DrapeDependencies.cmake` with the lines below. This is a source edit, not a terminal command; CMake reads it at configure time. (As built in Task 1: the range is `<4` because Eigen 3.4.0's version file rejects `3.4...<5` on Linux.)

```cmake
if(APPLE)
  list(APPEND CMAKE_PREFIX_PATH /opt/homebrew/opt/eigen@3 /usr/local/opt/eigen@3)  # keg-only Homebrew Eigen 3.4
endif()
find_package(Eigen3 3.4...<4 REQUIRED NO_MODULE)
```

- [ ] **Step 2: Give the sheet 20 s, not 10 s.** In `DrapedSheetComesToRest`, change the frame limit from 600 to 1200. Change the comment to "0.5 mm/s RMS held for 1 s within 20 s simulated". The 0.5 mm/s threshold stays.
- [ ] **Step 3: Verify.**

Run: `cmake --preset dev && cmake --build --preset dev && ctest --preset dev -R "DrapedSheetComesToRest" --output-on-failure`
Expected: PASS. The configure output reports Eigen 3.4.x.

- [ ] **Step 4: README.** The Mac line becomes `brew install cmake ninja eigen@3 git`.
- [ ] **Step 5: Commit** (`build: pin Eigen 3.4.x (Homebrew eigen@3 on Mac); sheet settle test gets 20 s`).

---

### Task 2: MakeHuman source access and macro weights

**Files:**
- Create: `tools/mannequin/{manifest.sha256, makehuman.py, test_makehuman.py, CMakeLists.txt}`
- Modify: `CMakeLists.txt` (add `add_subdirectory(tools/mannequin)` after `tools/drape_dump`)

**Interfaces:**
- Produces (Python, `tools/mannequin/makehuman.py`):
  - `PINNED_COMMIT: str`, `RAW_BASE: str` (Global Constraints values)
  - `class FetchError(Exception)`
  - `load_manifest(path: Path) -> dict[str, str]` (relative path → sha256 hex)
  - `fetch(relpath: str, cache_dir: Path, manifest: dict[str, str]) -> Path`: downloads to `cache_dir/relpath` if absent, then verifies. Raises `FetchError` whose message contains `relpath` on a hash mismatch, and contains the URL on a download failure.
  - `@dataclass BaseMesh: vertices: np.ndarray (19158×3, metres), body_quads: np.ndarray (13378×4 int), body_ids: np.ndarray (sorted unique body vertex indices, 13380)`
  - `read_base_obj(path: Path) -> BaseMesh`: decimetres × 0.1; quads from group `body` only; all vertices kept, since `joint-*` helpers drive the skeleton.
  - `read_target(path: Path) -> tuple[np.ndarray, np.ndarray]`: (indices int, offsets K×3 metres); skips `#` and blank lines.
  - `apply_targets(base: np.ndarray, weights: dict[str, float], cache_dir: Path, manifest: dict[str, str]) -> np.ndarray`: base + Σ weight·offsets; each target is fetched as `makehuman/data/targets/<key>`.
  - `macro_values(gender, age, muscle, weight, height, proportions, races=(1/3, 1/3, 1/3)) -> dict[str, float]`: the `*Val` formulas of MakeHuman `apps/human.py` (`_setGenderVals`, `_setAgeVals`, `_setMuscleVals`, `_setWeightVals`, `_setHeightVals`, `_setBodyProportionVals`). Keys are the value names: `male`, `female`, `baby`, `child`, `young`, `old`, `caucasian`, `asian`, `african`, `minmuscle`, `averagemuscle`, `maxmuscle`, `minweight`, `averageweight`, `maxweight`, `minheight`, `averageheight`, `maxheight`, `uncommonproportions`, `regularproportions`, `idealproportions`.
  - `macro_target_weights(values: dict[str, float]) -> dict[str, float]`: keys are paths relative to `makehuman/data/targets/`. Each weight is the product of the values named by the filename's dash-separated components; the group words `universal`, `height` and `proportions` count as 1. Families: `macrodetails/{race}-{gender}-{age}`, `macrodetails/universal-{gender}-{age}-{muscle}-{weight}`, `macrodetails/height/{gender}-{age}-{muscle}-{weight}-{min|max}height`, `macrodetails/proportions/{gender}-{age}-{muscle}-{weight}-{ideal|uncommon}proportions`, all with the `.target` suffix. Zero weights are dropped.
  - `@dataclass Rig: parent: dict[str, str | None], head_joint: dict[str, str], joints: dict[str, list[int]], weights: dict[str, tuple[np.ndarray, np.ndarray]]`
  - `read_rig(skel_path: Path, weights_path: Path) -> Rig`; `joint_pos(vertices, rig, joint) -> np.ndarray` (mean of the joint's vertices); `descendants(rig, bone) -> list[str]` (including `bone`)

- [ ] **Step 1: Write the manifest.** `tools/mannequin/manifest.sha256` holds these 26 lines verbatim, in `sha256sum` format with two spaces and paths relative to the repository root of MakeHuman:

```
f6089cba01cb570a24712b41ab8a586ccd3cc5ef53dc266ca50b95c288956d2c  LICENSE.ASSETS.md
8e761e6624b8f54536409135d1636da63b32486a90d4897f84e121d144f6fb4c  makehuman/data/3dobjs/base.obj
99f179bce0aa850b45d4191a1d0d234c5851f881c057439470ded3bddf729a24  makehuman/data/rigs/default.mhskel
0f3641d651ae3d00ad6b4ccee43142edb109d3bd909d27d9e4139ef1beed8625  makehuman/data/rigs/default_weights.mhw
894abc1fbb3d28543a51fef16f89d5d4bdf9aa2e1534413339811a3d47818b7d  makehuman/data/targets/macrodetails/african-male-young.target
ed2e8c191cb6b87b4a2d97c80486acb2604fa549316c7aa2a738a5d5a14334dc  makehuman/data/targets/macrodetails/asian-male-young.target
70e228ba7164737dae664454394536fc5935fa48d333c1a97d77e2dc6eacc5f5  makehuman/data/targets/macrodetails/caucasian-male-young.target
4ba5396ddabda448ece15650a566fbebfbb10239256ccb201e8f883429e12249  makehuman/data/targets/macrodetails/universal-male-young-averagemuscle-averageweight.target
76a79d873dc0820ae336645903591f8e6a8adc69865210a6a66f49908caa810b  makehuman/data/targets/macrodetails/height/male-young-averagemuscle-averageweight-minheight.target
3baacc70187410ebba2b62c56a53775266ab859b7844d47e658d36ee78493621  makehuman/data/targets/macrodetails/height/male-young-averagemuscle-averageweight-maxheight.target
7bbd790604c908417ff8ca11e566e3577508d92a4f397a2ace0f89f4cc3aa373  makehuman/data/targets/macrodetails/proportions/male-young-averagemuscle-averageweight-idealproportions.target
c171ff9cc95e96273beb3e3b1969a01988c2d4982ebc6fe04bf81cf72ee3b295  makehuman/data/targets/macrodetails/universal-male-young-maxmuscle-averageweight.target
76a79d873dc0820ae336645903591f8e6a8adc69865210a6a66f49908caa810b  makehuman/data/targets/macrodetails/height/male-young-maxmuscle-averageweight-minheight.target
3baacc70187410ebba2b62c56a53775266ab859b7844d47e658d36ee78493621  makehuman/data/targets/macrodetails/height/male-young-maxmuscle-averageweight-maxheight.target
d9452857903d18a2100ca26105d5a61083e7064fbe16f035863446728755ec20  makehuman/data/targets/macrodetails/proportions/male-young-maxmuscle-averageweight-idealproportions.target
92d61eeb3c164b421fd5a7c3537ee45e7e1a51de4d49bf312e19c3df2be1d8fc  makehuman/data/targets/macrodetails/african-female-young.target
095fe79694fa19e1fe98d93009ec116199bd524e081c640351a10eccf2cca1eb  makehuman/data/targets/macrodetails/asian-female-young.target
118379f6e8ba9266247fdb8788a20e1df40a239f97ced0b9905bcbcc74f6e820  makehuman/data/targets/macrodetails/caucasian-female-young.target
4ba5396ddabda448ece15650a566fbebfbb10239256ccb201e8f883429e12249  makehuman/data/targets/macrodetails/universal-female-young-averagemuscle-averageweight.target
375857dff927f67da53786fc5ac0545c473f4959e3b326727b6b91f49ebdd2e3  makehuman/data/targets/macrodetails/height/female-young-averagemuscle-averageweight-minheight.target
3baacc70187410ebba2b62c56a53775266ab859b7844d47e658d36ee78493621  makehuman/data/targets/macrodetails/height/female-young-averagemuscle-averageweight-maxheight.target
0a3bd099b7a88150d56d68b69c07d3d3538cb57279e0e775d13f19d3dc3e339d  makehuman/data/targets/macrodetails/proportions/female-young-averagemuscle-averageweight-idealproportions.target
2e56b09b44a8497b927585447fdaac5ea7293521d1f3a3df1059495873c53f4e  makehuman/data/targets/macrodetails/universal-female-young-maxmuscle-averageweight.target
375857dff927f67da53786fc5ac0545c473f4959e3b326727b6b91f49ebdd2e3  makehuman/data/targets/macrodetails/height/female-young-maxmuscle-averageweight-minheight.target
3baacc70187410ebba2b62c56a53775266ab859b7844d47e658d36ee78493621  makehuman/data/targets/macrodetails/height/female-young-maxmuscle-averageweight-maxheight.target
12654c6dd22d81a9595f41f149f1988c141ca22a350d4c2b91fccd8f9b2ce23b  makehuman/data/targets/macrodetails/proportions/female-young-maxmuscle-averageweight-idealproportions.target
```

- [ ] **Step 2: Write the failing tests** (`test_makehuman.py`, `unittest`, fully offline):

```python
class MacroValues(unittest.TestCase):
    def test_corner_settings(self):
        v = makehuman.macro_values(gender=1.0, age=0.5, muscle=0.75, weight=0.5, height=0.75, proportions=1.0)
        expect = {"male": 1, "female": 0, "young": 1, "old": 0, "child": 0, "baby": 0,
                  "maxmuscle": 0.5, "averagemuscle": 0.5, "minmuscle": 0, "averageweight": 1,
                  "maxweight": 0, "minweight": 0, "maxheight": 0.5, "averageheight": 0.5, "minheight": 0,
                  "idealproportions": 1, "regularproportions": 0, "uncommonproportions": 0,
                  "african": 1/3, "asian": 1/3, "caucasian": 1/3}
        for k, x in expect.items(): self.assertAlmostEqual(v[k], x, places=12, msg=k)
    def test_low_height_uses_minheight(self):
        v = makehuman.macro_values(1.0, 0.5, 0.75, 0.5, 0.25, 1.0)
        self.assertAlmostEqual(v["minheight"], 0.5); self.assertAlmostEqual(v["averageheight"], 0.5)

class MacroTargetWeights(unittest.TestCase):
    def test_male_build(self):
        w = makehuman.macro_target_weights(makehuman.macro_values(1.0, 0.5, 0.75, 0.5, 0.75, 1.0))
        m = "macrodetails/"
        expect = {m+"african-male-young.target": 1/3, m+"asian-male-young.target": 1/3,
                  m+"caucasian-male-young.target": 1/3,
                  m+"universal-male-young-averagemuscle-averageweight.target": 0.5,
                  m+"universal-male-young-maxmuscle-averageweight.target": 0.5,
                  m+"height/male-young-averagemuscle-averageweight-maxheight.target": 0.25,
                  m+"height/male-young-maxmuscle-averageweight-maxheight.target": 0.25,
                  m+"proportions/male-young-averagemuscle-averageweight-idealproportions.target": 0.5,
                  m+"proportions/male-young-maxmuscle-averageweight-idealproportions.target": 0.5}
        self.assertEqual(set(w), set(expect))
        for k, x in expect.items(): self.assertAlmostEqual(w[k], x, places=12, msg=k)

class Parsers(unittest.TestCase):
    def test_read_target_scales_to_metres(self):      # file "# c\n\n12 1 -2 0.5\n" -> [12], [[0.1, -0.2, 0.05]]
    def test_read_base_obj_keeps_body_group_only(self):  # 6 vertices, one quad in "g helper", one in "g body"
        # -> body_quads == [[2,3,4,5]] (0-based), body_ids == [2,3,4,5], vertices scaled by 0.1

class Fetch(unittest.TestCase):
    def test_hash_mismatch_names_the_file(self):        # Review Focus 1
        # cache/a/b.txt exists with "x"; manifest {"a/b.txt": "0"*64}; fetch raises FetchError containing "a/b.txt"
```

- [ ] **Step 3: Register the tests with ctest.** `tools/mannequin/CMakeLists.txt`: run `find_package(Python3 COMPONENTS Interpreter)`, then check `import numpy` with `execute_process`. When both succeed, `add_test(NAME MannequinBuilderUnit COMMAND ${Python3_EXECUTABLE} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR} -p "test_*.py")`. Otherwise print `message(STATUS "MannequinBuilderUnit skipped: python3 with numpy not found")`.
- [ ] **Step 4: Run to verify it fails.** Run: `cmake --preset dev && ctest --preset dev -R MannequinBuilderUnit --output-on-failure`. Expected: FAIL (`No module named 'makehuman'`).
- [ ] **Step 5: Implement `makehuman.py`** with the Interfaces above. Use `urllib.request` for fetching; set `ssl` defaults from the environment, never disabling verification.
- [ ] **Step 6: Run to verify it passes.** Same command. Expected: PASS.
- [ ] **Step 7: Commit** (`feat(mannequin): pinned MakeHuman source access and macro weights`).

---

### Task 3: Egg head and smooth feet

**Files:**
- Create: `tools/mannequin/style.py`, `tools/mannequin/test_style.py`

**Interfaces:**
- Consumes: `Rig`, `descendants` (Task 2).
- Produces (`style.py`):
  - `region_weights(rig, roots: list[str], exclude: set[str] = set(), n: int) -> np.ndarray` (n,): the summed skinning weights over the descendants of `roots`, minus `exclude`, clipped to [0, 1]. Head region: `roots=["head"]`. Toes: `roots=["foot.L", "foot.R"], exclude={"foot.L", "foot.R"}`.
  - `@dataclass Egg: centre: np.ndarray (3,), radii: np.ndarray (3,) = (ax, ay, az), taper: float = 0.85`
  - `egg_value(egg, p: np.ndarray) -> np.ndarray`: with q = p − centre, k = 1 for q_y ≥ 0, else 1 − (1 − taper)·min(1, −q_y/ay); value = (q_x/(ax·k))² + (q_y/ay)² + (q_z/(az·k))². It equals 1 on the surface.
  - `project_to_egg(egg, p) -> np.ndarray`: the point on the ray from the centre through p where `egg_value` = 1 (bisection on the ray parameter in [0, 4·max(radii)], 60 iterations).
  - `fit_egg(vertices, head_w, eye_y: float) -> Egg`: use the vertices with head_w ≥ 0.999. Width and depth are measured 3 cm above eye level, not at it: the rig has no ear bones, so measuring above the ears is how the spec's "ears excluded" is met.
    - y_top = max y; y_chin = min y of those with |x| ≤ 0.02; ay = (y_top − y_chin)/2; c_y = (y_top + y_chin)/2.
    - Forehead band: |y − (eye_y + 0.03)| ≤ 0.005. Half-width hx = (max x − min x)/2, half-depth hz = (max z − min z)/2, c_z = (max z + min z)/2, c_x = 0.
    - With t = (eye_y + 0.03 − c_y)/ay clamped to [−0.9, 0.9]: ax = hx/√(1 − t²), az = hz/√(1 − t²). This corrects the measured section to the egg's equator.
  - `make_egg_head(vertices, quads, head_w, egg) -> np.ndarray`: blend x + w·(P(x) − x), then 30 rounds of uniform-Laplacian relaxation (step 0.5·w over 1-ring neighbours), each followed by re-projecting w ≥ 0.999 vertices onto the egg. Reject any vertex move that flips one of its quads' normals against its pre-round normal.
  - `smooth_toes(vertices, quads, toe_w, rounds: int = 40) -> np.ndarray`: uniform-Laplacian, step 0.5·w. Vertices within 3 mm of the foot's lowest y move only in x and z.

- [ ] **Step 1: Write the failing tests** (`test_style.py`, synthetic meshes only):

```python
EGG = style.Egg(np.array([0.0, 1.62, 0.02]), np.array([0.075, 0.115, 0.095]))
def test_project_lands_on_surface():   # 500 random directions: |egg_value(project(c + d)) - 1| < 1e-9
def test_taper_halfway_down():         # horizontal radius at y = c_y - ay/2 equals ax*(1 - 0.15*0.5) within 1e-9
def test_fit_recovers_ellipsoid():     # quad UV-sphere scaled to radii (0.07, 0.11, 0.09) at (0, 1.6, 0.01), all w = 1,
                                       # eye_y = 1.6 - 0.03 -> fitted radii and centre within 1 mm
def test_egg_head_removes_bumps():     # UV-sphere (r 0.1) at EGG.centre plus a 2 cm "nose" bump, all w = 1 ->
                                       # every vertex within 1e-6 of the egg; every quad normal . (p - c) > 0
def test_zero_weight_vertices_never_move():   # egg head and toe smoothing leave w = 0 vertices bit-identical
def test_toe_smoothing_keeps_the_sole():      # bumpy grid at y in [0, 0.02], all w = 1 -> vertices that started
                                              # within 3 mm of the lowest y keep their y exactly
```

- [ ] **Step 2: Run to verify it fails.** `ctest --preset dev -R MannequinBuilderUnit --output-on-failure`. Expected: FAIL (`No module named 'style'`).
- [ ] **Step 3: Implement `style.py`.**
- [ ] **Step 4: Run to verify it passes.** Expected: PASS.
- [ ] **Step 5: Commit** (`feat(mannequin): egg head and smooth feet`).

---

### Task 4: Pose, frame and triangulation

**Files:**
- Create: `tools/mannequin/pose.py`, `tools/mannequin/test_pose.py`

**Interfaces:**
- Consumes: `Rig`, `joint_pos`, `descendants` (Task 2).
- Produces (`pose.py`):
  - `frontal_angle(d: np.ndarray) -> float`: degrees, `atan2(|d_x|, −d_y)`.
  - `@dataclass Posed: vertices: np.ndarray, joints: dict[str, np.ndarray]`. `joints` holds the posed `upperarm01.{L,R}____head`, `wrist.{L,R}____head`, `upperleg01.{L,R}____head` and `foot.{L,R}____head`.
  - `pose(vertices, rig, arm_deg: float = 45.0, leg_deg: float = 5.0) -> Posed`. Rotations are about +z through a joint, by θ such that the frontal angle hits the target. For the left side (+x), θ = target − current; the right side is mirrored, θ = current − target.
    - Every bone in `descendants("upperarm01.L")` gets T_armL (pivot: shoulder `upperarm01.L____head`).
    - Every bone in `descendants("upperleg01.L")` gets T_legL (pivot: hip `upperleg01.L____head`).
    - Every bone in `descendants("foot.L")` gets R(−θ_leg about the posed ankle) ∘ T_legL, which keeps the sole level. The right side is mirrored.
    - Skinning: v′ = v + Σ_b w_b·(T_b v − v). Unposed bones contribute nothing.
  - `to_drape_frame(vertices, body_ids, ankle_l, ankle_r) -> np.ndarray`: y −= min y over `body_ids`; z −= (ankle_l.z + ankle_r.z)/2. Apply the same shift to joints.
  - `triangulate(vertices, quads) -> np.ndarray` (2M×3): split each quad (a,b,c,d) along the shorter of a–c and b–d, as (a,b,c),(a,c,d) or (a,b,d),(b,c,d). This keeps orientation.

- [ ] **Step 1: Write the failing tests** (`test_pose.py`). Use a synthetic rig dict with bones `root`, `upperarm01.L/R`, `upperleg01.L/R`, `foot.L/R`, joints placed directly, and vertex strips fully weighted to one bone each:

```python
def test_arms_reach_45_degrees():      # strips at 40 deg -> frontal_angle(wrist - shoulder) == 45 +- 1e-9, both sides
def test_legs_reach_5_degrees():       # 6 deg -> 5 +- 1e-9, both sides
def test_soles_stay_level():           # horizontal foot strip weighted to foot.L stays horizontal (dy < 1e-12)
def test_unweighted_vertices_stay():   # vertices weighted to root are bit-identical
def test_mirror_symmetry():            # mirrored input -> mirrored output within 1e-12
def test_triangulate_short_diagonal(): # quad (0,0)(1,0)(1.1,1)(0,1): a-c longer -> (a,b,d),(b,c,d); normals match quad
def test_frame_puts_floor_at_zero():   # min y over body ids -> 0; z offset = mean ankle z
```

- [ ] **Step 2: Run to verify it fails.** Expected: FAIL (`No module named 'pose'`).
- [ ] **Step 3: Implement `pose.py`.**
- [ ] **Step 4: Run to verify it passes.** Expected: PASS.
- [ ] **Step 5: Commit** (`feat(mannequin): 45-degree pose, Drape frame and triangulation`).

---

### Task 5: Measurements, builder CLI and the committed assets

**Files:**
- Create: `tools/mannequin/measure.py`, `tools/mannequin/test_measure.py`, `tools/mannequin/build_mannequins.py`
- Create (generated, committed): `assets/mannequins/{male,female}.{obj,json,checks.json}`, `assets/mannequins/LICENSE.md`

**Interfaces:**
- Consumes: Tasks 2–4.
- Produces:
  - `measure.convex_hull_perimeter(points: np.ndarray (n×2)) -> float` (Andrew's monotone chain)
  - `measure.section_points(vertices, tris, y, vertex_mask) -> np.ndarray` (n×2 (x, z)): the intersection points of triangles whose 3 vertices are all in the mask with the plane at height y.
  - `measure.dominant_bones(rig, n: int) -> list[str]`: per vertex, the bone with the largest skinning weight (`""` if none).
  - `measure.landmarks_regions_girths(vertices, tris, dominant: list[str], rig, joints: dict[str, np.ndarray], egg) -> tuple[dict, dict, dict]`: (landmarks, regions, girths), with the definitions in Step 3. `joints` holds the posed joints from `pose.Posed` plus the unposed `breast.L____head`, `neck01____head` and `eye.L____head`, after the frame shift.
  - CLI: `python3 tools/mannequin/build_mannequins.py [--cache out/makehuman-cache] [--out assets/mannequins]`. It prints one report line per body: `male height=1.800 chest=… waist=… hip=… neck=… arm_l=45.00 arm_r=45.00 leg_l=5.00 leg_r=5.00`.
  - Asset formats (consumed by Task 6):
    - `<body>.obj`: line 1 `# Drape mannequin <body> — tools/mannequin/build_mannequins.py, MakeHuman a8bc2d5 (CC0)`, then `o body`, then `v %.6f %.6f %.6f`, then `f a b c` (1-based).
    - `<body>.json`: `{"format": 1, "body": "male", "source": {"makehuman_commit": "<full hash>", "builder_version": 1, "vertex_order": "makehuman base.obj body group, ascending index"}, "height": h, "girths": {"chest","waist","hip","neck"}, "landmarks": {name: [x,y,z]}, "regions": {"Torso": {"origin": [..], "axis": [..], "e0": [..], "radius": r}, …all 8}}`
    - `<body>.checks.json`: `{"format": 1, "egg": {"centre": [..], "radii": [..], "taper": 0.85}, "head_vertices": [0-based OBJ indices with head_w ≥ 0.999], "region_vertices": {region: [indices used for its radius]}}`

- [ ] **Step 1: Write the failing tests** (`test_measure.py`):

```python
def test_square_perimeter():    # unit-square corners plus an interior point -> 4.0 (1e-12)
def test_section_of_a_box():    # axis-aligned 0.2 x 0.1 box mesh cut at mid-height -> hull perimeter 0.6 (1e-9)
def test_mask_excludes_triangles(): # same box with half its vertices masked out -> only fully-masked triangles cut
```

- [ ] **Step 2: Run to verify it fails.** Expected: FAIL (`No module named 'measure'`).
- [ ] **Step 3: Implement `measure.py`.**
  - Vertex classes, by each vertex's dominant bone (largest weight):
    - *arm*: dominant bone is a descendant of `shoulder01.L` or `shoulder01.R`.
    - *head*: descendant of `head`.
    - *neck*: one of `neck01`, `neck02`, `neck03`.
    - *leg*: descendant of `upperleg01.L` or `upperleg01.R`.
    - *torso*: all others.
    - *non-arm*: every class except arm.
  - Landmarks. Body axis: x = 0, z_axis = the mid z-extent of torso vertices within ±1 cm of chest_y, where chest_y = the y of joint `breast.L____head`.
    - `neck_base` = (0, y of `neck01____head`, z_axis).
    - `chest` = the torso vertex with the largest z among those with |y − chest_y| ≤ 0.01.
    - `crotch` = the lowest body vertex with |x| ≤ 0.01 below the hip joint.
    - `waist` = (0, y_w, z_axis). y_w is the 1 cm step in [crotch_y + 0.10, chest_y] that minimises the non-arm section hull perimeter.
    - `hip` = (0, y_h, z_axis). y_h is the 1 cm step in [crotch_y, y_w] that maximises that perimeter.
    - `hps_left` = the highest non-arm vertex with x ∈ [r_neck, r_neck + 0.02] and |z − z_axis| ≤ 0.03, where r_neck is the Neck region radius. `hps_right` is mirrored.
    - `shoulder_left` = the highest vertex within 1.5 cm horizontally of the posed shoulder joint. `shoulder_right` is mirrored.
    - `ankle_left` / `ankle_right` = posed `foot.{L,R}____head`.
    - Extra keys for Task 6 angle tests: `shoulder_joint_*`, `wrist_*`, `hip_joint_*` (posed joints).
  - Regions. e0 is +Z for all vertical cylinders.
    - **Torso:** origin `neck_base`, axis +Y. Radius = max horizontal distance from the body axis over torso vertices with y ∈ [crotch_y, neck_base_y].
    - **Neck:** origin `neck_base`, axis +Y. Radius = max over neck vertices.
    - **Head:** origin egg centre, axis +Y. Radius = max(ax, az).
    - **ArmL:** origin `shoulder_joint_left`, axis = unit(wrist − shoulder), e0 = unit(+Y minus its axis component). Radius = max distance from the axis over left arm vertices whose axial coordinate is in [0, |wrist − shoulder|]. ArmR is mirrored.
    - **LegL:** origin `hip_joint_left`, axis = unit(ankle − hip), e0 = unit(+Z minus its axis component). Radius as for the arms, over [0, |ankle − hip|]. LegR is mirrored.
    - **Waist:** origin (0, y_w, z_axis), axis +Y. Radius = max horizontal distance over non-arm vertices with y ∈ [crotch_y, y_w].
  - Girths (non-arm hull perimeter): chest at chest_y, waist at y_w, hip at y_h, neck at neck_base_y + 0.02 over neck vertices.
- [ ] **Step 4: Run unit tests to verify they pass.** Expected: PASS.
- [ ] **Step 5: Implement `build_mannequins.py`.** For each body:
  1. Fetch files.
  2. Bisect the height macro h ∈ [0.5, 1.0] until |height − target| ≤ 0.0005 m, at most 40 iterations.
  3. Each evaluation runs: apply weighted targets → fit egg (eye_y from joint `eye.L____head`) → egg head → smooth toes → pose → Drape frame. Height is the max y over the body vertices.
  4. Measure, triangulate, keep body vertices only (remap indices), then write the three files.
  5. Write `LICENSE.md` once, with the CC0 statement, the pinned commit, the builder version and the vertex order.
- [ ] **Step 6: Build the assets and check determinism.**

Run: `python3 tools/mannequin/build_mannequins.py && cp -r assets/mannequins out/mannequins-run1 && python3 tools/mannequin/build_mannequins.py && diff -r out/mannequins-run1 assets/mannequins && echo IDENTICAL`
Expected: two report lines, with heights 1.800 and 1.700, arms 45.00 and legs 5.00 on each side; then `IDENTICAL`. Male chest is between 0.90 and 1.10, female between 0.80 and 1.00. If an angle, height or girth misses, debug the builder; do not edit the assets by hand.

- [ ] **Step 7: Commit** (`feat(mannequin): builder CLI and committed male/female mannequin assets`).

---

### Task 6: Mannequin loader and mannequin tests

**Files:**
- Create: `libs/avatar/include/drape/avatar/mannequin.hpp`, `libs/avatar/src/mannequin.cpp`, `libs/avatar/src/obj_reader.hpp`, `libs/avatar/src/obj_reader.cpp`, `libs/avatar/tests/test_mannequin.cpp`
- Modify: `libs/avatar/CMakeLists.txt`, `libs/avatar/tests/test_collision_perf.cpp`

**Interfaces:**
- Consumes: the asset formats (Task 5).
- Produces (namespace `drape::avatar`):

```cpp
enum class BodyType { Male, Female };
std::string_view bodyName(BodyType type);  // "male" | "female"
struct Landmarks { std::map<std::string, Vec3d> points; };
struct Mannequin {
  TriangleMesh mesh;                     // one closed component
  BodyRegions regions;                   // all 8 regions
  Landmarks landmarks;
  std::map<std::string, double> girths;  // chest, waist, hip, neck (m)
  double height = 0;
};
Mannequin loadMannequin(BodyType type);                                   // DRAPE_ASSET_DIR/mannequins
Mannequin loadMannequin(BodyType type, const std::filesystem::path& mannequinDir);
TriangleMesh readObjTriangles(const std::filesystem::path& path);         // obj_reader.hpp (internal)
```

- `DRAPE_ASSET_DIR`: `target_compile_definitions(drape_avatar PUBLIC DRAPE_ASSET_DIR="${PROJECT_SOURCE_DIR}/assets")`.
- Errors are `std::runtime_error`:
  - `"<path>: cannot open"`
  - `"<path>:<line>: <reason>"` for OBJ syntax (`face is not a triangle`, `vertex index out of range`)
  - `"<path>: unsupported format <n> (expected 1)"`
  - `"<path>: missing region '<Name>'"`
  - nlohmann parse errors rethrown with the path prefixed

- [ ] **Step 1: Write the failing tests** (`test_mannequin.cpp`). Load each body once into a static fixture, with `checks.json` read via nlohmann from `DRAPE_ASSET_DIR`:

```cpp
TEST(Mannequin, LoadsBothBodies)          // 13380 vertices, 26756 triangles, 8 regions, each body
TEST(Mannequin, ClosedAndManifold)        // every undirected edge used by exactly 2 triangles; min area >= 1e-10
TEST(Mannequin, HeightAndFloor)           // |height - 1.80 / 1.70| <= 0.01; max y == height (1e-6); min y in [0, 1e-4]
TEST(Mannequin, FacesPlusZ)               // landmarks chest.z > regions[Torso].origin.z + 0.05
TEST(Mannequin, MirrorSymmetric)          // each vertex's (-x, y, z) has a mesh vertex within 0.002 (voxel hash, 1 cm cells)
TEST(Mannequin, PoseAngles)               // frontal angle(wrist - shoulder_joint) 45 +- 2; (ankle - hip_joint) 5 +- 1; both sides
TEST(Mannequin, EggHead)                  // every head vertex within 0.001 of the egg (distance to its radial projection);
                                          // every triangle with 3 head vertices has normal . (centroid - centre) > 0
TEST(Mannequin, SmoothFeet)               // section at y = 0.005: segments with x > 0 form 1 connected loop; x < 0 also 1
TEST(Mannequin, RegionsContainTheirParts) // each region's checks vertices lie within radius + 1e-6 of its axis
TEST(Mannequin, GirthsInAdultRanges)      // male chest in [0.90, 1.10]; female chest in [0.80, 1.00]
TEST(Mannequin, CollisionFieldInsideOutside) // bake; sample(chest - (0,0,0.05)) < 0; sample(chest + (0,0,0.10)) > 0
TEST(MannequinLoader, MissingDirectoryNamesThePath)   // Review Focus 5: loadMannequin(Male, "/no/such/dir") message
                                                      // contains "/no/such/dir/male.obj"
TEST(MannequinLoader, NonTriangleFaceNamesTheLine)    // Review Focus 4: temp OBJ with "f 1 2 3 4" on line 5 -> ":5:"
TEST(MannequinLoader, IndexOutOfRangeNamesTheLine)    // Review Focus 4: "f 1 2 99" -> "vertex index out of range"
TEST(MannequinLoader, UnsupportedFormatIsRejected)    // Review Focus 2: copied assets with "format": 2 -> "unsupported format 2"
TEST(MannequinLoader, MissingRegionIsRejected)        // Review Focus 3: JSON without "Waist" -> "missing region 'Waist'"
```

  In `test_collision_perf.cpp` add `TEST(CollisionField, BakeMannequinUnderTwoSeconds)`: bake `loadMannequin(BodyType::Male).mesh` in under 2.0 s; the torso origin minus (0, 0.3, 0) samples negative.

- [ ] **Step 2: Run to verify it fails.** `cmake --build --preset dev`. Expected: build fails (`mannequin.hpp` missing).
- [ ] **Step 3: Implement** `obj_reader.cpp`, `mannequin.cpp`, and the CMake changes: new sources, add `tests/test_mannequin.cpp` to `drape_avatar_tests`, and the `DRAPE_ASSET_DIR` definition.
- [ ] **Step 4: Run to verify it passes.**

Run: `cmake --build --preset dev && ctest --preset dev -R "Mannequin" --output-on-failure && cmake --build --preset release && ctest --preset release -L perf`
Expected: all PASS, including `BakeMannequinUnderTwoSeconds` in release.

- [ ] **Step 5: Commit** (`feat(avatar): mannequin loader with mesh, pose, egg and region checks`).

---

### Task 7: Drape the test tee on the mannequins

**Files:**
- Modify: `libs/sim/tests/tee_drape_helpers.hpp`, `libs/sim/tests/test_tee_drape.cpp`, `libs/sim/tests/test_tee_drape_slow.cpp`, `tools/drape_dump/main.cpp`, `tools/drape_dump/CMakeLists.txt`, `tools/render_obj.py`, `README.md` (tool usage)

**Interfaces:**
- Consumes: `loadMannequin`, `Mannequin`, `BodyType` (Task 6); the existing Task 16 pipeline (`makeSimGarment`, `drapeToRest`, `placePattern`).
- Produces:
  - `void expectTeeDrapes(const std::string& size, SimQuality q, avatar::BodyType body = avatar::BodyType::Male)`. It keeps a static per-body cache of {Mannequin, baked CollisionField}.
  - The hem check becomes |mean hem y − (½·(hps_left.y + hps_right.y) − L)| ≤ 0.03, where L = `garment::resolveSpec(TestTeeGenerator().specTable(), size).at("body_length")`. The settled, failed, gap (< 1 mm), penetration (≤ 2 mm) and time (≤ 15 s) checks are unchanged.
  - `drape_dump [--body male|female]` (default male). It writes objects `body` and `tee`; the usage line becomes `usage: drape_dump --out <file.obj> [--size XS|S|M|L|XL|XXL] [--quality draft|standard|fine] [--body male|female]`.

- [ ] **Step 1: Write the failing tests** (rename and add):

```cpp
TEST(TeeDrape, DraftOnMaleMannequin) { expectTeeDrapes("M", SimQuality::Draft, avatar::BodyType::Male); }
TEST(TeeDrapeSlow, AllSizesDraftMale)        // every garment::kStandardSizes, Male
TEST(TeeDrapeSlow, AllSizesDraftFemale)      // every size, Female
TEST(TeeDrapeSlow, StandardOnMaleMannequin)  // "M", Standard, Male
```

  In `tools/drape_dump/CMakeLists.txt`: add `add_test(NAME DrapeDumpFemaleSmoke COMMAND drape_dump --out ${CMAKE_BINARY_DIR}/smoke_female.obj --body female)` with `LABELS slow`, and change `check_usage.cmake` to match the new usage line.

- [ ] **Step 2: Run to verify it fails.** `cmake --build --preset dev`. Expected: build fails (`expectTeeDrapes` has no `body` parameter).
- [ ] **Step 3: Implement.**
  - Update the helper (mannequin + bake cache, landmark-relative hem) and `drape_dump` (`--body`, objects `body` and `tee`).
  - In `render_obj.py`, set `BODY_RGB = (0.93, 0.93, 0.91)`.
  - README: document `--body`.
- [ ] **Step 4: Run to verify it passes.**

Run: `ctest --preset dev -R "TeeDrape|DrapeDump" --output-on-failure`
Expected: all PASS. Each test prints one summary line; every size settles within 15 s.
If a size fails, use superpowers:systematic-debugging before changing any threshold. The thresholds come from the spec.

- [ ] **Step 5: Commit** (`feat(sim): drape the test tee on the male and female mannequins`).

---

### Task 8: Previews, spec amendments and hand-off

**Files:**
- Modify: `docs/superpowers/specs/2026-10-01-drape-milestone-1-design.md` (8.1, 12.5, 17, Status line), `README.md` (mannequin builder section)

**Interfaces:** none.

- [ ] **Step 1: Previews.** Run: `for b in male female; do ./build/dev/tools/drape_dump/drape_dump --out out/tee_M_$b.obj --body $b && python3 tools/render_obj.py out/tee_M_$b.obj out/tee_M_$b.png; done`
Expected: both summaries show `settled=1 failed=0`. Open both PNGs with the Read tool and confirm:
  - egg head with no ears or nose;
  - smooth feet;
  - arms at about 45°;
  - sleeves around the arms and the rib around the neck;
  - no spikes and no body poking through the tee.

  Send both PNGs to Ian.
- [ ] **Step 2: Spec amendments.** Apply Section 10 of the A2 spec to the milestone spec verbatim in meaning:
  - 8.1 Look and Pose;
  - 12.5 adds the mannequin checks;
  - 17 adds row A2 before B and trims row C;
  - the Status line adds "Phase A2 mannequins (Sections 8.1, 12.5, 17)".
- [ ] **Step 3: README.** Add a "Mannequins" section: what the assets are, the CC0 source, and that `python3 tools/mannequin/build_mannequins.py` regenerates them (network needed once, numpy required).
- [ ] **Step 4: Full verification.**

Run: `ctest --preset dev -LE perf && cmake --build --preset release && ctest --preset release -L perf`
Expected: every test PASSes, including `slow` and `MannequinBuilderUnit` in dev and `perf` in release.

- [ ] **Step 5: Commit** (`docs: Phase A2 mannequins in the milestone spec; README`).
- [ ] **Step 6: Hand-off.**
  1. `git bundle create /mnt/user-data/outputs/drape.bundle --all`.
  2. Commit the bundle into the connected Dev 2 folder.
  3. In `Dev 2/drape`, run `git fetch -q ../drape.bundle main && git merge --ff-only FETCH_HEAD`.
  4. Confirm that `git log --oneline -1` matches.
  5. Tell Ian to run `git push` from `Dev 2/drape` to update `github.com/cache25/drape-2`.
