# Drape — Mannequins (Phase A2) Design

**Date:** 2026-10-04
**Status:** Draft for review
**Amends:** `2026-10-01-drape-milestone-1-design.md` Sections 8.1, 12.5 and 17 (amendments listed in Section 10)
**Reference:** Ian's photos of a white full-body male display mannequin: egg-shaped featureless head, toned athletic torso (pecs, abs), arms hanging slightly away from the body, legs slightly apart.

## 1. Goal

Replace Phase A's primitive test body (capsules and a sphere, no legs) with realistic mannequins in the style of the reference photos, before the Mac app (Phase B) puts a body on screen. From then on, every garment-facing use — drape tests, `drape_dump`, the Phase B viewport — uses the mannequins.

Sizing an avatar to measurements (solver, size charts, Avatar panel) stays in Phase C.

## 2. Decisions

| Question | Decision |
|---|---|
| When | Now, as Phase A2, before Phase B |
| Pose | 45° drape pose (industry standard), not the photos' ~20° arms |
| Bodies | Male and female, same style |
| Source | MakeHuman's CC0 body, restyled (not a generated blob, not a purchased model) |

## 3. Look

- **Head:** a smooth egg — no face, ears or hair. The neck blends into it.
- **Build:** toned and athletic, from MakeHuman's muscle macro, so chest, abs and shoulders show definition.
  - Male: height 1.80 m, chest about 98 cm.
  - Female: height 1.70 m, bust about 88 cm.
  - Heights are exact (±1 cm). Girths are whatever the chosen build gives; they are measured and recorded, not solved (Phase C).
- **Hands:** separate fingers, as on the photos.
- **Feet:** one smooth mannequin foot, no separate toes.
- **Pose:** each arm 45° from the body in the frontal plane; each leg 5° out from vertical; soles flat on the floor.
- **Colour:** white, soft satin finish (renderer, Phase B). Developer previews draw it off-white.
- **Out of scope:** stand or base plate, other poses, sizing to measurements.

## 4. Source assets

From `github.com/makehumancommunity/makehuman`, pinned at commit `a8bc2d54ff0ac92e78ff71431b1023eda42bf482`. Assets are CC0 (`LICENSE.ASSETS.md`): commercial use allowed, no attribution required.

| File | Use |
|---|---|
| `makehuman/data/3dobjs/base.obj` | Base mesh. Units are decimetres. Group `body` is 13,380 vertices / 13,378 quads, closed and manifold (checked). Groups `joint-*` are the helper geometry the skeleton's joint positions are computed from. Other helper groups are dropped |
| `makehuman/data/targets/macrodetails/**` | Macro targets: gender × age × muscle × weight (`universal-*`), height, proportions |
| `makehuman/data/modifiers/modeling_modifiers.json`, `makehuman/apps/humanmodifier.py` | The macro-modifier definitions and weighting rules the builder reproduces |
| `makehuman/data/rigs/default.mhskel`, `default_weights.mhw` | Skeleton (joint positions from `joint-*` groups) and skinning weights |

## 5. Mannequin builder (`tools/mannequin/build_mannequins.py`)

A developer-run Python 3 script (numpy only). It fetches exactly the files in Section 4 at the pinned commit into a cache folder, builds both bodies, and writes the outputs in Section 6. Outputs are committed, so builds and tests never need the network. Running it twice gives byte-identical outputs.

Steps, per body:

1. **Build.** Apply the macro targets with MakeHuman's weighting rules at: gender 1.0 (male) or 0.0 (female); age 0.5 (25 years); muscle 0.75; weight 0.5; proportions 1.0; race 1/3 each (MakeHuman's default). Height macro: found by bisection so the posed body's height (floor to top of the egg head) is 1.80 m (male) or 1.70 m (female) ± 1 mm.
2. **Egg head.** The head region is every vertex whose skinning weight on the `head` bone and its descendants (jaw, eyes, ears, tongue) is w > 0. The egg is defined in the head bone's frame:
   - centre at the head joint; semi-axes from the skull itself, with ears excluded: width and depth measured at eye level, height from the chin to the top of the head;
   - the lower half tapers to 0.85 of the upper half's horizontal radii, which gives the egg shape and a soft chin.

   Each head vertex moves to x + w·(P(x) − x), where P projects radially from the egg centre onto the egg. Then 30 rounds of Laplacian relaxation within the region, each followed by re-projection of w = 1 vertices. Any move that would flip a triangle is rejected. Nose, ears and eye sockets melt away; the neck blends through 0 < w < 1.
3. **Feet.** The toe region is every vertex with weight w > 0 on the toe bones. 40 rounds of Laplacian smoothing, scaled by w, with the sole plane held: vertices on the sole keep their height. Toes merge into one foot.
4. **Pose.** Linear blend skinning with the default skeleton and weights:
   - upper arms rotate in the frontal plane until the shoulder→wrist axis is 45° from vertical;
   - upper legs rotate outward until hip→ankle is 5° from vertical;
   - ankles counter-rotate so each sole is level.
5. **Frame.** Metres, Y up, the lowest vertex at y = 0, facing +Z, x = 0 on the symmetry plane, z = 0 midway between the ankle joints. Quads split along the shorter diagonal into triangles. Vertex order stays MakeHuman's, so Phase C's targets still map one-to-one.
6. **Regions and landmarks** (Section 6) computed from the posed skeleton and mesh.

## 6. Outputs (`assets/mannequins/`, committed)

| File | Contents |
|---|---|
| `male.obj`, `female.obj` | One object, triangles only, metres. About 13,400 vertices and 26,800 triangles |
| `male.json`, `female.json` | `height`, measured girths (chest/bust, waist, hip, neck as convex-hull perimeters, spec 8.2), landmarks, region cylinders |
| `LICENSE.md` | CC0 notice for the MakeHuman assets, the pinned commit, and the builder version |

Landmarks: `hps_left`, `hps_right` (high point of shoulder), `shoulder_left`, `shoulder_right`, `neck_base`, `chest`, `waist`, `hip`, `crotch`, `ankle_left`, `ankle_right` (positions in metres).

Region cylinders: one per `BodyRegion` (`Torso`, `Neck`, `Head`, `ArmLeft`, `ArmRight`, `LegLeft`, `LegRight`, `Waist`). They use the existing `RegionCylinder` fields and the conventions in `body_regions.hpp`: Torso and Neck anchored on the body axis at neck-base height; arms at the shoulder joints along the posed arm axis; legs at the hip joints along the posed leg axis; Waist on the body axis at waist height; Head at the head joint. Each radius is the largest distance from the axis to that part's vertices.

## 7. Loader (`drape_avatar`)

```cpp
enum class BodyType { Male, Female };
struct Landmarks { std::map<std::string, Vec3d> points; };
struct Mannequin {
  TriangleMesh mesh;      // one closed component
  BodyRegions regions;
  Landmarks landmarks;
  double height = 0;
};
Mannequin loadMannequin(BodyType type);                                  // from DRAPE_ASSET_DIR
Mannequin loadMannequin(BodyType type, const std::filesystem::path& assetDir);
```

- `DRAPE_ASSET_DIR` is a compile definition pointing at the source tree's `assets/`. The app bundle in Phase F passes its own path.
- A missing or malformed file throws `std::runtime_error` naming the file and line.
- The collision bake, placement and solver are unchanged: a mannequin is one component in `bakeCollisionField`.
- `makeTestBody()` stays for fast unit tests that want simple geometry, such as the primitive and placement unit tests.

## 8. What switches to the mannequins

- `TeeDrape*` tests and their helper: male by default, female where stated.
- `drape_dump`: new `--body male|female` option (default male). It writes objects `body` and `tee` instead of `body_0`…`body_4`.
- `tools/render_obj.py`: the body is drawn off-white.
- `CollisionPerf.BakeUnderTwoSeconds`: bakes the male mannequin (release build, < 2 s, spec 8.3).
- The tee hem check moves from fixed heights to a landmark-relative one (Section 9).

## 9. Tests

**Mannequin (each body):**
- Mesh closed and manifold; no triangle with area < 1e-10 m².
- Height within 1 cm of target; lowest vertex at y = 0 ± 0.1 mm; the chest landmark's z exceeds the back's (faces +Z).
- Left–right mirror symmetric within 2 mm.
- Arm axis 45° ± 2° from vertical; leg axis 5° ± 1°.
- Every head-region vertex with w = 1 lies within 1 mm of the egg; no head-region triangle normal points into the egg.
- Feet: a horizontal cross-section 1 cm above the floor through the toes is a single loop per foot.
- Every region cylinder contains its part's vertices.
- Collision field: points 5 cm inside the torso read negative, points 10 cm in front of the chest read positive.
- Loader: a missing file and a truncated file each throw with the file name.

**Garment:**
- Male: the test tee drapes and settles at every size (XS–XXL) at Draft. Settled within 15 s simulated, assembly gap < 1 mm, penetration ≤ 2 mm, mean hem height within 3 cm of (HPS height − body length).
- Female: the same, at every size.
- Male at Standard, size M.
- Fast suite: male M at Draft. The other sizes and the female runs are labelled `slow`.

**Visual:** front and side previews of both mannequins wearing the size M tee, checked by eye (egg head, smooth feet, sleeves around the arms, no spikes) and sent to Ian.

## 10. Amendments to the milestone spec

- **8.1 Body model.** "Look" becomes: white with a soft satin finish; egg-shaped featureless head; smooth mannequin feet; separate fingers; toned build from the muscle macro. "Pose" adds legs 5° apart with level soles. Phase A2 builds the bodies as in this document; Phase C adds sizing on top of them.
- **12.5 Avatar tests.** Add the mannequin checks in Section 9.
- **17 Phases.** New row before B: *A2. Mannequins: male and female mannequins (MakeHuman CC0, egg head, 45° pose), builder script, loader, drape tests on both bodies. Runs on Linux.* Row C becomes: *Measurement solver, built-in and custom size charts, Avatar panel (on top of the A2 mannequins).*

## 11. Risks

| Risk | Mitigation |
|---|---|
| Shoulder creasing when the arms are posed | MakeHuman's rest arms are already about 40° down, so the rotation is small. The symmetry and normal checks catch broken skinning |
| The egg projection folds triangles at the ears | Relaxation with flip rejection; the head-normal test |
| A men's tee on the female body fails to settle at small sizes | Test every size; if one fails, debug it before relaxing anything |
| A heavier mesh slows the collision bake past 2 s | The perf test catches it; the bake is per-triangle, about 27k triangles |
| MakeHuman's macro weighting is reproduced wrongly | Unit-test the weighting against hand-computed weights at the corner settings (each macro at 0, 0.5 and 1); weights within each macro group sum to 1; the built bodies' recorded girths fall in normal adult ranges (male chest 90–110 cm, female bust 80–100 cm) |
