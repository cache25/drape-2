# Drape — Milestone 1 Design Spec

**Status:** Approved 2026-10-01; amended during Phase A planning (bending constants, seam assembly, dependency versions) and Phase A execution (solver method XPBD → VBD, Section 5.1)
**Date:** 2026-10-01
**Owner:** Ian Schory
**Scope:** Sub-project #1 of the Drape roadmap: pre-made garments draped on a sized avatar, with real fabric physics and high-quality real-time rendering, for Apple Silicon Macs.

---

## 1. Product context

### 1.1 What Drape is

Drape is commercial desktop software for 3D fashion design and garment simulation, in the same category as CLO and Marvelous Designer. The long-term product lets designers draft 2D sewing patterns, simulate them as realistic 3D garments on digital avatars, prepare them for production (grading, nesting, POMs, tech packs), and export them to Blender, Maya and Unreal Engine.

### 1.2 First customer

**Indie and emerging apparel brands**, starting with streetwear and DTC labels. They are priced out of or underserved by CLO, which costs $50/month or $450/year for a single individual license. They need to see a garment on a body, dial in fit, and later hand a factory-ready package to a manufacturer.

### 1.3 Who builds it

Ian (product, design direction, QA) working with Claude (most engineering). Scope decisions in this spec are sized for that team. The architecture follows industry conventions so engineers can be hired into it later.

### 1.4 Decisions log

| # | Decision | Choice | Why |
|---|---|---|---|
| D1 | Purpose | Commercial product | Stated by Ian |
| D2 | First customer | Indie / emerging brands | Underserved by CLO on price and complexity; closest to Ian's own brand work |
| D3 | First milestone | Pre-made garments (library → resize → fabric/color → render) | Stated by Ian; fastest path to a convincing product |
| D4 | Garment representation | Garments are sewing patterns (2D pieces + seams), draped by physics — never frozen 3D meshes | Makes resizing real, and the future pattern editor edits the same data |
| D5 | Delivery | Installed desktop software | Stated by Ian |
| D6 | Language / UI | C++20 with Qt 6 | Industry standard (CLO, Marvelous, Maya, Houdini, Blender); every required export library (FBX, Alembic, OpenUSD, MaterialX) and the Cycles renderer are first-class C++ |
| D7 | Platform | Apple Silicon Macs first (macOS 14+, arm64 only); Windows is a later sub-project | Stated by Ian |
| D8 | GPU API | Metal via Apple's metal-cpp | Native Mac GPU API; physics sits behind a backend interface so a Windows backend can be added later |
| D9 | Rendering | High-quality real-time (Filament). Path-traced photoreal rendering is a later sub-project | Stated by Ian |
| D10 | Garment set | Long sleeve thermal, crewneck, pullover hoodie, puffer jacket, joggers, shorts, baggy jeans, cargo pants, beanie | Stated by Ian |
| D11 | Delivery order | Two waves inside milestone 1 (Section 2.3) | Proves the core engine on simpler garments before the hardest ones |
| D12 | Editable spec table | Included | Approved by Ian; cheap with generator architecture, high value for fit work |

---

## 2. Milestone 1 scope

### 2.1 User-facing capabilities

A designer using Drape v0.1 can:

1. **Set up a body.** Choose a female or male base avatar. Enter measurements (height, bust/chest, underbust, waist, hip, shoulder width, arm length, inseam, neck, head circumference) or pick a size from a size chart. The avatar reshapes to match. It appears as a neutral matte mannequin.
2. **Use size charts.** Built-in generic women's and men's charts (US letter sizes XS–XXL, EU numeric sizes), plus custom brand charts entered as a table and saved with the project.
3. **Pick garments from the library.** Nine garments (Section 7), browsed by category with thumbnails.
4. **Choose a garment size (XS–XXL) and watch it drape.** Flat pieces start wrapped around the body, seams pull together, and the garment settles.
5. **Edit the garment spec table.** View and edit any garment measurement for any size (for example, body length in L). The pattern regenerates and re-drapes.
6. **Swap fabrics.** Choose from 13 presets (Section 6). The choice changes both drape and look. Every physical and visual property is adjustable, and custom fabrics can be saved.
7. **Recolor.** Set any color per garment or per piece. Textures are tinted, not replaced.
8. **Build an outfit.** Wear at most one top, one bottom and one headwear item at once. Outerwear (hoodie, crewneck, puffer) uses the top slot, so the puffer is worn without another top underneath. Where a top and a bottom overlap at the waist, the top always lies over the bottom.
9. **Interact.** Orbit, pan and zoom the camera. Grab and drag cloth with the mouse while simulating.
10. **Read fit.** A fit-map overlay with two modes: **Strain** (fabric stretch relative to the flat pattern) and **Pressure** (garment pressure on the body), with a color legend.
11. **Present.** Three lighting presets. PNG capture up to 4096 px on the long side, with an optional transparent background.
12. **Save and open.** `.drape` project files, autosave every 2 minutes, crash recovery on next launch.
13. **Units.** Centimeters or inches throughout the UI.

### 2.2 Explicitly out of scope for milestone 1

- Drawing or editing pattern shapes (pattern editor sub-project). The spec table changes measurements, not shapes.
- Stacking whole garments on top of each other (for example, the puffer over the thermal), and working zippers, buttons or drawcords. Zippers and flies are sewn closed and rendered as trims.
- Grading rules editor, POMs, tech packs, nesting/print layout, DXF export.
- FBX / Alembic / USD / glTF / OBJ export.
- Path-traced rendering, avatar poses and animation.
- Measured fabric import (U3M, AxF, lab test entry).
- Windows, accounts, licensing, billing, cloud features, auto-update.

### 2.3 Delivery waves

**Wave 1 — the core engine on knits:** long sleeve thermal, crewneck, pullover hoodie (with hood and kangaroo pocket), joggers, shorts, beanie. Also a plain tee used only as an internal test garment, not shipped in the library.

Wave 1 requires: stretch, bending, seams, elastic, fold lines, layer order, body collision, self collision, the avatar system, the fabric system, the full app, rendering, the fit map, files, and the spec table.

**Wave 2 — the hard garments:** baggy jeans, cargo pants, puffer jacket.

Wave 2 adds: pressure (puffer chambers), rigid trims (rivets, buttons), closed zippers and flies, topstitch rendering, many-piece garments with flaps and bellows pockets, and rigid denim tuning.

Wave 2 starts only after every Wave 1 garment passes the drape matrix (Section 12.3).

### 2.4 Roadmap after milestone 1

Each item below is its own sub-project with its own spec, plan and build, in this order:

1. **Pattern editor:** 2D drafting and editing of pieces, seams and internal lines.
2. **Layering and trims:** full multi-garment layering with continuous collision, working zippers, buttons, drawcords, fusing.
3. **Production tools:** grading editor, POMs, tech pack PDF, nesting/print layout with fabric consumption, DXF-AAMA/ASTM export.
4. **Ecosystem export:** FBX, Alembic (animated cloth caches), OpenUSD, glTF, OBJ.
5. **Photoreal rendering:** embedded Cycles; avatar poses and animation.
6. **Fabric digitization:** U3M and AxF import plus manual lab-test entry. CLO's Fabric Kit writes CLO's proprietary format, so Drape targets the open formats.
7. **Windows port:** a new GPU backend for the physics and the renderer.
8. **Accounts, licensing and billing.**

---

## 3. Architecture

### 3.1 Modules

Six CMake libraries. Each has one job and a public header interface; internals can change without touching consumers.

| Module | Responsibility | Depends on | Builds on Linux? |
|---|---|---|---|
| `drape_core` | Data model, geometry math, units, `.drape` file I/O | Eigen, nlohmann/json, miniz | Yes |
| `drape_garment` | Parametric pattern generators, spec tables, pattern validation, pattern → sim mesh (the `SimMesh` type itself lives in core, so `drape_sim` needs only core), initial placement | core, CDT | Yes |
| `drape_avatar` | Base body, shape targets, A-pose, measurement solver, collision field bake | core | Yes |
| `drape_sim` | Cloth solver: `Solver` interface, `CpuSolver` (reference) and `MetalSolver` (production) | core (+ metal-cpp for the Metal backend) | CPU backend: yes. Metal backend: Mac only |
| `drape_render` | Filament viewport, materials, lighting, fit-map overlay, capture | core, Filament | Mac only |
| `drape_app` | Qt 6 application: windows, panels, commands, undo/redo, autosave | all modules, Qt 6 | Mac only |

The rule: everything that can be plain portable C++ stays out of the Mac-only modules. That keeps most logic testable in a Linux workspace and CI.

### 3.2 Data flow

1. A user action in the Qt UI creates a **command** (a `QUndoCommand`).
2. The command mutates the **Project** model in `drape_core`.
3. The **SimSession** receives a change notification and rebuilds only what changed:
   - Garment size or spec change → regenerate pattern → rebuild that garment's sim mesh → re-drape that garment.
   - Fabric change → update that garment's constraint parameters in place, without rebuilding the mesh.
   - Color change → render only.
   - Avatar change → re-bake the collision field → re-drape all garments.
4. The solver runs on a **dedicated simulation thread**. After each frame it publishes particle positions to a double-buffered snapshot.
5. The render loop on the UI thread reads the latest snapshot every frame. The UI never waits on physics.

### 3.3 Coordinate system and units

- Internal units are SI: meters, kilograms, seconds. Fabric properties use physical units (Section 6.1).
- 3D space: right-handed, **Y up**, avatar feet at y = 0, avatar facing +Z (matches glTF conventions for future export).
- Pattern space: 2D, meters. Each piece has a grainline vector defining the warp direction.
- The UI displays centimeters or inches.

### 3.4 Third-party components

All are cleared for closed-source commercial use.

| Component | Use | License |
|---|---|---|
| Qt 6 (latest LTS) | UI framework | LGPL v3 (dynamically linked); commercial license later |
| Filament (pinned release) | Real-time physically based renderer with a cloth shading model, Metal backend | Apache 2.0 |
| metal-cpp | Apple's official C++ bindings for Metal | Apache 2.0 |
| MakeHuman base mesh, shape targets, skeleton | Avatar | CC0 |
| CDT (artem-ogre) | Constrained Delaunay triangulation of pattern pieces | MPL 2.0 |
| Eigen 3.4 (system package: apt `libeigen3-dev`, Homebrew `eigen`) | Linear algebra | MPL 2.0 |
| nlohmann/json | JSON | MIT |
| miniz | ZIP read/write for `.drape` | MIT |
| GoogleTest | Tests | BSD-3 |
| ambientCG / Poly Haven textures and HDRIs | Fabric textures, lighting environments | CC0 |

Explicitly avoided: SMPL body model (commercial use requires a paid license) and Shewchuk's Triangle (non-commercial license).

The physics engine is written in-house from published methods. It is the product's core asset.

---

## 4. Data model (`drape_core`)

| Type | Fields |
|---|---|
| `Project` | metadata, display units, `AvatarParams`, size charts, garment instances, custom fabrics, scene (lighting preset, camera) |
| `AvatarParams` | base body (female / male), target measurements, optional chart reference (chart id + size), solved shape weights (cache) |
| `SizeChart` | id, name, base body, ordered size labels, body measurement per size |
| `GarmentInstance` | id, generator id, generator version, outfit slot (top / bottom / headwear), selected size, spec overrides, stored `Pattern`, fabric assignment per fabric slot, color overrides (garment-level and piece-level), sim quality |
| `Pattern` | pieces, seams, internal lines, trims |
| `PatternPiece` | id, name, outline (closed sequence of cubic Bézier segments), grainline vector, fabric slot, placement (body region, offset), optional layer relation (base piece id, order ±1), optional elastic contraction (factor, direction), mirror flag |
| `Seam` | id, edge A (piece, start/end parameter, direction), edge B (same), declared ease (length difference allowed), kind (`normal`, `closed_zipper`, `closed_fly`) |
| `InternalLine` | piece, polyline, kind: `fold` (target angle, stiffness multiplier), `quilting`, `topstitch` (offset, stitches per inch, thread color, thread thickness) |
| `Trim` | kind (`rivet`, `button`, `zipper_strip`), attachment (piece + 2D position, or seam id), size, material |
| `Fabric` | id, name, physical properties, visual properties (Section 6) |

The generated `Pattern` is **stored in the project**. Reopening a project uses the stored pattern, so app updates that improve a generator never silently change saved work. "Update to latest pattern" is an explicit user action. Storing the pattern also prepares for the pattern editor, which will edit it directly.

---

## 5. Physics engine (`drape_sim`)

### 5.1 Method

**Vertex Block Descent (VBD)** (Chen, Macklin et al., "Vertex Block Descent", ACM TOG 2024). Each simulation frame advances 1/60 s of simulated time, split into S substeps. Each substep minimizes the implicit-Euler energy, inertia plus every constraint energy, with I iterations of per-vertex Newton steps: one 3×3 solve per vertex using the Gauss–Newton Hessian of its incident energies. Vertices are processed color by color, so each color can run in parallel on the GPU. Every constraint in Section 5.4 is an energy ½·K·C², with stiffness K = 1/α taken from the compliance formulas there.

*Why not XPBD (the original choice):* with one iteration per substep, XPBD's resting state carries a stretch error of about (c·h/ℓ)² times the true strain, where c is the fabric's stretch-wave speed, h the substep and ℓ the edge length. At our quality settings that makes every fabric sag about 5% under its own weight, whatever its stiffness. Phase A's stretch test measured 137% strain where 2.5% is correct. VBD's resting state is the true static equilibrium. The same test lands within 0.4%.

| Quality | Particle distance | Default substeps × iterations per frame |
|---|---|---|
| Draft | 20 mm | 2 × 5 |
| Standard | 10 mm | 2 × 10 |
| Fine | 5 mm | 4 × 10 |

These defaults may be retuned during implementation. The physics tests in Section 12.2 are the authority on whether a value is acceptable.

### 5.2 Solver interface

```cpp
class Solver {
public:
  virtual ~Solver() = default;
  virtual void build(const SimScene& scene) = 0;                  // particles, constraints, collision field
  virtual void updateFabric(GarmentId g, const FabricPhysical& f) = 0;
  virtual void setPhase(GarmentId g, SimPhase phase) = 0;         // Assembling / Settling
  virtual void step(const StepInput& in) = 0;                     // one frame (all substeps)
  virtual void readPositions(std::span<Vec3f> out) const = 0;
  virtual SimStats stats() const = 0;                             // NaN flag, max speed, kinetic energy, seam gap
  virtual Snapshot snapshot() const = 0;
  virtual void restore(const Snapshot& s) = 0;
};
```

`CpuSolver` and `MetalSolver` implement the same interface and the same algorithm. Vertices are partitioned by **graph coloring**, so no two vertices that share an energy term have the same color. Each color is solved in parallel on the GPU, and the CPU solver processes the colors in the same order. Body collision and pins are applied as position projections after each substep's iterations.

### 5.3 Sim mesh from a pattern (`drape_garment`)

1. Resample each outline at the particle distance *h*.
2. For each seam, sample both edges with the **same count** N = ⌈max(lenA, lenB) / h⌉, giving 1:1 vertex pairs.
3. Fill the interior with a **hexagonal point lattice aligned to the grainline** at spacing *h*. Drop points closer than 0.5*h* to the boundary or to internal lines. This yields near-equilateral triangles with edges along warp and weft.
4. Insert internal lines (folds, quilting) as constrained edges resampled at *h*.
5. Triangulate with constrained Delaunay (CDT). Each vertex keeps its 2D rest position in pattern space.
6. Particle mass = fabric weight × ⅓ of the summed rest area of its incident triangles.

### 5.4 Constraints

**Stretch (anisotropic, per triangle).** Compute the deformation gradient F from the triangle's current 3D edges and its 2D rest edges, with rest axes rotated so *u* = warp (grainline) and *v* = weft. Let f_u = F·u and f_v = F·v.
- Warp: C = |f_u| − 1
- Weft: C = |f_v| − 1
- Bias (shear): C = (f_u · f_v) / (|f_u| |f_v|)
- Compliance α = 1 / (k · A), where k is the fabric stiffness for that direction (N/m) and A is the rest area. The VBD stiffness is K = 1/α = k·A.
- **Strain limiting:** a one-sided stiff energy (100 × the stretch stiffness) caps warp and weft stretch at the fabric's strain limit, so rigid wovens like denim never look rubbery.

**Bending (per interior edge).** Dihedral-angle constraint C = θ − θ₀, where θ₀ = 0 for flat fabric and θ₀ = the fold angle on fold lines.
- Direction-dependent stiffness: for an edge at angle φ from the warp in pattern space, B(φ) = p · sin²φ + q · cos²φ, with p = (11·B_warp − 3·B_weft) / 8 and q = (9·B_weft − B_warp) / 8, each clamped to at least 0.05 · min(B_warp, B_weft). B_warp is the resistance to bending the warp yarns, which happens when the fold runs across them. On the grain-aligned hexagonal lattice (Section 5.3), these coefficients make the effective bending rigidity along the warp exactly B_warp and along the weft exactly B_weft. The naive law B_warp · sin²φ + B_weft · cos²φ would blend them to 0.75·B_warp + 0.25·B_weft.
- Compliance from bending rigidity B (N·m): α = (A₁ + A₂) / (2 · B · |e|²), where A₁ and A₂ are the rest areas of the two adjacent triangles and |e| is the rest edge length. The constant 2 makes the discrete hinge energy on the hexagonal lattice equal the continuum bending energy ½·B·κ² for cylindrical bending. The cantilever test (Section 12.2) verifies it.

**Seams.** One stiff one-sided spring per seam vertex pair (energy ½·K_seam·(|x_a − x_b| − L)² when the gap exceeds L; K_seam defaults to 10⁴ N/m), plus a dihedral bending constraint across the seam (stiffness = mean of the two fabrics' bending rigidity) so seams don't act as free hinges.

**Elastic.** Elastic pieces (waistbands, cuffs, beanie rib) carry a contraction factor s ∈ [0.5, 1] along a direction (default: the band's length). Their 2D rest shape is scaled by s along that direction before the constraints are built.

**Fold lines.** Bending constraints on fold-line edges use the line's target angle (180° for a fully folded hem or beanie cuff) and a stiffness multiplier.

**Layer order.** For a piece with layer relation (base, order = +1), every particle stays on the outward side of the base piece's surface:
- One-sided constraint: (x − p) · n ≥ d, where p is the closest point on the base surface, n is its normal, and d = (thickness_upper + thickness_base) / 2.
- order = −1 flips the side. Linings use it.
- Applies to pockets, flaps, linings, puffer shell vs. lining, and the beanie cuff flap.
- **Outfit layering:** every piece of the top garment has an implicit order = +1 relation to the bottom garment's surface wherever the two overlap (the waist region). Headwear has none. Full stacking of garments of the same slot stays out of scope (Section 2.2).

**Pressure (Wave 2).** Each puffer chamber is the closed region between shell and lining bounded by quilting lines and seams. Quilting lines are zero-length constraints joining corresponding shell and lining vertices. Each chamber has a volume constraint C = V − V_target, where:
- V is computed with the divergence theorem over the chamber's shell and lining triangles.
- V_target = chamber rest area × the garment's **loft** parameter (mm).

**Trims (Wave 2).**
- **Rivets and buttons** attach to a triangle by barycentric anchor. They add their mass to that triangle's particles and render following the triangle's local frame. They are not independent rigid bodies in milestone 1.
- **Closed zippers** are `closed_zipper` seams: zero-length seam pairs, plus bending stiffness raised along the seam line by a zipper-tape stiffness parameter, plus rendered zipper geometry (tape and teeth) following the seam polyline.
- **Closed flies** are `closed_fly` seams with a topstitched J-curve and a button trim.

### 5.5 Collisions

**Body.** A signed distance field (SDF) is baked from the avatar on a regular grid at ≤ 4 mm resolution, padded 5 cm around the body, and sampled trilinearly.
- Constraint: φ(x) ≥ r, where r = collision offset (default 3 mm) + thickness / 2.
- Positional friction uses the fabric's friction coefficient.
- Contact impulses are accumulated per particle to drive the Pressure fit map.

**Self (cloth vs. cloth, including across garments in an outfit).** Discrete vertex–triangle proximity with thickness d = max(fabric thickness, 1.5 mm).
- Candidate pairs are gathered once per frame with a GPU spatial hash, using bounds inflated by the frame's maximum travel. They are resolved every substep.
- Excluded pairs: a vertex and the triangles of its own 2-ring, and any vertex–triangle pair on the same piece whose pattern-space rest distance is under 2d.
- **Tunneling guard:** per-substep displacement is clamped to 0.5d.
- Edge–edge tests and continuous collision are deferred to the layering sub-project.

### 5.6 Forces and damping

- Gravity 9.81 m/s².
- Linear velocity damping from the fabric's damping value.
- Air drag proportional to triangle area and normal velocity.

### 5.7 Simulation phases

| Phase | Behavior | Ends when |
|---|---|---|
| Assembling | Gravity off, extra damping. Each seam pair's rest length shrinks from its starting gap to zero at a fixed closing speed (default 0.25 m/s), so seams close smoothly at a bounded speed | Max seam gap < 1 mm, or 3 s simulated |
| Settling | Gravity on, normal damping | RMS particle speed stays below 2 mm/s for 1 s simulated (60 frames) |
| Settled (asleep) | Solver idles to save battery | Any change, or a mouse drag on cloth |
| Interactive | Normal simulation while the user drags | Drag ends → back to Settling |

During Assembling and Settling, the sim runs as many frames as the time budget allows rather than being locked to the display rate.

### 5.8 Stability guard

- A GPU reduction each frame computes a NaN/Inf flag, total kinetic energy, and the share of particles held at the displacement clamp (Section 5.5).
- A snapshot (positions + velocities) is kept every 30 frames.
- **Failure** means any of:
  - a NaN/Inf;
  - more than 5% of particles held at the displacement clamp for 30 consecutive frames;
  - kinetic energy rising for 60 consecutive frames with no user input and no phase change.
- On failure: restore the last snapshot, double the substeps, and retry.
- After 3 failed retries: stop simulating that garment, keep its last stable state, and show: *"Drape couldn't settle this garment. Try Draft quality or a different fabric."*

### 5.9 Mouse dragging

A ray cast from the cursor picks the nearest cloth triangle. While the user drags, a soft distance constraint pulls the picked point toward the cursor's position on a camera-facing plane. Because of the tunneling guard, cloth follows a fast-moving cursor with a slight lag. This is expected.

---

## 6. Fabrics

### 6.1 Physical properties (physical units)

| Property | Unit |
|---|---|
| Stretch stiffness: warp, weft, bias | N/m |
| Strain limit: warp, weft | % |
| Bending rigidity: warp, weft | µN·m |
| Weight | g/m² |
| Thickness | mm |
| Friction coefficient | 0–1 |
| Damping | 0–1 |

### 6.2 Visual properties

- Tileable base-color texture (tinted by the chosen color), normal map, roughness map
- Texture scale in millimeters per tile, so knit and weave structures render at true size
- Sheen color and sheen roughness, subsurface color (Filament cloth model)
- Inside-face darkening amount

### 6.3 Presets (13)

| Preset | Starting weight (g/m²) | Typical use in the library |
|---|---|---|
| Jersey | 160 | Test tee, shorts option |
| Heavyweight jersey | 240 | Streetwear tees and shorts |
| Waffle / thermal knit | 220 | Long sleeve thermal |
| French terry | 320 | Crewneck, hoodie, joggers, shorts |
| Fleece (brushed back) | 380 | Hoodie, crewneck, joggers |
| Rib knit | 280 | Cuffs, hem bands, neck ribs, beanie |
| Rigid denim (13.5 oz) | 458 | Baggy jeans |
| Twill | 270 | Cargo pants |
| Canvas | 400 | Cargo pants option |
| Nylon ripstop | 50 | Puffer shell and lining |
| Mesh | 120 | Shorts option |
| Satin | 100 | Linings, shorts option |
| Faux leather | 350 | Shorts and pants option |

Stretch, bending, strain-limit, thickness, friction and damping starting values come from published textile measurement ranges for each fabric category. Each value is then tuned against photos of real garments. The source and range for each value, plus the expected Cusick drape-coefficient range for the fabric category, are recorded next to the preset in the repository. The acceptance tests are in Section 12.2.

Users can adjust every property and save the result as a custom fabric in the project.

---

## 7. Garment library (`drape_garment`)

### 7.1 Generators and spec tables

Each garment is a **versioned parametric generator**:

- **Input:** a spec (garment measurements, i.e. points of measure) for one size, plus fit options.
- **Output:** a `Pattern` with pieces (cubic Bézier outlines), seams, notches, grainlines, internal lines, trims, fabric slots, layer relations and placements.

Each garment's **spec table** stores base values for size M and a grade increment per size from XS to XXL. Users can override any cell. Changing a cell regenerates the pattern for that size and re-drapes it. Overrides are saved in the project.

Generators draft patterns with standard block pattern-making methods. Before public launch, a freelance patternmaker reviews every block for fit.

### 7.2 Pattern validation (run on every generation)

- Every outline is closed and does not intersect itself; every piece has positive area.
- Every piece has a grainline, a fabric slot and a placement.
- Paired seam edges match in length within 2 mm, unless the seam declares ease or an elastic ratio accounts for the difference.
- **Measuring the generated pattern reproduces the spec within 1 mm.** For example, the hoodie's chest width measured across the front and back pieces at the armhole line must match the spec. Factories check patterns the same way.

A generator that fails validation is a bug. The app shows an error and never simulates an invalid pattern.

### 7.3 Initial placement

Body regions (torso front/back, arms, legs front/back, waist, neck, head) each have a bounding cylinder sized from the solved avatar. Each piece's 2D outline is wrapped onto its region's cylinder, with mirrored pieces placed symmetrically. Offset from the body: 3 cm for bottoms and headwear, 6 cm for tops, so a top always starts outside the bottom. Seams then pull the pieces together during Assembling.

### 7.4 Garments

| Garment | Wave | Pieces | Default fabrics | Special features |
|---|---|---|---|---|
| Test tee (internal only) | 1 | Front, back, 2 sleeves, neck rib | Jersey, rib | Physics baseline |
| Long sleeve thermal | 1 | Front, back, 2 sleeves, neck rib, 2 cuffs | Waffle knit, rib | Folded hem (fold line) |
| Crewneck | 1 | Front, back, 2 set-in sleeves, neck rib, 2 cuffs, hem band | French terry, rib | Elastic rib trims |
| Pullover hoodie | 1 | Front, back, 2 sleeves, 3-panel hood, kangaroo pocket, 2 cuffs, hem band | Fleece, rib | Pocket as layer +1 on front, open at hand openings; folded hood edge |
| Joggers | 1 | Front L/R, back L/R, waistband, 2 cuffs, back patch pocket | French terry, rib | Elastic waistband and cuffs; pocket layer |
| Shorts | 1 | Front L/R, back L/R, waistband | French terry | Elastic waistband; folded hem |
| Beanie | 1 | Body (one piece, back seam) with 4-section gathered crown | Rib knit | Negative ease against head circumference; 180° folded cuff with layer order |
| Baggy jeans | 2 | Front L/R, back L/R, back yoke L/R, waistband, 5 belt loops, 2 back pockets, front pocket facings | Rigid denim | Closed fly + button, 6 rivets, topstitching, strain-limited denim |
| Cargo pants | 2 | Front L/R, back L/R, waistband, belt loops, 2 bellows side pockets (pocket + gusset), 2 side flaps, 2 back pockets with flaps | Twill | Layered pockets and flaps, closed fly + button, topstitching |
| Puffer jacket | 2 | Front L/R, back, 2 sleeves, stand collar, each as shell + lining | Nylon ripstop | Quilted chambers with pressure/loft, closed center-front zipper, elastic cuffs |

Key points of measure per category:

- **Tops:** chest width, body length (HPS), shoulder width, sleeve length, sleeve opening, neck width, cuff and band heights. Hoodie adds hood height, hood width, pocket width and height. Puffer adds collar height, baffle spacing and loft.
- **Bottoms:** waist (relaxed), hip, front and back rise, thigh, knee, leg opening, inseam, waistband height. Cargo adds pocket width, height and depth, and flap height.
- **Beanie:** relaxed circumference, height, cuff height.

---

## 8. Avatar (`drape_avatar`)

### 8.1 Body model

- **Base:** the MakeHuman base mesh (about 13,000 vertices) with its CC0 shape targets: macro targets (gender, age, weight, muscle, height, proportions) and measurement targets (neck, bust, underbust, waist, hips, upper arm and others). Helper geometry (eyes, teeth, tongue) is removed.
- **Bases offered:** female and male, as presets of the gender target.
- **Pose:** a fixed A-pose with the arms about 45° from the torso, produced with MakeHuman's CC0 skeleton using linear blend skinning. Measurements are taken on the unposed shape; the pose is applied afterward.
- **Look:** matte light-grey mannequin, bald.

### 8.2 Measurements

| Measurement | Definition |
|---|---|
| Height | Floor to top of head |
| Bust / chest | Perimeter of the convex hull of a horizontal cross-section at the bust landmark (how a tape measure bridges hollows) |
| Underbust (female) | Convex-hull perimeter at the underbust landmark |
| Waist | Convex-hull perimeter at the natural-waist landmark |
| Hip | Largest convex-hull perimeter in the hip region |
| Shoulder width | Straight distance between the left and right shoulder-point landmarks |
| Arm length | Surface path from shoulder point through elbow to wrist |
| Inseam | Vertical distance from crotch to floor |
| Neck | Convex-hull perimeter at the neck base |
| Head circumference | Convex-hull perimeter at forehead level |

### 8.3 Solver

Levenberg–Marquardt over the continuous shape-target weights, within each target's allowed range. Objective: weighted squared error between measured and requested values, plus a small regularization term pulling toward a plausible body.

- **Accuracy:** every measurement within ±1 cm for every size in the built-in charts.
- **Outside that range:** the app shows the achieved value with a warning icon next to each measurement that missed.
- **Speed:** solve < 1 s; collision field bake < 2 s (M1).

### 8.4 Size charts

- **Built-in, labeled "generic":** US Women XS–XXL, US Men XS–XXL, EU Women 34–46, EU Men 44–56. Values are compiled from common industry ranges.
- **Custom charts:** a table editor with sizes as columns and body measurements as rows. Charts are saved in the project and selectable for the avatar.

---

## 9. Rendering (`drape_render`)

- **Engine:** Filament (Metal backend), drawing into the Qt viewport through a native `CAMetalLayer`.
- **Fabric material:** Filament's cloth shading model, with tinted base-color texture, normal map, roughness, sheen and subsurface color. Two-sided, with inside faces darkened by the fabric's inside-face amount.
- **Trims:** Filament's standard lit model with metallic/roughness (rivets, buttons, zipper teeth).
- **Topstitching (Wave 2):** instanced stitch geometry placed along topstitch lines, re-positioned on the cloth surface every frame. Parameters: stitches per inch, thread color, thread thickness.
- **Normals:** recomputed every frame from particle positions (area-weighted), averaged across seam vertex pairs so seams shade smoothly.
- **Lighting presets:** Studio Soft, Studio Contrast and Overcast Outdoor. Each combines a CC0 HDRI environment, a key light with soft shadows, ambient occlusion and anti-aliasing.
- **Fit map overlay:**
  - Strain shows maximum principal stretch versus the flat pattern, from 0% to 20%+.
  - Pressure shows body contact pressure in kPa, from body-collision impulses.
  - Both use a color ramp with an on-screen legend.
- **Sim → render path:** the Metal solver writes positions to shared-storage buffers. Apple Silicon's unified memory lets the CPU read them without a copy, and they are uploaded to Filament vertex buffers each frame.
- **Capture:** offscreen render at up to 4096 px on the long side, supersampled, PNG, with an optional transparent background.

---

## 10. Application (`drape_app`)

**Layout (Qt Widgets, docking panels):**

- **Center:** 3D viewport.
- **Left dock:**
  - **Library:** garments by category (Tops, Outerwear, Bottoms, Headwear) with thumbnails.
  - **Avatar:** base body, measurement form, size chart picker, chart editor.
- **Right dock:**
  - **Properties:** selected garment's size, spec table, per-piece fabric and color.
  - **Fabric:** preset list, property sliders, save custom fabric.
- **Toolbar:** Simulate (play/pause), Reset drape, Quality (Draft/Standard/Fine), Fit map (Off/Strain/Pressure), Lighting preset, Capture PNG, Units (cm/in).
- **Status bar:** sim phase, frames per second, particle count.

**Interaction:**

- Click a garment in the viewport to select it. Option-click selects a single piece for per-piece fabric and color.
- Library categories map to outfit slots: Tops and Outerwear → top, Bottoms → bottom, Headwear → headwear.
- Adding a garment to an occupied outfit slot replaces the current garment, with undo.
- Every change to the project is a `QUndoCommand`. Simulation state itself is not part of undo.

Ian directs the visual styling. This spec fixes layout and behavior only.

---

## 11. Files and error handling

### 11.1 `.drape` format

A ZIP container:

| Entry | Contents |
|---|---|
| `manifest.json` | Integer format version, app version, created/modified timestamps |
| `project.json` | Everything in `Project` (Section 4), including stored patterns |
| `cache/<garment-id>.bin` | Settled particle positions plus a hash of (pattern, fabric, quality, avatar). Used only if the hash matches; otherwise the garment re-drapes on open |
| `thumbnail.png` | Viewport snapshot for Finder and the open dialog |

Files from older format versions are migrated on load. Files from a newer version are refused with a clear message.

### 11.2 Error handling

| Situation | Behavior |
|---|---|
| Simulation failure | Rollback and retry, then stop with a message (Section 5.8) |
| Metal device unavailable or lost | Switch to `CpuSolver` at Draft quality, with a banner explaining why |
| Corrupt or unreadable file | Error dialog naming the problem; the app keeps running |
| Invalid generated pattern | Error naming the garment and the failed rule; nothing is simulated (Section 7.2) |
| Avatar measurement out of range | Achieved values are shown with per-measurement warnings (Section 8.3) |
| App crash | Autosave every 2 minutes; on next launch, offer to recover the autosaved project. Crash logs are written locally |

---

## 12. Testing

All portable code is written test-first. The framework is GoogleTest; Qt Test covers the app layer.

### 12.1 Unit tests

- Geometry and math utilities.
- `.drape` round trip: save → load gives an identical project. Every older format version has a fixture file that must load. Corrupted and truncated files must produce errors, never crashes.
- Sim mesh build: seam pairs are 1:1, every vertex has a valid rest position, and triangle quality stays above a minimum angle.
- Pattern validation (Section 7.2) for every generator × every size.
- Commands and undo/redo for every project change.

### 12.2 Physics tests

Run on `CpuSolver` everywhere and on `MetalSolver` on Macs:

1. **Stretch test:** a 200 × 50 mm strip under a known force per width must show strain that reproduces the fabric's stretch stiffness within 5%, inside the strain limit.
2. **Cantilever bending test** (Peirce method): a strip is pushed over a ledge until its tip drops to 41.5°. Then bending length c = overhang / 2, and the computed flexural rigidity W · c³ must match the fabric's bending rigidity within 10%.
3. **Drape test** (Cusick method): a 30 cm fabric disc on an 18 cm support disc. Each preset's simulated drape coefficient must fall in the range recorded for its fabric category (Section 6.3), and the presets must rank in the right stiffness order.
4. **Stability:** a hanging sheet and a sheet dropped on a sphere must settle with no stability failure, no body penetration beyond 2 mm, and a symmetric result.
5. **CPU–Metal parity:**
   - Short scenarios (≤ 60 frames): maximum per-particle difference < 1 mm.
   - Long scenarios: settled measurements (hem height, center of mass) agree within 2 mm.

### 12.3 Drape matrix

Every garment (including the test tee) × all 6 sizes × both base bodies = 120 runs. Each run must pass:

- No stability failure.
- Max seam gap < 1 mm when Assembling ends.
- No particle deeper than 2 mm inside the body after settling.
- No intersections between non-adjacent cloth triangles after settling at Standard quality. Draft-quality CI runs allow up to 0.5% of triangles.
- On Mac: settles within the time budgets in Section 14.

The matrix runs on `CpuSolver` at Draft quality nightly in Linux CI. The full matrix runs at Standard on a Mac weekly and before every release.

### 12.4 Visual regression

Every garment at size M on both bodies, under Studio Soft, from fixed front and back cameras. Images are compared with a perceptual metric against stored references. Any difference above threshold fails the run and produces a diff image for review.

### 12.5 Avatar tests

- Every size in every built-in chart solves to within ±1 cm on every measurement.
- Solve time < 1 s and collision-field bake < 2 s on the reference Mac.

### 12.6 Benchmarks

Automated benchmarks for every target in Section 14, run on a Mac. A regression beyond 10% fails the run.

### 12.7 Usability check

Before milestone 1 is called done, two designers who have not seen Drape before must complete the task "set your body to size M, dress it in hoodie + joggers + beanie in fleece, change the hoodie color, export a 4K PNG" without help.

---

## 13. Build, development workflow and distribution

### 13.1 Toolchain

- C++20, CMake with presets, Ninja.
- Source dependencies pinned to exact versions via CMake FetchContent: nlohmann/json v3.12.0, miniz 3.1.0, GoogleTest v1.17.0, CDT 1.4.5. Eigen 3.4 comes from the system package manager (apt on Linux, Homebrew on Mac). Qt comes from the official Qt installer (latest Qt 6 LTS). Filament comes from its official prebuilt macOS release (pinned).
- **Targets:** macOS 14+ on arm64 for the app. Linux x86-64 for the portable modules (development and CI only).

### 13.2 Workflow

- **Repository:** private GitHub repo `drape`, with feature branches and pull requests.
- **Claude's workspace (Linux):** builds and runs all tests for `drape_core`, `drape_garment`, `drape_avatar` and the CPU solver on every change.
- **GitHub Actions:**
  - A Linux job runs the portable tests on every push and the Draft drape matrix nightly.
  - A macOS arm64 job does a full app build and runs the non-GPU tests on every push. GPU tests run there when the runner exposes a Metal device; otherwise they run on Ian's Mac.
- **Ian's MacBook Pro:**
  - One-time setup: Xcode Command Line Tools, Homebrew `cmake` and `ninja`, Qt.
  - `scripts/build-mac.sh` builds the app; `scripts/test-mac.sh` runs the GPU, parity and benchmark tests.
  - With Ian's permission, Claude runs these scripts in the Mac's Terminal.

### 13.3 Distribution (milestone 1)

- A Developer ID–signed, notarized `.dmg`, so macOS opens it without security warnings. This requires an Apple Developer Program membership ($99/year).
- Qt is linked dynamically as frameworks inside the app bundle, with license notices included, to meet LGPL v3.
- No auto-update in milestone 1.

---

## 14. Performance targets

The baseline machine is an **M1 MacBook Air with 8 GB**. Targets are measured on Ian's MacBook Pro throughout development and confirmed on an M1 Air before launch.

| Scenario | Target |
|---|---|
| Hoodie at Standard: from Assembling start to Settled | ≤ 5 s |
| Hoodie at Standard: frame rate while orbiting or dragging cloth | ≥ 30 fps |
| Hoodie at Fine: Assembling start to Settled | ≤ 30 s |
| Outfit (hoodie + joggers + beanie) at Standard: Assembling start to Settled | ≤ 10 s |
| Viewport frame rate with the sim asleep | 60 fps |
| Avatar measurement solve | < 1 s |
| Collision field bake | < 2 s |
| 4096 px PNG capture | < 5 s |
| App launch to usable window | < 3 s |
| Peak memory, outfit at Fine | < 3 GB |

---

## 15. Milestone 1 is done when

1. All nine library garments and the test tee pass the drape matrix (Section 12.3).
2. All physics tests pass on both solvers (Section 12.2).
3. Every performance target in Section 14 is met on the M1 Air baseline.
4. Avatar measurements land within ±1 cm for every built-in chart size.
5. The usability check passes (Section 12.7).
6. A freelance patternmaker has reviewed every garment block, and their fixes are in.
7. A signed, notarized `.dmg` installs and runs on a clean Mac.

---

## 16. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Self-collision robustness (pockets, flaps, hood, puffer) | Tangling or poke-through on complex garments | Layer-order constraints for in-garment layers; rest-distance exclusion; tunneling guard; drape matrix catches regressions; continuous collision planned in the layering sub-project |
| Fabric realism | Garments look "CG" and lose designer trust | Physical units; stretch, cantilever and Cusick tests; tuning against real garment photos |
| Pattern fit quality | Brands judge fit instantly | Spec-table validation; patternmaker review before launch |
| Mac-only build loop slows development | Slower iteration on GPU, render and UI code | Most logic lives in portable modules tested on Linux; macOS CI on every push |
| Filament limits for fabric-specific rendering (topstitching, inside faces) | Visual compromises | Rendering sits behind `drape_render`'s interface; custom Filament materials; topstitching as our own instanced geometry |
| Puffer pressure stability | Wave 2 slips | Built only after Wave 1 passes; volume constraints are well-studied; stability guard |
| Performance on 8 GB M1 | Slow drape hurts first impressions | Quality presets, auto-sleep, benchmarks in CI, unified-memory buffers |
| Qt LGPL obligations | Licensing exposure | Dynamic linking, notices, no Qt modifications; buy a commercial license when revenue allows |

---

## 17. Implementation phases

Milestone 1 is too large for one implementation plan. It is built in six phases. Each phase gets its own plan and ends with working, tested software.

| Phase | Delivers | Runs on |
|---|---|---|
| A. Foundations | `drape_core` data model and `.drape` files; sim mesh builder; `CpuSolver` with stretch, bending, seams, gravity, damping and body collision; the collision-field bake from `drape_avatar`, run on a static test body mesh; test tee generator; physics tests 1, 2 and 4 (Section 12.2). Result: the test tee drapes headlessly on a static test body, verified by tests and a developer-only OBJ dump for inspection | Linux |
| B. Mac app shell | Qt app skeleton, Filament viewport, `MetalSolver` with CPU–Metal parity tests. Result: the test tee drapes live on screen | Mac |
| C. Avatar | MakeHuman body, A-pose, measurement solver, built-in and custom size charts, Avatar panel | Linux + Mac |
| D. Wave 1 | Elastic, fold lines, layer order, self collision, outfit layering, air drag, mouse dragging, fit map, 13 fabric presets, Wave 1 garments, spec table, autosave and recovery, PNG capture, lighting presets, Cusick test, drape matrix, visual regression | Linux + Mac |
| E. Wave 2 | Pressure, trims, closed zippers and flies, topstitching, jeans, cargo pants, puffer jacket | Linux + Mac |
| F. Launch readiness | Performance tuning on the M1 Air, patternmaker review fixes, usability check, signing, notarization, `.dmg` | Mac |
