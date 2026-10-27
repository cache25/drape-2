# Drape — Mac App Shell (Phase B) Design

**Date:** 2026-10-25
**Status:** Approved 2026-10-25; amended during execution (parity scenes and long-run tolerance, Section 7.2, 2026-10-26)
**Amends:** `2026-10-01-drape-milestone-1-design.md` Sections 3.4, 13.1 and 17 (amendments listed in Section 10)
**Builds on:** Phase A (foundations) and Phase A2 (mannequins), `main` at `af4e333`

## 1. Goal

Open Drape as a real Mac app and watch the Phase A test tee drape live on the A2 mannequins, simulated on the GPU.

Phase B delivers, in one phase (Ian chose not to split it):

- the Qt 6 application shell (`drape_app`);
- the Filament viewport (`drape_render`);
- `MetalSolver`, the GPU implementation of the existing Newton–CG cloth solver, behind the same `Solver` interface as `CpuSolver`, with CPU–Metal parity tests;
- a frame-by-frame drape session, so the app can simulate on its own thread.

Out of scope (later phases): Library, Avatar, Properties and Fabric panels; fabric presets and textures; lighting presets; fit map; mouse dragging of cloth; `.drape` open/save, autosave and recovery; PNG capture UI; signing and notarization; GitHub Actions CI.

## 2. Decisions

| Question | Decision |
|---|---|
| Slicing | One phase: app, viewport and Metal solver together |
| Qt source | Homebrew `qt` (6.12), not the official installer. *Amended 2026-10-27 (Ian): on Ian's Mac, the official Qt 6.12.0 installed with aqtinstall onto the external drive, because the system drive has no room for Homebrew Qt; `scripts/build-mac.sh` finds it, and Homebrew still works.* |
| GPU solver approach | Full GPU port in 32-bit, with the accept/reject sums in 64-bit on the CPU (Section 5) |
| Where it builds | Ian's MacBook Pro (Apple M1, 8 GB, macOS 26), in `Dev 2/drape/build/` on the external volume; Claude runs the scripts through the linked desktop app |
| Garment and bodies | The Phase A test tee (men's on the male, women's cut on the female); no new garments |

## 3. Build and dependencies

### 3.1 Modules

| Module | New in B | Platforms |
|---|---|---|
| `drape_sim` | `DrapeSession` (portable); `MetalSolver` and its shaders (Mac only) | CPU parts: Linux + Mac. Metal: Mac |
| `drape_render` | New: Filament viewport, materials, camera, offscreen render | Mac |
| `drape_app` | New: Qt application | Mac |

On Linux, CMake skips the Mac-only targets entirely, so the cloud workspace keeps building and testing every portable module.

### 3.2 Pinned dependencies

| Component | Version | How it arrives |
|---|---|---|
| Qt 6 (Widgets) | 6.12 (Homebrew `qtbase`; the full `qt` formula pulls 38 modules) | `brew install qtbase` or aqtinstall (see README), found with `find_package(Qt6 6.12 COMPONENTS Widgets)`; optional at configure time (without it, everything but the app builds) |
| Filament | v1.78.1 (stable, 2026-10-21), official prebuilt macOS release | Downloaded by CMake into the build folder, SHA-256 checked |
| metal-cpp | Apple's release matching the installed macOS SDK, pinned by URL and SHA-256 | Downloaded by CMake into the build folder |
| CC0 studio HDRI | One Poly Haven HDRI, pinned by URL and SHA-256 | Downloaded once; converted to an IBL with Filament's `cmgen` at build time |

Nothing except Qt is installed on the Mac's internal drive (about 7 GB free).

### 3.3 Toolchain and scripts

- C++20, CMake presets, Ninja, Apple Clang from Xcode. A new `mac` configure preset (Debug) and `mac-release` preset.
- Targets: macOS 14+, arm64 only. The app is unsigned during development (signing is Phase F).
- Metal shaders are `.metal` sources in the repo. *(As built, 2026-10-26: Xcode 26 ships the Metal shader compiler as a separate ~700 MB download, which would not fit on the development Mac; the kernel headers and wrappers are combined at build time into one `drape_kernels.metal` that `MetalSolver` compiles at runtime through Metal.framework, with fast math off. The file sits in the app bundle's Resources. A prebuilt `.metallib` can replace it once the toolchain is installed.)*
- `scripts/build-mac.sh`: configure and build the `mac` preset.
- `scripts/test-mac.sh`: run the Mac test suite (portable tests, Metal physics and parity tests, render smoke test, app self-test).

## 4. Application and viewport

### 4.1 Window

- **Center:** the 3D viewport. No placeholder docks.
- **Toolbar:**
  - Simulate (play/pause);
  - Reset drape (rebuild the scene and start again from Assembling);
  - Quality: Draft / Standard / Fine (rebuilds the sim mesh at that particle distance and re-drapes);
  - Body: Male / Female (switches mannequin and tee cut, then re-drapes) — a stand-in until the Avatar panel in Phase C;
  - Solver: GPU / CPU (rebuilds the scene on the chosen backend and re-drapes).
- **Status bar:** sim phase (Assembling / Settling / Settled, or Failed), frames per second, particle count. On a stability failure it shows the spec 5.8 message.
- Visual styling follows Ian's direction; this spec fixes layout and behavior only.

### 4.2 Viewport (`drape_render`)

- Filament (Metal backend) draws into a `QWidget` that owns a native `NSView` backed by a `CAMetalLayer`; Filament's swap chain is created from that layer. It resizes with the widget and handles Retina scale.
- **Mannequin:** Filament's standard lit material, white with a soft satin finish (spec A2 Section 3).
- **Tee:** Filament's cloth shading model, flat blue base color like the developer previews (no textures in Phase B), two-sided, with back faces darkened by a fixed inside-face amount.
- **Normals:** recomputed every frame from particle positions (area-weighted) and averaged across seam vertex pairs, so seams shade smoothly (spec 9).
- **Lighting:** one setup — the CC0 studio HDRI as image-based light plus a key light with soft shadows. The three presets are Phase D.
- **Camera:** orbit (left-drag), pan (right-drag or two-finger drag), zoom (scroll or pinch). On launch and on Body change, it frames the whole mannequin.
- **Offscreen render:** `drape_render` can render a scene to an RGBA image without a window (used by the smoke test and the app self-test).

### 4.3 Simulation thread and data flow

- **`DrapeSession` (portable, `drape_sim`):** the drape runner as a frame-by-frame state machine. `advance()` runs one frame and returns the state: Assembling, Settling, Settled or Failed. It contains everything `drapeToRest` does today: phases, seam-gap end of assembly, settle detection, snapshots every 30 frames, the stability guard and its retries. `drapeToRest` becomes a loop over `DrapeSession`, so every existing runner and tee test still covers it.
- **`SimThread` (app):** owns the solver and the session. It runs frames as fast as the solver allows while Assembling or Settling, and sleeps once Settled. Toolbar actions reach it as queued commands (play/pause, reset, rebuild with new quality/body/backend).
- **Snapshot handoff:** after each frame the sim thread copies particle positions into the back half of a double buffer and swaps under a mutex; the viewport reads the latest front buffer every display frame. The UI thread never waits on physics.
- **Display:** the viewport renders at the display rate (60 fps target) whether or not the sim is running.

## 5. MetalSolver

### 5.1 Same algorithm

`MetalSolver` implements spec 5.1 and 5.5 exactly as `CpuSolver` does, step for step: per substep, predict `y` and the initial guess `x + h·v`; pick contact planes for particles near the body; run the configured Newton iterations, each one assembling gradient and block-sparse Hessian, solving with block-Jacobi preconditioned CG restricted to contact planes, reading contact normal forces from the residual, and backtracking on total energy; then the post-substep safety push-out, pins, velocity update and damping. Constants (`kContactReach`, `kContactSnap`, `SimSettings`) are shared, not copied. Mesh building, slot tables and constraint setup run once in `build()` on the CPU with the same code as `CpuSolver` and are uploaded.

### 5.2 Precision

Apple GPUs have no 64-bit floating point. The rules:

1. **Displacement variables.** Inside a substep the GPU works in `u = x − xStart` (and `y − xStart`), not absolute positions. Substep motions are sub-millimetre, so 32-bit floats keep nanometre-scale accuracy where the energies are evaluated. Positions between substeps stay 32-bit absolute, as on the CPU.
2. **Accept/reject in 64-bit.** For the line search the GPU writes each term's energy (per particle, triangle, hinge and seam); the CPU sums those in 64-bit from shared memory, in a fixed order, and makes the Armijo decision. The same applies to the Newton slope `grad·step`.
3. **CG on the GPU.** CG vectors and scalars are 32-bit. Dot products use a fixed-order two-level reduction (threadgroup tree, then a single final pass), so results are deterministic.
4. **Fix precision, not tolerances.** If a parity test fails because of 32-bit precision, the fix is to move that quantity to 64-bit on the CPU. The 1 mm / 2 mm parity tolerances change only with Ian's sign-off.

### 5.3 GPU work

- **Term kernels:** one thread per particle (inertia, friction), stretch triangle, hinge and seam. Each writes its energy, its gradient contributions and its local Hessian blocks to its own output slots.
- **Gather kernels:** one thread per particle sums gradient contributions; one thread per CSR block slot sums Hessian contributions. Both read precomputed contribution lists in a fixed order. No atomics anywhere.
- **Preconditioner:** each free particle's 3×3 diagonal block is inverted on the GPU (LDLT, identity fallback when not positive definite, as on the CPU).
- **CG:** block-sparse matrix–vector product, contact-plane filter, vector updates and reductions as kernels. The CG loop is encoded in chunks; a GPU-side "converged" flag (relative residual ≤ `cgTolerance`, or `pAp ≤ 0`) turns later iterations into no-ops, so the CPU checks only between chunks.
- **Body field:** the collision field's samples are uploaded once per `build()`. Kernels sample it with the same trilinear and central-difference code as `CollisionField::sample` and `gradient`, not hardware texture filtering.
- **Per frame on the CPU:** `stats()` (non-finite flag, kinetic energy, RMS speed, seam gap, penetration) is computed in 64-bit from the shared position and velocity buffers.

### 5.4 Determinism and memory

- Shaders compile with fast-math off. Same inputs on the same Mac give bit-identical results, and `snapshot()`/`restore()` replays a run bit for bit, as the stability guard requires.
- All buffers use shared storage. `readPositions` copies from the shared position buffer; the viewport reads the published double buffer (Section 4.3).

## 6. Error handling

- No Metal device, or the `.metallib` fails to load: the app falls back to `CpuSolver`, disables the GPU option, and says so in the status bar. `MetalSolver` construction throws `std::runtime_error` naming the cause.
- Filament or Qt initialization failure: the app shows a modal error naming the component and exits with a non-zero code.
- Stability failure after 3 retries: the garment stops at its last stable state and the status bar shows the spec 5.8 message.
- Bad mannequin or garment assets: existing loader errors (absolute paths) surface in a modal error.

## 7. Testing

### 7.1 Cloud (Linux)

- `DrapeSession` unit tests: state sequence, settle detection, retry and failure paths, and equality with `drapeToRest` results on the existing runner scenarios.
- Every existing test keeps passing (168 at `af4e333`).

### 7.2 Mac (`scripts/test-mac.sh`)

- **Physics tests 1, 2 and 4 on both solvers** (spec 12.2): the strip stretch tests, the cantilever tests, and the stability tests (hanging sheet, sheet on a sphere, rests on the body, friction slope) are parameterized over the backend and run on `CpuSolver` and `MetalSolver`. Test 3 (Cusick drape) needs the fabric presets and stays in Phase D.
- **Parity (spec 12.2 test 5):**
  - Short runs, ≤ 60 frames — hanging sheet, a small patch dropped on the sphere, and the full tee continuing 60 frames from its CPU-settled state on the male mannequin: maximum per-particle difference < 1 mm. *(Amended during execution, 2026-10-26: the sheet dropped on the sphere and the tee's first 60 frames were dropped as short-run scenes because they are chaotic at the 1 mm scale even CPU vs CPU — a 1 nm change to the start moves the CPU's own result by ~10 cm and ~1.4 cm respectively.)*
  - Long runs — the tee draped to rest on the male and on the female: both settle, and hem height and center of mass agree within 2 mm **or within 1.5× the CPU's own spread, whichever is larger** — the spread is the largest difference from the unperturbed CPU result over ten CPU runs whose start moves by up to 1 nm. *(Ian's decision, 2026-10-26: the female tee's settled hem varies by up to 6.7 mm CPU vs CPU, so a fixed 2 mm check cannot tell a correct GPU solver from a wrong one. Revised by Ian 2026-10-27 from "max of three runs" to "1.5× max of ten": on the Mac a sound GPU run measured 6.47 mm against a max-of-three limit of 4.56 mm, and max-of-three fails a sound run about one time in four.)*
- **Determinism:** two `MetalSolver` runs of the same scenario, and a snapshot/restore replay, are bit-identical.
- **Tee on the GPU:** the existing tee drape checks (settled, seam gap, penetration) pass with `MetalSolver` on both bodies.
- **Render smoke test:** Filament renders the settled tee on the male offscreen at 800×1000; the image is not blank, and the garment's blue-tinted pixels cover between 5% and 40% of the frame.
- **App self-test:** `Drape --selftest` opens the window, drapes the tee on the male with the GPU solver at Standard, saves a viewport screenshot, and exits 0 once Settled (1 on failure). Claude reviews the screenshot and sends it to Ian.

### 7.3 Performance

Recorded in the hand-off notes, not gated: tee settle time on CPU vs GPU at Draft, Standard and Fine; viewport frame rate while draping.

Gated for Phase B on Ian's Mac:

- viewport ≥ 60 fps with the sim asleep (spec 14);
- launch to usable window < 3 s (spec 14);
- `MetalSolver` settles the tee faster than `CpuSolver` at Standard.

The hoodie and outfit targets in spec 14 apply once those garments exist (Phase D).

## 8. Phase B is done when

1. Ian opens Drape on his Mac, watches the tee drape live on both mannequins, and can switch GPU/CPU, Quality and Body from the toolbar.
2. Every test in Section 7 passes: portable tests in the cloud and on the Mac; Metal, parity, render and self-test on the Mac.
3. The Section 7.3 gates are met and the recorded numbers are in the hand-off notes.
4. The milestone spec amendments in Section 10 are applied.

## 9. Risks

| Risk | Mitigation |
|---|---|
| 32-bit precision breaks parity on stiff fabrics | Displacement variables; 64-bit line-search sums on the CPU; Section 5.2 rule 4 |
| CG convergence differs in 32-bit, so GPU takes more iterations | Same 50-iteration cap and tolerance; parity tests measure the result, not iteration counts; record iteration counts in benchmarks |
| GPU sync overhead (many small dispatches per frame) eats the speedup | CG encoded in chunks with a GPU-side converged flag; CPU syncs only for the line search and between chunks |
| Filament–Qt integration on macOS 26 | Native `CAMetalLayer` view, the approach Filament's own macOS samples use; renderer behind `drape_render`'s interface |
| 8 GB RAM and 7 GB free on the internal drive | Build and downloads on the external volume; only Qt on the internal drive |
| The Mac is offline when a step needs it | Portable work (`DrapeSession`, shared solver code) is done and tested in the cloud first; Mac steps are batched |

## 10. Amendments to the milestone spec

- **3.4 Third-party components:** Qt row reads "Qt 6 (6.12 from Homebrew during development)"; Filament row reads "Filament (v1.78.1 prebuilt macOS release)".
- **13.1 Toolchain:** "Qt comes from the official Qt installer (latest Qt 6 LTS)" becomes "Qt comes from Homebrew (`qt`, 6.12) during development; the shipping build may move to the official LTS"; Filament is pinned to v1.78.1.
- **17 Implementation phases:** row B adds "`DrapeSession` frame-by-frame runner; app toolbar (Simulate, Reset, Quality, Body, Solver)".
- **Status line:** adds "Phase B Mac app shell (Sections 3.4, 13.1, 17)".
