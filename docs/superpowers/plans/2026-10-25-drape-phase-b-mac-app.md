# Drape Phase B — Mac App Shell Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drape opens as a Mac app and the test tee drapes live on the male and female mannequins, simulated on the GPU by a `MetalSolver` that matches `CpuSolver`.

**Architecture:** Portable work comes first and is tested in the Linux workspace. That covers the shared solver model, a frame-by-frame `DrapeSession`, a tee-scene builder, and the sim controller thread. It also covers the whole GPU algorithm, whose kernel bodies are written once in headers that compile both as Metal Shading Language and as C++. A CPU "emulated device" runs those headers, so GPU math, precision and CPU parity are proven before a Mac is involved. The Mac tasks then add the Metal device, the Filament renderer and the Qt app as thin layers over tested code.

**Tech Stack:** C++20, CMake presets + Ninja, Eigen 3.4, GoogleTest; Mac: Apple Clang, metal-cpp, Metal Shading Language 3.1, Filament v1.78.1, Qt 6.12 Widgets.

**Spec:** `docs/superpowers/specs/2026-10-25-drape-phase-b-mac-app-design.md` (Phase B), within `docs/superpowers/specs/2026-10-01-drape-milestone-1-design.md` (milestone).

## Global Constraints

- C++20. Every new library/test target uses `drape_add_library` / `drape_add_tests` (warnings as errors).
- Portable code builds and passes on Linux (`dev` preset). Mac-only targets are inside `if(APPLE)` and are skipped on Linux.
- macOS 14+, arm64 only: `CMAKE_OSX_DEPLOYMENT_TARGET=14.0`, `CMAKE_OSX_ARCHITECTURES=arm64`. The app is unsigned.
- Pinned dependencies:
  - Qt 6.12 from Homebrew: `find_package(Qt6 6.12 REQUIRED COMPONENTS Widgets)`, prefix `/opt/homebrew/opt/qt`.
  - Filament v1.78.1, the official prebuilt macOS release (`filament-v1.78.1-mac.tgz` from `github.com/google/filament/releases`), SHA-256 checked.
  - metal-cpp: tag `release/metal-cpp_macOS26.4_iOS26.4` of `github.com/apple/metal-cpp` (commit `c595afef4a5dc388f4047cd0c69f9e7f9468d9ed`).
  - HDRI: Poly Haven `studio_small_09` 2K `.hdr` (CC0), SHA-256 checked.
- Downloads go into the build folder. Nothing but Qt is installed on the Mac's internal drive.
- The Mac checkout is `/Volumes/Backup of Macintosh HD shnnon/Dev 2/drape`. The path contains spaces, so quote every path in scripts and CMake custom commands.
- Kernels: no atomics, fixed-order reductions, Metal compiled with `-fno-fast-math`. Same inputs give bit-identical results on the same device.
- Parity tolerances: short runs (≤ 60 frames) < 1 mm per particle; long runs agree on hem height and center of mass within 2 mm. These change only with Ian's sign-off. If precision fails, move that quantity to 64-bit on the CPU (spec 5.2 rule 4).
- Contact constants exist once, in `sim_model.hpp`: `kContactReach = 0.01`, `kContactSnap = 0.0005`.
- ctest labels: `slow` (> 10 s), `gpu` (needs a Metal device), `perf` (timing gates, release preset).
- Commit messages end with the session's `Co-Authored-By` / `Claude-Session` trailers.
- **Sync between cloud and Mac:** Tasks 1–9 run in the cloud. Their commits reach the Mac through `github.com/cache25/drape-2`, on branch `phase-b` (pushed with Ian's approval; the Mac runs `git fetch && git checkout phase-b`). Mac tasks run through Desktop Commander and push back the same way.

## Review Focus

1. **Toolbar changes while the sim runs** (switching Body or Quality mid-Assembling, rapid clicks): the viewport must never draw positions against the wrong mesh. Pinned by `SimController.RapidRebuildsNeverMixRevisions` (Task 9).
2. **Quitting while simulating:** the sim thread must stop and join promptly. Pinned by `SimController.DestroyWhileAssemblingJoinsQuickly` (Task 9).
3. **No Metal or a missing `.metallib`:** the app must fall back to CPU, not crash. Pinned by `MetalSolver.MissingMetallibThrowsWithPath` (Task 11) and `SimController.FallsBackToCpuWhenGpuUnavailable` (Task 9).
4. **Window resize and Retina scale changes** (dragging to an external display): no stretched or blank frame. Pinned by the self-test resize step (Task 13) and `Renderer.OffscreenRendersAtTwoSizes` (Task 12).
5. **Stability failure in the app:** the garment freezes at its last stable state, the status bar shows the spec 5.8 message, and the thread idles. Pinned by `SimController.FailureStopsAndReportsMessage` (Task 9).

---

## File Structure

```
libs/sim/
  src/sim_model.hpp/.cpp            SimModel: scene setup, CSR structure, stats, snapshot (shared by both solvers)
  src/cpu_solver.cpp                CpuSolver = SimModel + double work arrays (refactored)
  include/drape/sim/drape_session.hpp, src/drape_session.cpp   frame-by-frame runner
  include/drape/sim/solver_factory.hpp, src/solver_factory.cpp SolverBackend, makeSolver, isAvailable
  kernels/gpu_compat.h              float3/Mat3 types and helpers for C++ and MSL
  kernels/gpu_layout.h              buffer element structs shared by host and kernels
  kernels/term_math.h               float energies, field sampling, collision, 3x3 inverse
  kernels/kernels.h                 per-thread kernel bodies
  kernels/drape_kernels.metal       [[kernel]] wrappers (Mac)
  src/gpu/compute_device.hpp        ComputeDevice interface, Kernel enum
  src/gpu/emulated_device.hpp/.cpp  runs kernels.h on the CPU
  src/gpu/gpu_solver_core.hpp/.cpp  host orchestration (substep, Newton, CG chunks, line search)
  src/gpu/metal_device.hpp/.cpp     metal-cpp device (Mac)
  include/drape/sim/metal_solver.hpp, src/gpu/metal_solver.cpp (Mac)
libs/session/                       NEW portable lib drape_session
  include/drape/session/tee_scene.hpp, src/tee_scene.cpp
  include/drape/session/position_exchange.hpp
  include/drape/session/sim_controller.hpp, src/sim_controller.cpp
libs/render/
  include/drape/render/normals.hpp, orbit_camera.hpp, image_metrics.hpp + src (portable: drape_render_math)
  include/drape/render/renderer.hpp, src/renderer.cpp, src/metal_layer.mm, materials/*.mat (Mac: drape_render)
apps/drape/                         Qt app (Mac): main.cpp, main_window.*, viewport_widget.*
cmake/DrapeMacDependencies.cmake    Filament, metal-cpp, HDRI, Qt (Mac)
scripts/build-mac.sh, scripts/test-mac.sh
```

---

### Task 1: Extract `SimModel` from `CpuSolver`

A pure refactor. Both solvers will share scene setup, bookkeeping, stats and snapshots.

**Files:**
- Create: `libs/sim/src/sim_model.hpp`, `libs/sim/src/sim_model.cpp`
- Modify: `libs/sim/src/cpu_solver.cpp`, `libs/sim/CMakeLists.txt`
- Test: `libs/sim/tests/test_sim_model.cpp`

**Interfaces:**
- Produces (`namespace drape::sim`, private header, visible to tests via `src/`):
  - `inline constexpr double kContactReach = 0.01; inline constexpr double kContactSnap = 0.0005;` (moved from `cpu_solver.cpp`).
  - `struct SimModel` holding everything in today's `CpuSolver::Impl` that is not a Newton/CG work array. Members keep their current names and types: `settings, body, garments, fabrics, x, v, mass, vertexArea, fExt, pFabric, pGarment, pinned, pins, stretch, hinges, hingeSides, hingeGeom, seams, rowStart, colIndex, diagSlot, stretchSlots, hingeSlots, seamSlots, contactForce`. `Garment` and `HingeSide` move with it.
  - Methods: `static SimModel build(const SimScene&)` (today's `CpuSolver::build` body plus `buildStructure`'s CSR part; same exceptions); `void updateFabric(const GarmentId&, const std::vector<FabricPhysical>&)`; `void setPhase(const GarmentId&, SimPhase)`; `std::size_t garmentIndex(const GarmentId&) const`; `double contactRadius(std::size_t i) const`; `double seamTarget(const SeamSpring&) const`; `double hingeStiffness(std::size_t) const`; `void updateMass(std::uint32_t)`; `std::uint32_t slot(std::uint32_t row, std::uint32_t col) const`; `SimStats stats() const`; `Snapshot snapshot() const`; `void restore(const Snapshot&)`; `ParticleRange range(const GarmentId&) const`.
  - `CpuSolver::Impl` becomes `{ SimModel m; /* xd, xStart, y, grad, step, r, z, p, ap, trial, apFull, resid, blocks, precond, contactNormal, contactOffset, frictionLambda, contactNear, contactActive, substepH */ }`.

- [ ] **Step 1: Write the failing test** `test_sim_model.cpp`:
  - `SimModel.BuildMatchesCpuSolverCounts`: build the 0.2 × 0.05 m strip from `test_cpu_solver.cpp`'s `StripStretchWarp` scene; `EXPECT_EQ(m.x.size(), cpu.particleCount())`, `EXPECT_EQ(m.rowStart.size(), m.x.size() + 1)`, and every `diagSlot[i]` satisfies `colIndex[diagSlot[i]] == i`.
  - `SimModel.SnapshotRoundTrip`: `m.restore(m.snapshot())` leaves `x`, `v`, `contactForce` bit-identical.
  - `SimModel.StatsMatchCpuSolver`: after `cpu.step({1/60., 1, 2})` ×10, a `SimModel` restored from `cpu.snapshot()` gives `stats()` equal to `cpu.stats()` field by field.
- [ ] **Step 2: Run it to verify it fails.** `cmake --build --preset dev`. Expected: build fails (`sim_model.hpp` missing).
- [ ] **Step 3: Implement.** Move code without changing arithmetic or evaluation order. `CpuSolver` methods delegate to `impl_->m`, and `CpuSolver::build` sizes the work arrays after `SimModel::build`.
- [ ] **Step 4: Verify.** `ctest --preset dev -LE perf`. Expected: all tests pass (168 plus the 3 new ones), including the bit-for-bit `SnapshotRestoreIsDeterministic` tests.
- [ ] **Step 5: Commit** (`refactor(sim): SimModel shared by solvers`).

---

### Task 2: `DrapeSession`, the frame-by-frame runner

**Files:**
- Create: `libs/sim/include/drape/sim/drape_session.hpp`, `libs/sim/src/drape_session.cpp`
- Modify: `libs/sim/src/drape_runner.cpp` (becomes a loop), `libs/sim/CMakeLists.txt`
- Test: `libs/sim/tests/test_drape_session.cpp`

**Interfaces:**
- Consumes: `Solver`, `DrapeOptions`, `DrapeResult`, `kDrapeFailureMessage` (unchanged).
- Produces:
  ```cpp
  enum class DrapeState { Assembling, Settling, Settled, Failed, TimedOut };
  class DrapeSession {
   public:
    DrapeSession(Solver& solver, std::vector<GarmentId> garments, DrapeOptions options = {});  // sets Assembling
    DrapeState advance();            // one frame; returns the new state. No-op once finished().
    DrapeState state() const;
    bool finished() const;           // Settled, Failed or TimedOut
    const DrapeResult& result() const;
    double simulatedSeconds() const;
  };
  ```
  `drapeToRest(solver, garments, o)` = construct a session, `while (!s.finished()) s.advance();`, return `s.result()`.

- [ ] **Step 1: Write the failing tests** (reuse `ScriptedSolver` by moving it into `libs/sim/tests/scripted_solver.hpp`):
  - `DrapeSession.StartsAssembling`: right after construction, `state() == Assembling`, `result().framesRun == 0`, and the solver saw `setPhase(Assembling)`.
  - `DrapeSession.SequenceToSettled`: the script gives seam gap 0 from frame 1 and rms 0. Then advance #1 returns `Settling`, advances #2…#60 return `Settling`, and #61 returns `Settled`.
  - `DrapeSession.AdvanceAfterFinishIsNoOp`: after `Settled`, `advance()` returns `Settled` and `framesRun` is unchanged.
  - `DrapeSession.FailureMessage`: with a script that is non-finite on every frame, the session ends in `Failed` with `result().message == kDrapeFailureMessage` and `retries == 3`.
  - `DrapeSession.TimedOut`: with a script that is never still and `maxSimSeconds = 1`, the session ends in `TimedOut` after 60 frames, with neither `settled` nor `failed` set.
  - `DrapeSession.MatchesDrapeToRestOnHangingSheet`: the `Runner.HangingSheetSettlesSymmetric` scene on two `CpuSolver`s gives identical `DrapeResult` fields and bit-identical positions.
- [ ] **Step 2: Run to verify they fail** (build error: header missing).
- [ ] **Step 3: Implement.** Move the loop state of `drapeToRest` (`time`, `phaseTime`, `substeps`, `stillFrames`, `riseFrames`, `keptFrames`, `lastEnergy`, `saved`) into members. Keep the logic order and `kTimeEps` exactly.
- [ ] **Step 4: Verify.** `ctest --preset dev -LE perf`. Expected: all pass, including every `Runner.*` and tee test unchanged.
- [ ] **Step 5: Commit** (`feat(sim): DrapeSession frame-by-frame runner; drapeToRest loops over it`).

---

### Task 3: `drape_session` library and the tee scene builder

**Files:**
- Create: `libs/session/CMakeLists.txt`, `libs/session/include/drape/session/tee_scene.hpp`, `libs/session/src/tee_scene.cpp`
- Modify: `libs/CMakeLists.txt` (add `session`), `tools/drape_dump/main.cpp`, `tools/drape_dump/CMakeLists.txt`, `libs/sim/tests/tee_drape_helpers.hpp` (use the builder), `libs/sim/CMakeLists.txt` (tests depend on `drape_session`)
- Test: `libs/session/tests/test_tee_scene.cpp`

**Interfaces:**
- Produces (`namespace drape::session`):
  ```cpp
  struct TeeScene {                    // not movable once built: scene.body points at field
    avatar::BodyType bodyType;
    SimQuality quality;
    avatar::Mannequin mannequin;
    CollisionField field;
    Pattern pattern;
    SimMesh mesh;
    sim::SimScene scene;               // one garment "tee"
  };
  const garment::GarmentGenerator& teeGenerator(avatar::BodyType);   // men's on male, women's cut on female
  std::unique_ptr<const TeeScene> makeTeeScene(avatar::BodyType, SimQuality, const std::string& size = "M",
                                               const std::optional<std::filesystem::path>& mannequinDir = {});
  ```
  It uses the fabrics `{"body": testJersey, "rib": testRib}`, as `drape_dump` does today.

- [ ] **Step 1: Write the failing tests:**
  - `TeeScene.MaleDraft`: `particleCount` equals `mesh.rest.size()`, `scene.body == &field`, and `scene.garments[0].id == "tee"`.
  - `TeeScene.FemaleUsesWomensCut`: the pattern's body length equals `resolveSpec(WomensTestTeeGenerator().specTable(), "M").at("body_length")`.
  - `TeeScene.BadMannequinDirThrowsWithPath`: a nonexistent dir throws `std::runtime_error` whose message contains the absolute path.
- [ ] **Step 2: Run to verify they fail.**
- [ ] **Step 3: Implement**, then switch `drape_dump` and `expectTeeDrapes` to `makeTeeScene` (their behavior and output lines stay identical).
- [ ] **Step 4: Verify.** `ctest --preset dev -LE perf`. Expected: all pass. `DrapeDumpSmoke` prints the same summary line as before.
- [ ] **Step 5: Commit** (`feat(session): tee scene builder shared by drape_dump, tests and the app`).

---

### Task 4: Solver factory and backend-parameterized physics tests

**Files:**
- Create: `libs/sim/include/drape/sim/solver_factory.hpp`, `libs/sim/src/solver_factory.cpp`, `libs/sim/tests/backends.hpp`
- Modify: `libs/sim/tests/test_cpu_solver.cpp`, `test_cantilever.cpp`, `test_body_collision.cpp`, `test_drape_runner.cpp`
- Test: the modified files

**Interfaces:**
- Produces:
  ```cpp
  enum class SolverBackend { Cpu, GpuEmulated, Metal };
  std::string_view backendName(SolverBackend);         // "cpu" | "gpu-emulated" | "metal"
  bool isAvailable(SolverBackend);                     // Metal: Apple build with a device and the metallib
  std::unique_ptr<Solver> makeSolver(SolverBackend);   // unavailable -> std::runtime_error naming the backend and cause
  ```
  `backends.hpp`: `std::vector<SolverBackend> testBackends()` returns every available backend, and a gtest name generator built on `backendName`. Until Task 7, `GpuEmulated` reports unavailable.
- Parameterized (`TEST_P` over `testBackends()`), as physics tests 1, 2 and 4:
  - `StripStretchWarp`, `StripStretchWeft`;
  - the three `Cantilever*` tests;
  - `ParticlesRestOnSphere`, `SheetDrapesOverSphere`, `DrapedSheetComesToRest`, `FrictionHoldsOnGentleSlope`, `SlidesWhenSlippery`;
  - `HangingSheetSettlesSymmetric`.

  Their assertions are unchanged. All other tests stay CPU-only.

- [ ] **Step 1: Convert the listed tests to `TEST_P`.** Add `SolverFactory.CpuAlwaysAvailable` and `SolverFactory.UnavailableThrowsNamingBackend` (on Linux, `makeSolver(Metal)` throws with "metal" in the message).
- [ ] **Step 2: Run to verify they fail** (build error: factory missing).
- [ ] **Step 3: Implement the factory.** The `Metal` branch is compiled only `if(APPLE)`; elsewhere it throws "metal: not built on this platform".
- [ ] **Step 4: Verify.** `ctest --preset dev -LE perf`. Expected: all pass; the converted tests' names show a `/cpu` suffix.
- [ ] **Step 5: Commit** (`test(sim): physics tests 1, 2 and 4 run per solver backend`).

---

### Task 5: GPU compatibility layer and float term math

**Files:**
- Create: `libs/sim/kernels/gpu_compat.h`, `libs/sim/kernels/gpu_layout.h`, `libs/sim/kernels/term_math.h`
- Test: `libs/sim/tests/test_term_math.cpp`

**Interfaces:**
- `gpu_compat.h`:
  - Under `__METAL_VERSION__`, it maps to MSL (`float3`, `float4`, `metal::sqrt`, …) and defines `DRAPE_DEVICE` as `device` and `DRAPE_CONSTANT` as `constant`.
  - Otherwise it defines C++ structs `float2`, `float3`, `float4` (members `x, y, z[, w]`, the arithmetic operators MSL has), `uint`, and the free functions `dot, cross, length, length_squared, normalize, sqrt, fabs, fmax, fmin, clamp, atan2`. The two qualifier macros are empty.
  - Both: `struct Mat3 { float3 c0, c1, c2; }` with `mul(Mat3, float3)`, `add`, `scale`, `outer(float3, float3)`, `identity3()`; `float3 xyz(float4)`; `float4 make4(float3, float w)`.
- `gpu_layout.h` defines the buffer elements, laid out identically in both compilers (only `float`, `uint`, `float4`):
  - `GpuStretch { uint v[4]; float4 dmInv; float area; uint fabric; uint pad[2]; }`
  - `GpuHinge { uint v[4]; float K, restAngle; uint pad[2]; }`
  - `GpuSeam { uint a, b, garment; float startGap; }`
  - `GpuFabric` (the `FabricPhysical` fields the energies read, as `float`)
  - `GpuGarment { uint phase; float assemblyTime; uint pad[2]; }`
  - `Block3 { float4 c0, c1, c2; }`
  - `FieldInfo { float4 origin; float cellSize, band; uint dims[3]; uint pad; }`
  - `CgScalars { float rz, pAp, alpha, beta, r0, rr; uint converged, iteration; }`
  - `StepParams { float4 gravity; float h, invH2, alpha, limitScale, seamStiffness, closingSpeed, frictionSlip, cgTolerance, collisionOffset, dampingScale, assemblyExtraDamping, contactReach, contactSnap; uint n, nStretch, nHinge, nSeam, nBlocks, withDerivatives, maxCgIterations; }`

  The non-`h` fields are filled from `SimSettings` and the Task 1 constants. `static_assert` the sizes on the C++ side.
- `term_math.h`: float ports, line for line, of `evalStretchTerm`, `evalHingeTerm`, `evalSeamTerm`, `evalFrictionTerm` (into `TermEvalF<N> { float energy; float3 grad[N]; Mat3 hess[N][N]; }`), plus:
  - `float fieldSample(const DRAPE_DEVICE float*, FieldInfo, float3)` and `float3 fieldGradient(…)`, matching `CollisionField::sample`/`gradient` (trilinear; central differences with step `cellSize`; zero if |g| < 1e-9);
  - `collideBodyF`, `landOnBodyF`, matching `collision.hpp`;
  - `Mat3 invertSpd3(Mat3, thread bool& ok)` (LDLT; `ok = false` when a pivot ≤ 0).

  Term inputs are positions relative to the term's first vertex start (Task 7 builds them). Translation invariance makes this exact.

- [ ] **Step 1: Write the failing tests** (C++ build of the headers). For 200 random configurations each (fixed seed, edge lengths 2–20 mm, strains −20%…+30%, testJersey/testStiff/testAniso):
  - `TermMath.StretchMatchesDouble`, `HingeMatchesDouble`, `SeamMatchesDouble`, `FrictionMatchesDouble`: |E_f − E_d| ≤ 1e-4·|E_d| + 1e-12; gradient and Hessian ‖·‖_F errors ≤ 1e-3 relative to the double value's norm (plus 1e-9 absolute).
  - `TermMath.FieldMatchesCollisionField`: on the sphere field, at 1,000 points in and around the band, |sample| error ≤ 1e-6 m and gradient angle ≤ 1e-3 rad.
  - `TermMath.CollideMatchesDouble`: `collideBodyF` and `landOnBodyF` agree with `collideBody` / `landOnBody` within 1e-6 m.
  - `TermMath.InvertSpd3`: for random SPD matrices, `mul(invert(A), A·x) ≈ x` within 1e-4 relative; a matrix with a negative pivot returns `ok == false`.
  - `GpuLayout.Sizes`: `sizeof(GpuStretch) == 48`, `sizeof(Block3) == 48`, `sizeof(StepParams) == 96`, `sizeof(CgScalars) == 32`.
- [ ] **Step 2: Run to verify they fail.**
- [ ] **Step 3: Implement the three headers.** Port the double code expression by expression (same branch structure, same constants) so the Metal and C++ builds agree.
- [ ] **Step 4: Verify.** `ctest --preset dev -R "TermMath|GpuLayout"`. Expected: PASS.
- [ ] **Step 5: Commit** (`feat(sim): float term math and GPU layout shared by Metal and C++`).

---

### Task 6: Compute device interface, emulated device and kernels

**Files:**
- Create: `libs/sim/src/gpu/compute_device.hpp`, `libs/sim/src/gpu/emulated_device.hpp`, `libs/sim/src/gpu/emulated_device.cpp`, `libs/sim/kernels/kernels.h`
- Test: `libs/sim/tests/test_kernels.cpp`

**Interfaces:**
- `compute_device.hpp`:
  ```cpp
  enum class Kernel : std::uint32_t { Predict, ContactSetup, ContactActivate, InertiaFriction, StretchTerms, HingeTerms,
    SeamTerms, GatherGrad, GatherBlocks, ContactRelease, Precondition, CgInit, Spmv, CgAlpha, CgUpdate, CgCheck,
    CgPrecond, CgBeta, CgDirection, ReducePartial, ReduceFinal, ContactForce, TrialPositions, ApplyStep, FinishSubstep,
    Count };
  std::string_view kernelName(Kernel);   // "predict", "contact_setup", ... (Metal function names)
  struct BufferId { std::uint32_t index = 0; };
  class ComputeDevice {
   public:
    virtual ~ComputeDevice() = default;
    virtual BufferId create(std::size_t bytes) = 0;            // zero-filled, CPU-visible (shared)
    virtual void* contents(BufferId) = 0;
    virtual void begin() = 0;                                  // start recording
    virtual void dispatch(Kernel, std::uint32_t threads, std::span<const BufferId> buffers) = 0;  // in order, serial
    virtual void submitAndWait() = 0;
  };
  ```
- `kernels.h`: one function per `Kernel`, `void k_<name>(uint tid, …DRAPE_DEVICE buffers…)`. Each returns immediately when `tid` ≥ its count. Each kernel's argument order is the buffer order passed to `dispatch` (documented in a comment above each kernel). Reductions are barrier-free:
  - `ReducePartial`: thread i sums elements [256i, 256i+256) of a float array, in order, into `partials[i]`.
  - `ReduceFinal`: thread 0 sums `partials` in order into a named `CgScalars` field (selected by a uint parameter).
- `EmulatedDevice : ComputeDevice` stores buffers as `std::vector<std::byte>`. `dispatch` records the call; `submitAndWait` runs each recorded call as `for tid in [0, threads)` over the `k_*` function.

- [ ] **Step 1: Write the failing tests:**
  - `Kernels.ReduceIsDeterministicAndAccurate`: sums 100,003 floats twice to bit-identical results, within 1e-5 relative of a double sum.
  - `Kernels.SpmvMatchesDense`: on a random 40-particle block-CSR SPD matrix with 3 pinned particles, `Spmv` matches the dense product (pinned rows and columns zeroed) within 1e-5 relative.
  - `Kernels.CgSolvesSpdSystem`: running `CgInit` then the CG kernels for 50 iterations solves that system to relative residual ≤ 1e-2, with the converged flag set and `iteration` ≤ 50.
  - `Kernels.ContactFilterRemovesNormal`: after `CgUpdate` with an active particle, `dot(step, n)` is 0 within 1e-7.
- [ ] **Step 2: Run to verify they fail.**
- [ ] **Step 3: Implement** `kernels.h` and `EmulatedDevice`. The CG kernels follow `CpuSolver::solveLinear` exactly:
  - check `pAp ≤ 0` before the update;
  - check the residual after the update;
  - once `converged == 1`, every per-particle CG kernel is a no-op.
- [ ] **Step 4: Verify.** `ctest --preset dev -R Kernels`. Expected: PASS.
- [ ] **Step 5: Commit** (`feat(sim): compute device interface, emulated device and solver kernels`).

---

### Task 7: `GpuSolverCore`, emulated GPU solver, parity in the cloud

**Files:**
- Create: `libs/sim/src/gpu/gpu_solver_core.hpp`, `libs/sim/src/gpu/gpu_solver_core.cpp`
- Modify: `libs/sim/src/solver_factory.cpp` (`GpuEmulated` becomes available), `libs/sim/tests/backends.hpp`
- Test: `libs/sim/tests/test_gpu_parity.cpp` (short runs), `libs/sim/tests/test_gpu_parity_slow.cpp` (label `slow`)

**Interfaces:**
- Consumes: `SimModel` (Task 1), `ComputeDevice`/`Kernel` (Task 6), layouts (Task 5).
- Produces: `class GpuSolverCore final : public Solver { explicit GpuSolverCore(std::unique_ptr<ComputeDevice>); … }`. The device holds `x`, `v` and `contactForce`. `syncToModel()` copies them into `SimModel` before `stats()`, `snapshot()` and `readPositions()`. `syncFromModel()` copies them back after `build`, `restore`, `setPhase` and `updateFabric`. `makeSolver(GpuEmulated)` returns `GpuSolverCore` over `EmulatedDevice`.
- Algorithm per substep (`h = dt / substeps`). This mirrors `CpuSolver::Impl::substep`; the numbers are the CPU's:
  1. `Predict`: `xStart = x`; `yRel = h·v + h²·(gravity if Settling + fExt/m)`; `u = h·v` (pinned: `yRel = u = 0`).
  2. `ContactSetup`: sample the field at `xStart`. If `phi < radius + kContactReach` and the gradient is non-zero, set near, normal and `offset = radius − phi`, and `frictionLambda = previous contactForce`. Always set `contactForce = 0` and `active = 0`.
  3. Newton, `it < iterations`:
     - a. `ContactActivate`: `c = offset − n·u`; `c > −kContactSnap` → active; active and `c > 0` → `u += c·n`.
     - b. Term kernels with `withDerivatives = 1` evaluate at `p_k = (xStart_k − xStart_v0) + u_k` (inertia uses `u − yRel`; friction uses `u`). Then `GatherGrad` (pinned rows zeroed), `GatherBlocks`, `ContactRelease` (`grad·n < 0` → inactive). Submit.
     - c. CPU: `e0 = Σ termEnergy` in double, fixed order.
     - d. `Precondition`, `CgInit` and the reductions for `rz` and `r0`, then CG in chunks of 10 iterations per submission until `converged` or 50 iterations. Then `ContactForce`.
     - e. CPU: `slope = Σ grad·step` in double; stop Newton if `!(slope < 0)`.
     - f. Line search, up to 20 halvings from `alpha = 1`: `TrialPositions` plus the term kernels with `withDerivatives = 0`, submit, CPU sum; accept when `E ≤ e0 + 1e-4·alpha·slope`.
     - g. `ApplyStep`: `u += alpha·step`; `frictionLambda = active ? contactForce : 0`; per-particle max-abs move into `partials`. CPU takes the max; stop if `< 1e-9`.
  4. `FinishSubstep`:
     - `x = xStart + u`;
     - for unpinned particles with `fieldSample(x) < radius − kContactSnap`: `landOnBodyF` when the gradient is zero, else `collideBodyF`;
     - pins;
     - `v = (x − xStart)/h · max(0, 1 − c·h)`, where `c = dampingScale·fabric.damping (+ assemblyExtraDamping when Assembling)`.

     The CPU then advances `assemblyTime` for assembling garments and rewrites the garments buffer.

- [ ] **Step 1: Write the failing tests.**
  - `test_gpu_parity.cpp`:
    - `GpuParity.HangingSheet60Frames`: the hanging-sheet scene from `test_drape_runner.cpp`, CPU vs `GpuEmulated`, 60 frames at `{1/60, 1, 2}`; max per-particle distance < 0.001 m.
    - `GpuParity.SheetOnSphere60Frames`: `centredSheet(0.3, 0.01, 0.25)` over the sphere field; same bound.
    - `GpuParity.TeeFirst60Frames`: `makeTeeScene(Male, Draft)` through `DrapeSession`, 60 advances each; same bound.
    - `GpuDeterminism.TwoRunsBitIdentical` and `GpuDeterminism.SnapshotReplayBitIdentical` (the `CpuSolver.SnapshotRestoreIsDeterministic` pattern) on `GpuEmulated`.
  - `test_gpu_parity_slow.cpp`, `GpuParity.TeeSettledMale` and `GpuParity.TeeSettledFemale`: both backends drape to rest at Draft; both settle; mean hem height (the hem vertices `expectTeeDrapes` uses) and the garment's center of mass agree within 0.002 m.
  - The tee drape checks on the GPU (spec 7.2): `expectTeeDrapes` gains a trailing `sim::SolverBackend backend = Cpu` parameter. `test_tee_drape_slow.cpp` adds `TeeDrape.GpuMaleM` and `TeeDrape.GpuFemaleM` (size M, Draft), run on every available non-CPU backend in `testBackends()`.
- [ ] **Step 2: Run to verify they fail** (`GpuEmulated` unavailable).
- [ ] **Step 3: Implement `GpuSolverCore`.** `build()` uploads `SimModel` and builds the gather lists from the slot tables, in fixed (term, local index) order.
- [ ] **Step 4: Verify.** `ctest --preset dev -LE perf`. Expected: all pass, including the parameterized physics tests' `/gpu-emulated` variants and the slow parity tests. If a parity or physics test fails on precision, apply Global Constraints' precision rule and record which quantity moved to 64-bit.
- [ ] **Step 5: Commit** (`feat(sim): GPU solver core with an emulated device; CPU parity in the cloud`).

---

### Task 8: Portable render math

**Files:**
- Create: `libs/render/CMakeLists.txt`; `libs/render/include/drape/render/normals.hpp`, `orbit_camera.hpp`, `image_metrics.hpp`; their `src/*.cpp`
- Modify: `libs/CMakeLists.txt`
- Test: `libs/render/tests/test_render_math.cpp`

**Interfaces (portable target `drape_render_math`, depends on `drape_core`):**
- `std::vector<Vec3f> vertexNormals(std::span<const Vec3f> positions, std::span<const std::array<std::uint32_t,3>> triangles, std::span<const std::pair<std::uint32_t,std::uint32_t>> weldPairs);` Area-weighted (unnormalized cross products summed), each weld pair given the normalized sum of both normals, then normalized. A zero-area vertex gets `(0,1,0)`.
- `class OrbitCamera`:
  - members `target`, `distance`, `yaw`, `pitch`;
  - `void frame(const Vec3d& boundsMin, const Vec3d& boundsMax, double verticalFovRad, double aspect)` fits the box with 10% margin, facing +Z toward −Z;
  - `void orbit(double dxPixels, double dyPixels)` at 0.005 rad/px, pitch clamped to ±1.5 rad;
  - `void pan(double dx, double dy, double viewportHeightPx, double verticalFovRad)`;
  - `void zoom(double steps)`: `distance *= pow(0.9, steps)`, clamped to [0.2, 20] m;
  - `Vec3d eye() const`, `Vec3d up() const`.
- `double coverage(std::span<const std::uint8_t> rgba, std::uint32_t w, std::uint32_t h, const std::function<bool(std::uint8_t r, std::uint8_t g, std::uint8_t b)>& pred);` returns the fraction of pixels matching.
- `bool isBlueTint(r, g, b)` is `b > r + 30 && b > g + 10`.

- [ ] **Step 1: Write the failing tests:**
  - `Normals.FlatSheetPointsUp`: every normal is `(0,1,0)` ±1e-6.
  - `Normals.WeldPairsShareNormal`: two quads meeting at 90° with duplicated edge vertices welded; the welded vertices get identical normals, at 45° from both faces.
  - `OrbitCamera.FrameContainsBox`: all 8 box corners project inside the frustum.
  - `OrbitCamera.PitchClamped`: a large orbit gives |pitch| = 1.5.
  - `OrbitCamera.ZoomClamped`: zoom(1000) gives distance 0.2.
  - `ImageMetrics.Coverage`: a 10×10 image with 25 blue pixels gives 0.25.
- [ ] **Step 2: Run to verify they fail.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Verify.** `ctest --preset dev -R "Normals|OrbitCamera|ImageMetrics"`. Expected: PASS.
- [ ] **Step 5: Commit** (`feat(render): portable normals, orbit camera and image metrics`).

---

### Task 9: `SimController` — the sim thread, commands and position exchange

**Files:**
- Create: `libs/session/include/drape/session/position_exchange.hpp`, `libs/session/include/drape/session/sim_controller.hpp`, `libs/session/src/sim_controller.cpp`
- Test: `libs/session/tests/test_sim_controller.cpp`

**Interfaces:**
- Consumes: `makeTeeScene` (Task 3), `DrapeSession` (Task 2), `makeSolver`/`isAvailable` (Task 4).
- Produces:
  ```cpp
  struct PublishedFrame {
    std::uint64_t revision = 0;                    // bumps on every rebuild (new mesh)
    std::uint64_t frame = 0;                       // bumps on every published frame
    std::shared_ptr<const TeeScene> scene;         // mesh, body, pattern for this revision
    std::vector<Vec3f> positions;                  // size == scene->mesh.rest.size()
  };
  class PositionExchange {                         // double buffer, mutex-guarded swap
   public:
    void publish(PublishedFrame&& f);
    bool latest(PublishedFrame& out, std::uint64_t sinceFrame) const;   // false if nothing newer
  };
  struct SimConfig { avatar::BodyType body = avatar::BodyType::Male; SimQuality quality = SimQuality::Standard;
                     sim::SolverBackend backend = sim::SolverBackend::Metal; std::string size = "M";
                     std::optional<std::filesystem::path> mannequinDir; };
  struct SimStatus { sim::DrapeState state; bool running; double fps; std::size_t particles;
                     std::string message; sim::SolverBackend backend; bool gpuAvailable; };
  class SimController {
   public:
    explicit SimController(SimConfig, std::function<std::unique_ptr<sim::Solver>(sim::SolverBackend)> factory = sim::makeSolver);
    ~SimController();                              // stops and joins the thread
    void setRunning(bool);                         // Simulate play/pause
    void reset();                                  // rebuild same config
    void configure(SimConfig);                     // Quality/Body/Solver change -> rebuild
    SimStatus status() const;
    const PositionExchange& exchange() const;
  };
  ```
  - The thread takes the newest pending command (later commands supersede earlier ones). It rebuilds through `makeTeeScene` and the factory, and calls `DrapeSession::advance()` while running and not finished. After each frame it publishes positions, then sleeps on a condition variable once finished or paused.
  - If the requested backend is unavailable, it uses `Cpu`, sets `gpuAvailable = false`, and puts "GPU unavailable: <reason>; using CPU" in `message`.
  - On `Failed`, it sets `message` to `kDrapeFailureMessage`.
  - `fps` is frames per second of wall time over the last second.

- [ ] **Step 1: Write the failing tests** (CPU backend, male, Draft):
  - `PositionExchange.LatestSinceFrame`.
  - `SimController.ReachesSettled`: within 120 s wall time; the last published positions' size equals the mesh's.
  - `SimController.PauseStopsFrames`: frame count unchanged over 0.5 s while paused.
  - `SimController.RapidRebuildsNeverMixRevisions`: 20 alternating `configure` Body/Quality calls 5 ms apart. Every frame read in a polling loop has `positions.size() == scene->mesh.rest.size()`; the final revision's config equals the last request.
  - `SimController.DestroyWhileAssemblingJoinsQuickly`: the destructor returns within 1 s.
  - `SimController.FailureStopsAndReportsMessage`: a factory returning a solver whose `stats()` is always non-finite. The state becomes `Failed`, `message == kDrapeFailureMessage`, and frames stop.
  - `SimController.FallsBackToCpuWhenGpuUnavailable`: on Linux, a `Metal` config runs on `Cpu` with `gpuAvailable == false` and a message starting "GPU unavailable".
- [ ] **Step 2: Run to verify they fail.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Verify.** `ctest --preset dev -R "PositionExchange|SimController"`, then the full `-LE perf` suite. Expected: PASS.
- [ ] **Step 5: Commit** (`feat(session): sim controller thread with position exchange`). Then push `phase-b` to `cache25/drape-2` (Ian approved this in the hand-off).

---

### Task 10: Mac toolchain, presets and scripts

**Files:**
- Create: `scripts/build-mac.sh`, `scripts/test-mac.sh`, `cmake/DrapeMacDependencies.cmake`
- Modify: `CMakePresets.json` (`mac`, `mac-release`), `CMakeLists.txt` (`if(APPLE) include(DrapeMacDependencies)`), `README.md` (Mac setup)

**Interfaces:**
- Presets `mac` (Debug `-g -O2`, `build/mac`) and `mac-release` (`build/mac-release`), both with `CMAKE_OSX_DEPLOYMENT_TARGET=14.0`, `CMAKE_OSX_ARCHITECTURES=arm64` and `CMAKE_PREFIX_PATH=/opt/homebrew/opt/qt;/opt/homebrew/opt/eigen@3`.
- `build-mac.sh [--release]`: configure and build; exits non-zero on failure.
- `test-mac.sh [--release]`: `ctest --preset mac -LE perf` (or `mac-release -L perf` with `--release`), then the app self-test (Task 13). Prints a final `ALL PASSED` or the failing step.
- `DrapeMacDependencies.cmake` downloads into `${CMAKE_BINARY_DIR}/_deps`, SHA-256 checked:
  - metal-cpp (FetchContent, pinned commit);
  - Filament tarball, extracted (`file(DOWNLOAD … EXPECTED_HASH SHA256=…)`);
  - HDRI.

  The two hashes are recorded on the first download (Step 2) and committed.

- [ ] **Step 1: On the Mac:** `brew install qt` (only if `brew list qt` is empty). Check the result: `brew info qt` shows 6.12 installed.
- [ ] **Step 2: Record hashes.** Download `filament-v1.78.1-mac.tgz` and `studio_small_09_2k.hdr` once into `build/_downloads`, run `shasum -a 256`, and write both into `DrapeMacDependencies.cmake`.
- [ ] **Step 3: Write the presets, scripts and dependency file.** `find_package(Qt6 6.12 REQUIRED COMPONENTS Widgets)` lives inside `if(APPLE)`.
- [ ] **Step 4: Verify.** `scripts/build-mac.sh && ctest --preset mac -LE "perf|gpu"`. Expected: every portable test passes on the Mac, including Tasks 1–9. Also check that Linux `ctest --preset dev -LE perf` still passes.
- [ ] **Step 5: Commit** (`build: Mac presets, scripts and pinned Mac dependencies`).

---

### Task 11: `MetalDevice` and `MetalSolver`

**Files:**
- Create: `libs/sim/src/gpu/metal_device.hpp`, `libs/sim/src/gpu/metal_device.cpp`, `libs/sim/kernels/drape_kernels.metal`, `libs/sim/include/drape/sim/metal_solver.hpp`, `libs/sim/src/gpu/metal_solver.cpp`
- Modify: `libs/sim/CMakeLists.txt` (metallib custom command; Apple-only sources), `libs/sim/src/solver_factory.cpp`, `libs/sim/tests/backends.hpp`
- Test: `libs/sim/tests/test_metal.cpp` (label `gpu`); the parameterized and parity tests gain `/metal`

**Interfaces:**
- `class MetalSolver final : public Solver { explicit MetalSolver(const std::filesystem::path& metallib = defaultMetallibPath()); … }` wraps `GpuSolverCore` over `MetalDevice`.
  - `defaultMetallibPath()`: next to the executable (`drape_kernels.metallib`); otherwise `<exe>/../Resources/drape_kernels.metallib`; otherwise the build-time `DRAPE_METALLIB_PATH`.
  - No device → `std::runtime_error("metal: no Metal device")`. Library load failure → `std::runtime_error("metal: cannot load <absolute path>: <NSError description>")`.
- `MetalDevice : ComputeDevice` holds one `MTL::Device`, one `CommandQueue`, and one `ComputePipelineState` per `Kernel` (function name = `kernelName`). Buffers use `MTL::ResourceStorageModeShared`. `begin()` opens a command buffer with one serial compute encoder; `dispatch` sets buffers 0…k and calls `dispatchThreads(threads, 64)`. `submitAndWait()` ends encoding, commits and waits. A command buffer error throws `std::runtime_error` naming the error.
- `drape_kernels.metal` has one `[[kernel]] void <kernelName>(uint tid [[thread_position_in_grid]], …buffers…)` wrapper per `Kernel`, calling `k_<name>` from `kernels.h`.
- Build: `xcrun -sdk macosx metal -std=metal3.1 -fno-fast-math -I<kernels> -c drape_kernels.metal -o drape_kernels.air`, then `xcrun -sdk macosx metallib`. Output goes to `${CMAKE_BINARY_DIR}/drape_kernels.metallib`, and is copied next to the test binaries and into the app bundle (Task 13).

- [ ] **Step 1: Write the failing tests** (`gpu` label):
  - `MetalSolver.MissingMetallibThrowsWithPath`: `MetalSolver("/nonexistent/x.metallib")` throws with the absolute path in the message.
  - `MetalSolver.MatchesEmulatedOneFrame`: the hanging sheet, one frame, Metal vs `GpuEmulated`; max difference < 1e-5 m.
  - `isAvailable(Metal)` is true on the Mac, so every `TEST_P` and parity test gains a `/metal` variant. Extend the parity suites to compare CPU vs Metal with the same bounds as Task 7, and the determinism tests to run on Metal.
- [ ] **Step 2: Run to verify they fail.** `scripts/build-mac.sh`. Expected: build fails (`metal_solver.hpp` missing).
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Verify.** `ctest --preset mac -LE perf`. Expected: every test passes, including `/metal` physics tests 1, 2 and 4, the CPU–Metal parity tests (short < 1 mm; long hem and COM within 2 mm), the Metal determinism tests, and `MetalSolver.*`.
- [ ] **Step 5: Commit** (`feat(sim): MetalSolver on Apple GPUs`).

---

### Task 12: Filament renderer

**Files:**
- Create: `libs/render/include/drape/render/renderer.hpp`, `libs/render/src/renderer.cpp`, `libs/render/src/metal_layer.mm`, `libs/render/materials/mannequin.mat`, `libs/render/materials/cloth.mat`
- Modify: `libs/render/CMakeLists.txt` (Apple target `drape_render`, `matc` and `cmgen` custom commands), `cmake/DrapeMacDependencies.cmake` (Filament imported targets)
- Test: `libs/render/tests/test_renderer.cpp` (label `gpu`)

**Interfaces:**
- ```cpp
  struct RendererAssets { std::filesystem::path materialsDir, iblDir; };   // .filamat and ibl/skybox .ktx
  struct RgbaImage { std::uint32_t width = 0, height = 0; std::vector<std::uint8_t> pixels; };
  class Renderer {
   public:
    explicit Renderer(const RendererAssets&);          // throws std::runtime_error("filament: <cause>")
    ~Renderer();
    void setBody(const TriangleMesh& body);
    void setGarment(std::span<const std::array<std::uint32_t,3>> triangles,
                    std::span<const std::pair<std::uint32_t,std::uint32_t>> weldPairs, std::size_t vertexCount);
    void updateGarment(std::span<const Vec3f> positions);   // recomputes normals (Task 8) and uploads
    OrbitCamera& camera();
    void attachLayer(void* caMetalLayer);               // swap chain from a CAMetalLayer
    void resize(std::uint32_t widthPx, std::uint32_t heightPx);
    void renderFrame();
    RgbaImage renderOffscreen(std::uint32_t width, std::uint32_t height);
  };
  ```
- Materials, compiled by `matc -a metal -p desktop`:
  - `mannequin.mat`: lit, base color (0.92, 0.91, 0.89), roughness 0.45, clear coat 0.3.
  - `cloth.mat`: `shadingModel: cloth`, `doubleSided: true`, parameters `baseColor` and `sheenColor`. Back faces multiply base color by 0.6 (fixed inside-face amount).

  The tee color is (0.16, 0.38, 0.85), the preview blue.
- Lighting: `cmgen --format=ktx --size=256 --deploy=<iblDir> studio_small_09_2k.hdr`, giving the IBL at intensity 30,000; plus a directional key light from (−0.4, −1, −0.6), with soft shadows (PCSS) on.
- `metal_layer.mm`: `void* makeMetalLayerForView(void* nsView)` installs a `CAMetalLayer` as the view's layer; `contentsScale` follows the window's backing scale.

- [ ] **Step 1: Write the failing tests** (`gpu` label; the scene comes from `makeTeeScene(Male, Draft)` draped to rest on CPU):
  - `Renderer.OffscreenTeeCoverage`: at 800×1000, `coverage(isBlueTint)` is within [0.05, 0.40], and the image is not a single color.
  - `Renderer.OffscreenRendersAtTwoSizes`: 400×500 then 1200×1500; both are non-blank and their coverage differs by < 0.02.
  - `Renderer.MissingMaterialThrows`: an empty `materialsDir` throws with "filament" in the message.
  - Save the 800×1000 image to `build/mac/render_smoke.png` for Claude to view.
- [ ] **Step 2: Run to verify they fail.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Verify.** `ctest --preset mac -R Renderer`. Expected: PASS. Open `render_smoke.png` with the Read tool and confirm a white mannequin, a blue tee, and soft shading with no black or inside-out faces.
- [ ] **Step 5: Commit** (`feat(render): Filament renderer with cloth shading and offscreen capture`).

---

### Task 13: The Drape app

**Files:**
- Create: `apps/drape/CMakeLists.txt`, `apps/drape/main.cpp`, `apps/drape/main_window.hpp/.cpp`, `apps/drape/viewport_widget.hpp/.mm`, `apps/drape/self_test.hpp/.cpp`
- Modify: root `CMakeLists.txt` (`if(APPLE) add_subdirectory(apps/drape)`), `scripts/test-mac.sh`

**Interfaces:**
- Consumes: `SimController` (Task 9), `Renderer` (Task 12), `MetalSolver` / `makeSolver` (Task 11).
- `Drape.app` bundle: `Contents/Resources/` holds `mannequins/`, `materials/*.filamat`, `ibl/*.ktx` and `drape_kernels.metallib`. Qt is linked dynamically from Homebrew during development (bundling and `macdeployqt` are Phase F).
- `MainWindow`:
  - the viewport as the central widget;
  - toolbar actions: Simulate (checkable, on at launch), Reset drape, Quality combo (Draft/Standard/Fine, default Standard), Body combo (Male/Female), Solver combo (GPU/CPU, default GPU; the GPU item is disabled when `!status().gpuAvailable`);
  - status bar labels, refreshed by a 10 Hz `QTimer` from `SimController::status()`:
    - phase: "Assembling" / "Settling" / "Settled" / "Failed" / "Stopped" (for `TimedOut`);
    - "NN fps";
    - "N particles";
    - the message.
- `ViewportWidget`:
  - sets `WA_NativeWindow`, `WA_PaintOnScreen` and `WA_NoSystemBackground`, and gets the `NSView` via `winId()`;
  - renders on a 16 ms `QTimer`. Each tick it pulls `exchange().latest(...)`. On a new `revision` it calls `setBody` and `setGarment`, then `camera().frame` on body bounds; on a new frame, `updateGarment`. Then `renderFrame()`.
  - Mouse: left-drag orbit, right-drag pan, wheel and pinch zoom. `resizeEvent` sends device pixels (`width() * devicePixelRatioF()`).
- Errors (spec section 6):
  - `Renderer` or Qt failure: `QMessageBox::critical`, naming the component, then exit code 1;
  - a missing asset: the loader message in a modal, then exit code 1;
  - Metal unavailable: handled by `SimController`'s fallback.
- `--selftest <out.png>`:
  - Start: male, Standard, GPU. Print `launch_ms=<ms>` when the first frame is rendered.
  - At 1 s, resize the window to 640×480; at 2 s, back to 1280×800.
  - Wait for `Settled` (or `Failed` / `TimedOut`, 180 s timeout).
  - Measure frame rate over 2 s while asleep and print `fps_asleep=<n>`.
  - Save `renderOffscreen(1280, 800)` to `<out.png>`.
  - Print `settled=<0|1> backend=<name> seconds=<sim seconds>`; exit 0 only if settled.

- [ ] **Step 1: Write the self-test check into `test-mac.sh`:**
  - run `build/mac/apps/drape/Drape.app/Contents/MacOS/Drape --selftest build/mac/selftest.png`;
  - require exit 0 and `backend=metal`;
  - require `launch_ms` < 3000 and `fps_asleep` ≥ 60 (spec 7.3 gates);
  - require `coverage(isBlueTint)` of the PNG in [0.025, 0.20] *(as built: the 1280×800 landscape window shows the framed body over twice the area of the spec's 800×1000 render, so the spec band is halved; measured 0.041)*, via a tiny `drape_image_check` CLI built with `drape_render_math`.
- [ ] **Step 2: Run to verify it fails** (no app target).
- [ ] **Step 3: Implement the app.**
- [ ] **Step 4: Verify.** `scripts/build-mac.sh && scripts/test-mac.sh`. Expected: `ALL PASSED`. Open `selftest.png` with the Read tool: the tee is draped on the male, with no spikes and no body showing through. Send it to Ian.
- [ ] **Step 5: Commit** (`feat(app): Drape Mac app with live viewport and toolbar`).

---

### Task 14: Performance gates, benchmarks, docs and hand-off

**Files:**
- Create: `libs/sim/tests/test_tee_benchmark.cpp` (label `perf`)
- Modify: `docs/superpowers/specs/2026-10-01-drape-milestone-1-design.md` (3.4, 13.1, 17, Status line), `docs/superpowers/specs/2026-10-25-drape-phase-b-mac-app-design.md` (Status: approved, and the as-built notes), `README.md`

- [ ] **Step 1: Write the benchmark.** `TeeBenchmark.GpuFasterThanCpuAtStandard` drapes `makeTeeScene(Male, Standard)` to rest on `Cpu` and `Metal`, measuring wall time for each. It asserts `metal < cpu`. It prints one line per backend and quality (Draft, Standard, Fine): `bench backend=<b> quality=<q> wall_s=<t> sim_s=<s> frames=<n> particles=<p>`.
- [ ] **Step 2: Run.** `scripts/build-mac.sh --release && scripts/test-mac.sh --release`. Expected: PASS. Copy the six `bench` lines, plus `launch_ms` and `fps_asleep` from the self-test, into the Phase B spec under a new "As built" note.
- [ ] **Step 3: Spec amendments.** Apply Phase B spec Section 10 to the milestone spec; mark the Phase B spec "Approved 2026-10-25".
- [ ] **Step 4: README.** Add a "Mac app" section: `brew install cmake ninja eigen@3 qt`, `scripts/build-mac.sh`, `scripts/test-mac.sh`, and how to open `Drape.app`.
- [ ] **Step 5: Full verification.**
  - Linux: `ctest --preset dev -LE perf`.
  - Mac: `scripts/test-mac.sh && scripts/test-mac.sh --release`.

  Expected: all pass on both.
- [ ] **Step 6: Commit** (`docs: Phase B as built; milestone spec amendments; README`). Push `phase-b`. Tell Ian to open `Drape.app` from `Dev 2/drape/build/mac/apps/drape/`, and to merge `phase-b` into `main` when he's happy.
