# Build performance flags — what's enabled and how to use it

Companion to [`PERF_FLAGS_ANALYSIS.md`](PERF_FLAGS_ANALYSIS.md). That document analyses the options;
this one records what is actually wired into the build and how to drive the opt-in pieces (PGO).

## What is enabled, and where

| Optimization | Plugin (`hdtSMP64.dll`) | Dependencies (Bullet, TBB, …) |
|---|---|---|
| `/O2` (maximize speed) | CMake default (Release) | CMake default (Release) |
| `/Ob3` (most aggressive inlining) | `src/CMakeLists.txt` | `cmake/triplets/_perf_flags.cmake` |
| `/Gw` (global-data COMDATs) | `src/CMakeLists.txt` | `cmake/triplets/_perf_flags.cmake` |
| `/fp:fast` (FMA + FP reassociation) | `src/CMakeLists.txt` | `cmake/triplets/_perf_flags.cmake` |
| `/GL` + `/LTCG` (whole-program opt) | `INTERPROCEDURAL_OPTIMIZATION` in `src/CMakeLists.txt` | `cmake/triplets/_perf_flags.cmake` |
| `/sdl` dropped in Release (kept in Debug) | `src/CMakeLists.txt` | n/a |
| `/arch:AVX*` SIMD tiers | `CMakePresets.json` per preset | `cmake/triplets/x64-windows-static-md*.cmake` |
| PGO (opt-in) | `FSMP_PGO` option in `src/CMakeLists.txt` | profiled transitively via the DLL's LTCG |

All of the above are applied for the **Release** (and release-like **Profile**) configurations only.
Debug builds keep `/sdl`, keep precise FP, and are not LTCG'd, so they stay debuggable.

These flags take effect identically for **local builds** and **GitHub CI** because they live in
`src/CMakeLists.txt` and the vcpkg triplets, which both paths use. The CI vcpkg cache key in
`.github/workflows/build.yml` hashes `cmake/triplets/**`, so editing the dependency flags forces CI
to rebuild the dependencies with the new flags instead of restoring a stale cache.

### Why the dependency flags live in the triplet

Bullet is statically linked into the DLL and runs the heavy collision/constraint math every frame.
The only way to change how vcpkg compiles it is through `VCPKG_*_FLAGS` in the triplet — see
`cmake/triplets/_perf_flags.cmake`, included by all four SIMD triplets. Building the deps with `/GL`
lets the plugin's `/LTCG` link fold Bullet into the same whole-program optimization pass as our own
code.

### Toolset alignment (required by dependency LTCG)

`/GL` + `/LTCG` `.lib`/`.obj` files are tied to the exact MSVC toolset version. The plugin and the
dependencies must therefore be built with the **same** compiler. `cmake/triplets/_pin_toolset.cmake`
pins the vcpkg dependency compiler to MSVC 14.44; the `v143` toolset selected by the CMake presets on
Visual Studio 2022 resolves to that same 14.44 build, so they match. If the dependency toolset is ever
re-pinned, rebuild the deps (the CI cache key change above handles this automatically).

## Profile-Guided Optimization (opt-in)

PGO is the single largest realistic win for the hot paths, on top of LTCG. It is **not** part of the
default build because the instrumentation must be exercised by a real, representative gameplay scene —
something GitHub CI cannot do (it has no game, GPU, or save data). The default CI/local Release build
therefore ships LTCG (not full PGO); PGO is a deliberate, local, release-grade step.

Because Bullet is compiled with `/GL` and folded into the DLL's LTCG, a single PGO pass over
`hdtSMP64.dll` also profiles the heavy Bullet math — no separate dependency PGO is needed.

### Workflow

Pick a **fixed, repeatable, physics-heavy scene** (fixed save, camera, actor count with cloth/hair) —
the same one used for benchmarking. A garbage profile produces a garbage result.

1. **Instrument.** Build the instrumented DLL (adds `/GENPROFILE`):

   ```pwsh
   cmake --preset vs2022-windows-avx2-pgo-instrument
   cmake --build --preset avx2-pgo-instrument
   ```

2. **Profile.** Install the instrumented `hdtSMP64.dll`, launch Skyrim, load the standard scene, let
   it run for a representative period, then quit normally. On a clean process exit the instrumented
   build writes `hdtSMP64!N.pgc` files next to the linker output (`out/build/pgo-avx2/...`).
   Repeat for more coverage if desired (each run adds a `.pgc`).

3. **Optimize.** Re-link using the collected profile (adds `/USEPROFILE`, which merges the available
   `.pgc` data into `hdtSMP64.pgd`). This reuses the same build directory so the profile is found:

   ```pwsh
   cmake --preset vs2022-windows-avx2-pgo-optimize
   cmake --build --preset avx2-pgo-optimize
   ```

   The DLL in `out/build/pgo-avx2/...` is now the PGO-optimized build. A/B it against the LTCG-only
   build on the same scene before shipping.

`FSMP_PGO` can also be set directly (`-DFSMP_PGO=INSTRUMENT` / `OPTIMIZE`) on any MSVC configure; the
presets above just wire the two stages to a shared build directory for the avx2 (recommended) tier.

## Validation gate for `/fp:fast`

`/fp:fast` changes floating-point results (it is not bit-reproducible and enables FMA contraction).
For non-networked, non-deterministic cloth/hair physics this is acceptable, but it must be **visually
validated**: run the standard scene and watch for jitter, spring "explosions", NaN-driven disappearing
meshes, or other instability. If anything regresses, removing `/fp:fast` from `src/CMakeLists.txt` and
`cmake/triplets/_perf_flags.cmake` is the rollback.
