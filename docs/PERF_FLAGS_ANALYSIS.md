# Compiler & Linker Options — Performance Analysis

Scope: the options available to **us** (MSVC `cl.exe` compiler + `link.exe` linker, driven
through CMake/CMakePresets/vcpkg) and how each one affects the **runtime performance of the
shipped `hdtSMP64.dll`**, not build time or developer ergonomics.

## 0. What we are actually optimizing

`hdtSMP64.dll` is a SKSE plugin that runs **every rendered frame**: it drives a Bullet-based
skinned-mesh physics simulation (cloth/hair/body collisions) across multiple threads (TBB). So
"performance of the final DLL" means **per-frame physics step time** — dominated by:

1. The hot loops in `src/hdtSkinnedMesh/*` (vertex transforms, collision narrow-phase, constraint
   solving), and
2. **Bullet**, which is statically linked into the DLL from vcpkg.

Two consequences that shape everything below:

- **The DLL is distributed to many users with different CPUs.** "Maximum performance" on the
  developer's machine (AVX-512, `-march=native`) is not the same as "maximum performance we can
  *ship*". This is why the project already builds four SIMD tiers (`noavx`, `avx`, `avx2`,
  `avx512`) and tags the DLL accordingly.
- **Bullet is part of the DLL.** An optimization that only touches our `.cpp` files leaves a large
  fraction of the hot code (Bullet) untouched. Flags that reach Bullet do so through the vcpkg
  **triplet** (`cmake/triplets/x64-windows-static-md*.cmake` → `VCPKG_CXX_FLAGS`), not through
  `src/CMakeLists.txt`.

## 1. Current Release configuration (baseline)

| Layer | Flags in effect for `Release` |
|---|---|
| CMake default (`CMAKE_CXX_FLAGS_RELEASE`) | `/O2 /Ob2 /DNDEBUG` |
| Preset (`CMAKE_CXX_FLAGS`, avx2) | `/EHsc /MP /W4 /arch:AVX2` |
| `src/CMakeLists.txt` compile (Release/Profile) | `/Zc:inline /JMC- /Ob3` (+ `/sdl /utf-8 /Zi /permissive- /Zc:preprocessor /bigobj` for all configs) |
| `src/CMakeLists.txt` link (Release/Profile) | `/INCREMENTAL:NO /OPT:REF /OPT:ICF /DEBUG:FULL` |
| CRT | `/MD` (`MultiThreadedDLL`) |
| IPO/LTCG on `hdtSMP64` target | **none** |
| IPO/LTCG on `CommonLibSSE` static lib | enabled in Release (`CMAKE_INTERPROCEDURAL_OPTIMIZATION`) |
| Deps (Bullet/TBB via vcpkg) | triplet `/arch:AVX*`, default `/O2`, **no LTCG** |

> Note: `/Ob3` appears after the default `/Ob2`, so it wins → most aggressive inlining is active.
> `/O2` already implies a bundle: `/Og /Oi /Ot /Oy /Ob2 /GF /Gy` (global optimizations, intrinsics,
> favor-speed, frame-pointer omission, string pooling, function-level linking).

---

## 2. Options grouped by impact on final-DLL performance

### Group A — Large impact (the levers that actually move the needle)

| Option | What it does | Perf effect | Risk / tradeoff | Status here |
|---|---|---|---|---|
| **`/GL` + `/LTCG`** (Whole-Program / Interprocedural Optimization) | Defers codegen to link time so the optimizer sees **all** translation units at once → cross-`.cpp` inlining, whole-program devirtualization, better register allocation across module boundaries. In CMake: `CMAKE_INTERPROCEDURAL_OPTIMIZATION`. | **Highest "free" win.** Our hot loops span several `.cpp` files (collision, body, world) so cross-TU inlining is exactly what's missing today. | Longer link times; LTCG `.lib`s are toolset-version-specific (matters only if we LTCG the deps too). Safe for a single DLL. | **OFF on our DLL** (ON for CommonLibSSE). |
| **`/LTCG:PGO`** (Profile-Guided Optimization) | Instrument → run representative gameplay → re-optimize using the real branch/call profile. Lays out hot code together, predicts branches, inlines what's actually hot. | **Biggest single win for hot paths**, on top of LTCG. | Heavy workflow: needs a repeatable physics-heavy scene; profile is toolset/version specific; two-stage build. | OFF. |
| **`/O2`** vs `/O1` / `/Od` | `/O2` = maximize speed (default for Release). `/O1` = size, `/Od` = none. | Going below `/O2` would be a large regression; nothing to gain above it directly (the gains come from LTCG/PGO/inlining). | None — keep `/O2`. | ON (CMake default). |
| **`/Ob3`** vs `/Ob2`/`/Ob1`/`/Ob0` | Inline-expansion aggressiveness. `/Ob3` is the most aggressive. | Inlining is one of the largest single contributors to hot-loop speed. | Larger code → can pressure I-cache; usually a net win for this workload. | **ON** (`/Ob3`). |
| **`/arch:SSE2\|AVX\|AVX2\|AVX512`** | Selects the SIMD instruction set the compiler may emit (and that Bullet's SSE math uses). | Large for vectorizable physics/vector math. AVX2 also unlocks **FMA3**. | **Compatibility:** AVX needs Sandy Bridge+/Bulldozer+, AVX2 needs Haswell+/Zen+, AVX-512 is spotty (downclocking, removed from many consumer Intel parts). Wrong target = illegal-instruction crash on user CPUs. | Tiered (`noavx/avx/avx2/avx512`); user default is **AVX2**. |
| **`/fp:fast`** vs `/fp:precise` / `/fp:strict` | Floating-point model. `/fp:fast` allows reassociation, contraction (**FMA**), and faster transcendental/`/Oi` math; relaxes strict IEEE ordering. | Often substantial for FP-heavy physics; lets the compiler actually *use* FMA under `/arch:AVX2`. | **Changes results** (not bit-reproducible); can expose latent NaN/denormal assumptions. Acceptable for non-networked, non-deterministic physics — but must be visually validated. | `/fp:precise` (default). |

### Group B — Moderate impact

| Option | What it does | Perf effect | Status |
|---|---|---|---|
| **`/Gw`** (optimize global data) | Puts each global/static into its own COMDAT so `/OPT:REF` can drop unused data and LTCG can place it better. | Small-to-moderate; complements `/OPT:REF` and LTCG. Not implied by `/O2`. | **OFF** — easy add. |
| **`/Gy`** (function-level linking) | Each function in its own COMDAT → `/OPT:REF`/`/OPT:ICF` can fold/strip per function. | Moderate, but already implied by `/O2`. | ON (via `/O2`). |
| **`/GF`** (string pooling) | Merges identical string literals read-only. | Small; better locality. | ON (via `/O2`). |
| **`/OPT:REF`** | Linker drops unreferenced functions/data. | Smaller image, better I-cache locality. | **ON**. |
| **`/OPT:ICF`** | Folds identical COMDATs (functions/data). | Smaller image; minor. (Note: changes function-pointer identity — fine here.) | **ON**. |
| **`/INCREMENTAL:NO`** | Disables incremental linking → no jump thunks/padding → tighter, more direct calls. | Small but real for a release image. | **ON**. |
| **`/MD` vs `/MT`** (CRT linkage) | Dynamic vs static C runtime. Static can inline CRT and removes a layer of indirection. | Small; mostly relevant to startup/alloc paths. | `/MD` — **keep** (SKSE ecosystem convention; mixing CRT state across plugins is risky). |
| **Toolset version** (`v143` → `v145`/14.5x) | Newer MSVC optimizer/codegen. | "Free" gains from a better compiler each release. | `v143`; `vs2026`/`v145` work in progress. |

### Group C — Small / situational (mostly correctness or security with a perf side-effect)

| Option | Perf note | Recommendation |
|---|---|---|
| **`/sdl`** (additional security checks) | Adds runtime checks beyond `/GS` → measurable cost in tight loops. | Candidate to drop **in Release only** *after* review — but weigh against the project's input-validation/security stance (NIF/XML parsers). Keep in Debug. |
| **`/GS`** (buffer security check / stack cookie) | Small per-function prologue/epilogue cost. `/GS-` removes it. | **Keep.** The perf gain is tiny and the safety loss is real. |
| **`/guard:cf`** (Control Flow Guard) | Adds indirect-call check overhead. | Ensure **off** for the perf build (it is, by default). |
| **`/Qspectre`** | Inserts speculation barriers → overhead. | Ensure **off** (default). |
| **`/GR`** (RTTI) | `dynamic_cast`/`typeid` support; tiny size cost. | Leave on unless proven unused. |
| **`/EHsc`** | C++ exception model. Disabling EH is unsafe with our deps. | Keep. |
| **`/Zc:inline`** | Removes unreferenced COMDATs the compiler can prove unused → smaller image. | **ON** — keep. |
| **`/JMC-`** | Disables Just-My-Code debugging instrumentation (which only exists in Debug anyway). | ON for Release/Profile — correct. |
| **`/Oy`** (frame-pointer omission) | Frees a register; on x64 largely implied by `/O2`. | ON (via `/O2`). |
| **`/Qpar`** (auto-parallelize) | Rarely beats hand-threading; we already use TBB. | Skip. |

### Group D — **No** runtime-perf impact (do not touch these for speed)

These affect build time, diagnostics, or correctness only — listed so the plan doesn't waste effort
"optimizing" them:

- `/Zi`, `/DEBUG:FULL`, PDB generation — debug info lives in the PDB, **not** loaded at runtime →
  zero runtime cost. **Keep `/DEBUG:FULL`** for crash minidumps from end users.
- `/MP` — parallel *compile*; build-time only.
- `/W4`, `/WX`, `/wd4100`, `/wd4201`, `/external:*`, `/analyze` — diagnostics only.
- `/permissive-`, `/Zc:preprocessor` — standards conformance.
- `/bigobj` — raises object-file section limit; build-time only.
- `/utf-8` — source/exec charset.

---

## 3. Where each flag must be set (so the plan targets the right file)

| Target of optimization | Where to set it |
|---|---|
| Our `.cpp` (hdtSkinnedMesh, hooks, etc.) | `src/CMakeLists.txt` `target_compile_options` / `target_link_options`, or `CMAKE_INTERPROCEDURAL_OPTIMIZATION`. |
| All configs' baseline (`/arch`, `/EHsc`) | `CMakePresets.json` `CMAKE_CXX_FLAGS`. |
| **Bullet & other vcpkg deps** (statically linked, part of the hot path) | `cmake/triplets/x64-windows-static-md*.cmake` → `VCPKG_CXX_FLAGS` / `VCPKG_LINKER_FLAGS`. **This is the only way to optimize Bullet.** |
| CommonLibSSE | Already self-enables IPO in Release. |

## 4. Headline conclusion

The compiler-level optimizer is already near-maxed for our own code (`/O2 /Ob3`, `/arch:AVX2`).
The unrealized performance is almost entirely at the **whole-program / link / profile** layer and in
**the dependencies**:

1. **LTCG/IPO is off for the DLL** — the single biggest free win.
2. **No PGO** — the biggest win overall, at the cost of a profiling workflow.
3. **`/fp:fast` + FMA** is unused despite `/arch:AVX2` — large for FP-heavy physics, needs validation.
4. **Bullet (the heaviest math) is built without LTCG and without `/fp:fast`/`/Gw`** via the triplet.
5. SIMD tiering is already correct; AVX2 is the right "recommended" default.

See `PLAN.md` for the staged, measurable rollout.
