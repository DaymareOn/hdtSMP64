# Community Shaders — "Strand Hair" feature (render half of the wig PoC)

This is the **Community Shaders (CS)** side of the strand-hair proof of concept. FSMP (hdtSMP64)
simulates guide strands and exports world-space bead positions over a C-ABI; this CS feature
resolves that ABI at runtime and draws the strands over the opaque scene.

**These exact files were built** against a CS fork (upstream `doodlum/skyrim-community-shaders`)
with CMake 4.2 + VS2022 + vcpkg. They are kept here, version-controlled with the FSMP branch, so
the feature can be reproduced in any CS fork. hdtSMP64 does **not** compile them.

## Files
- `StrandHair.h` / `StrandHair.cpp` — the CS `Feature` subclass. Resolves FSMP's `FSMPWig_*`
  exports via `GetModuleHandle`/`GetProcAddress` (no link dependency), uploads each wig's beads
  into a `StructuredBuffer`, and draws a line list from a new `PostOpaque()` callback.
- `Shaders/StrandHair/StrandHair.hlsl` — VS reads beads by `SV_VertexID` (no input layout needed)
  and applies the camera `viewProj`; PS is a Kajiya-Kay diffuse term with a root→tip gradient.
- `Shaders/Features/StrandHair.ini` — `[Info] Version = 1-0-0` (makes the feature "installed").
- `cs-core-edits.patch` — the 5 tiny CS-core changes that add the post-opaque hook + register the
  feature (12 inserted lines total). Apply with `git apply` inside a CS checkout.

## How to reproduce in a CS fork
1. Clone a fork of `doodlum/skyrim-community-shaders --recursive`; set `VCPKG_ROOT` to a vcpkg
   whose checkout matches `vcpkg.json`'s `builtin-baseline`.
2. Copy the sources into place:
   - `StrandHair.h` + `StrandHair.cpp` → `src/Features/`
   - `Shaders/StrandHair/StrandHair.hlsl` → `features/StrandHair/Shaders/StrandHair/`
   - `Shaders/Features/StrandHair.ini` → `features/StrandHair/Shaders/Features/`
3. Apply the core edits: `git apply contrib/cs-strandhair/cs-core-edits.patch` (from the CS root).
   They are:
   - `src/Feature.h` — add `virtual void PostOpaque() {}`.
   - `src/Deferred.cpp` — call `Feature::ForEachLoadedFeature("PostOpaque", …)` right after
     `EndDeferred()` in `Main_RenderWorld_BlendedDecals::thunk` (post-opaque, before water).
   - `src/Globals.h` / `src/Globals.cpp` / `src/Feature.cpp` — declare/define/register the singleton.
4. Configure + build (VS2022 here, since no VS2026):
   ```
   cmake --preset ALL-VS2022
   cmake --build build/ALL-VS2022 --target CommunityShaders --config Release
   ```
   `src/*.cpp` is globbed with `CONFIGURE_DEPENDS`, and the version registry is regenerated from
   the INI on configure, so no CMakeLists edit is needed.
5. Deploy the built DLL + the `Shaders/` tree to your CS mod folder and enable "Strand Hair"
   (under the **Characters** category). Enabling it calls `FSMPWig_SetEnabled(1)`, so FSMP starts
   simulating as soon as the feature is active.

## The handoff contract (matches FSMP's `src/FSMP_WigAPI.h`)
- `FSMPWig_GetInstanceCount()`, `FSMPWig_GetInstance(i,&desc)`, `FSMPWig_CopyPositions(i,dst,cap)`.
- Positions are Skyrim world space (Z up, no axis remap) and feed straight into `viewProj`.

## Known PoC state (see the plan's Part B for the real versions)
- **Confirmed rendering in-game (2026-07-07)** on every SMP-active actor.
- **Camera transform (load-bearing):** the game's live per-frame cbuffer **b12** via
  `Common/FrameBuffer.hlsli`, camera-relative position, matrix-first mul — exactly
  `mul(FrameBuffer::CameraViewProj, float4(pos - FrameBuffer::CameraPosAdjust.xyz, 1.0))`.
  Do NOT use the CPU-side `frameBufferCached` capture or row-vector `mul(v, M)`: both produce
  off-screen/distorted output (this cost days to diagnose).
- **Line-list strands**, not thickness quads; **alpha blend**, not PPLL OIT.
- **Depth test is OFF** (`SetupResources`) — hair draws over everything. For correct occlusion,
  bind the `kMAIN` DSV and switch the depth state to `LESS_EQUAL` (like TerrainBlending).
- Single hard-coded directional light; no self-shadow, no SSS/Skylighting parity.
