# smpview

A standalone HDT-SMP physics viewer/tuner that reuses FSMP's **actual** `hdt::`
solver outside Skyrim, so previews are faithful by construction (same constraint
code) rather than an approximation. Tracks issue #391.

## Status: Phase 0 (foundation)

Proven: the physics core (`src/hdtSkinnedMesh/*`) compiles and links with **no
CommonLibSSE / SKSE**, behind a ~210-line compatibility shim, and a smoke test
constructs + steps a real `SkinnedMeshWorld` standalone.

The seam is the PCH: the mod force-includes `src/PCH.h` (which pulls
`<RE/Skyrim.h>`); `smpcore` swaps in [`re_shim/smpcore_pch.h`](re_shim/smpcore_pch.h),
which provides only the few `RE::` utility types the core uses
(`BSIntrusiveRefCounted`, `BSTSmartPointer`, `make_smart`, `BSFixedString`,
`logger`). The one core→Skyrim coupling — two data members and the
`RESET_PHYSICS` sentinel — was removed by hoisting them into the base classes
(`SkinnedMeshWorld` / `SkinnedMeshSystem`), which also fixes a base-includes-
derived smell in the mod itself.

## Layout

- `re_shim/smpcore_pch.h` — standalone PCH (CommonLibSSE-free).
- `src/smoketest.cpp` — constructs + steps an empty `SkinnedMeshWorld`.
- `CMakeLists.txt` — self-contained project building `smpcore` (static lib) and
  `smpview_smoketest`. Links Bullet + TBB only.

## Build

Self-contained; configure with the vcpkg toolchain that supplies Bullet + TBB:

```
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
./build/smpview_smoketest      # prints "smpcore OK: ..."
```

## Not yet implemented (later phases / follow-up issues)

nifly-backed NIF/skeleton loading · GLFW/OpenGL + ImGui rendering and live
parameter sliders · kinematic driver (gravity-settle / procedural sway / drag) ·
SMP XML round-trip and Validator-clean export.
