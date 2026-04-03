# Changelog

All notable player-facing changes are documented here.

---

## [3.0.0-dev] — since `1b08468`

### 🐛 Bug Fixes

- **Facial morphs no longer break** — Fixed a Skyrim bug where hard-part items (like SMP hair) could cause face morphs (expressions, RaceMenu morphs) to stop working. Affects characters with physics-enabled hair or head accessories.
- **Dynamic HDT (DHDT) no longer corrupts game saves** — Saving while a DHDT override was active could make the save impossible to load. This is now fixed with a safer save/load system.
- **Improved DHDT transfer stability** — Switching physics presets via DHDT no longer causes visual explosions or constraint glitches. World transforms are now properly computed before transferring velocities between rigs.
- **Fixed the "slot 32" physics bug** — A long-standing bug where certain outfits' physics would completely break has been resolved.
- **Bone name matching is now case-insensitive** — Short or mixed-case bone names no longer cause SMP glitches or physics failures.
- **Fixed first-person ↔ third-person reset** — Switching between first and third person no longer causes a physics explosion on the player character.
- **Fixed race condition in transform writes** — Transform data is now safely written on the main thread, preventing rare physics corruption.
- **Fixed a memory leak in skeleton nodes** — Previously, skeleton nodes were not always properly cleaned up, which could cause memory growth over long sessions.
- **Fixed head corruption leading to CTD** — Corrected an issue where face geometry (FaceGen) processing could corrupt data over time and eventually crash the game.
- **Raised the collision contact limit** — The old artificial limit was causing FPS drops when two physics shapes got stuck inside each other. The limit has been doubled, giving Bullet Physics more information to resolve overlaps without a framerate tank.
- **Physics shapes are now built from the neutral havok pose** — Prevents shapes from being built in incorrect positions at load time.
- **Log level is now correctly read from the config file** — The log verbosity setting was previously being ignored.
- **VR: Fixed crashes from bone limit and bad bone references** — Two separate crash-to-desktop bugs affecting VR users have been fixed.
- **Fixed minor memory leaks** — Several small memory management issues in collision and mesh code have been resolved.

---

### ⚡ Performance Improvements

- **Loading hair/outfit physics in RaceMenu is now 2–4× faster** — `SystemCreator` was heavily optimized: bone lookups are now O(1) via hash maps, triangle-building no longer spams heap allocations, collision shapes are built in parallel, and F16C hardware instructions are used to convert bone weights on supported CPUs (AVX2+).
- **Smoother physics simulation** — Multiple internal optimizations to how simulation "islands" (groups of interacting objects) are managed reduce overhead each frame.
- **Better CPU cache usage in transform processing** — The per-bone parallelism was replaced with per-system parallelism, which is faster due to reduced cache misses and thread overhead.
- **Optimized collision dispatcher** — The low-level collision broadphase and dispatch code was cleaned up and significantly optimized.
- **Optimized mesh collision** — Skinned mesh (soft body) collision now avoids unnecessary heap allocations and redundant work each frame.
- **Compiler and SIMD improvements** — Several hot paths now use SSE4 and AVX2 intrinsics for faster math (quaternion/transform operations).

---

### ✨ New Features

- **BudgetMS: Time-based physics budget** — The physics timing system has been redesigned. Instead of a frame percentage, you now set a `budgetMs` value: the maximum number of milliseconds FSMP can use per frame. This makes behavior more predictable across different hardware and frame rates. The metrics display has also been improved to show clearer frame-time impact numbers.
- **Improved wind simulation** — The wind implementation has been overhauled for more realistic and stable cloth/hair movement in wind zones.
- **AVX build is now the default** — The installer now defaults to the AVX variant (better performance on modern CPUs) instead of the no-AVX fallback.
- **AVX variant is logged at startup** — The log file now reports which AVX level the DLL was compiled for, making it easier to verify your installation.
- **Config and SMP reset events are now logged** — When the physics config is loaded or SMP resets, the active settings are written to the log. Useful for diagnosing configuration issues.
- **XSD schema for `configs.xml`** — An XSD validation file is now included, allowing mod authors to validate their `configs.xml` files in editors like VS Code or Notepad++ before shipping.
- **Better dev build identification** — Dev builds now log their exact version string (e.g. `hdtsmp64 v3.0.0-dev-abc1234`) at startup.

---

### 🗑️ Removals / Cleanup

- **CUDA support has been removed** — CUDA-accelerated physics was unmaintained and caused build complexity with no practical benefit. All users were already running the CPU path. The FOMOD installer has been cleaned up accordingly.
- **Unused config fields removed** — Several leftover configuration fields that had no effect have been removed from the sample config files to reduce confusion.

---

### 📦 Version

- **Official version is now 3.0.0** — This dev branch targets the 3.0.0 release milestone.
