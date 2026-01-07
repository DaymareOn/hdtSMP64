# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

hdtSMP64 is a physics simulation plugin for Skyrim SE/AE/VR that provides cloth and hair physics using the Bullet Physics engine. It's an SKSE (Skyrim Script Extender) plugin that hooks into the game to simulate soft body physics on character meshes.

## Build System

This is a Visual Studio 2019+ solution (`hdtSMP64.sln`). The project requires:
- Visual Studio 2019 or newer
- CUDA 11.6 (for GPU-accelerated builds)
- Microsoft Detours library
- Bullet Physics library (built separately)
- SKSE64 source code (the solution expects to be placed within the SKSE source tree)

### Build Configurations

Configuration names follow the pattern: `{VERSION}_{CUDA}_{AVX}[_DEBUG]`

**Game Versions:**
- `SE` - Skyrim SE (v1.5.97)
- `VR` - Skyrim VR (v1.4.15)
- `V1_6_353` - Anniversary Edition early
- `V1_6_640` - Anniversary Edition post-629
- `V1_6_659` - Anniversary Edition
- `V1_6_1170` - Anniversary Edition latest

**CUDA Options:**
- `CUDA` - GPU collision detection enabled (requires NVIDIA GPU with compute capability 5.0+)
- `NOCUDA` - CPU-only collision

**AVX Options:**
- `NoAVX` - Maximum compatibility
- `AVX` - Requires AVX support
- `AVX2` - Requires AVX2 support
- `AVX512` - Requires AVX512 support

Example: `V1_6_659_CUDA_AVX2` builds for AE 1.6.659 with CUDA and AVX2 optimizations.

### Building

1. Set up dependencies (Detours, Bullet, SKSE source) per README.md
2. Open `hdtSMP64.sln` in Visual Studio
3. Select appropriate configuration (e.g., `SE_NOCUDA_AVX|x64`)
4. Build Solution

Output is a `.dll` file that goes in Skyrim's `Data/SKSE/Plugins/` directory.

## Architecture

### Solution Structure

- **hdtSMP64** - Main plugin, produces the SKSE plugin DLL
- **hdtSSEUtils** - Utility library for NetImmerse/NIF handling
- **skse64** - SKSE dependency (static lib)
- **skse64_common** - SKSE common code
- **common_vc14** - Shared utilities

### Core Components

**Entry Point & Hooks (`main.cpp`, `Hooks.cpp`)**
- SKSE plugin entry point
- Detours-based hooks into game functions for armor attach/detach events
- Menu event handling (loading screens, race menu)

**Actor Management (`ActorManager.cpp/h`)**
- Tracks all NPCs and player with physics-enabled meshes
- Manages skeleton activation/deactivation based on distance, visibility, and performance
- Handles armor piece attachment and head part physics
- Key class: `ActorManager::Skeleton` contains all physics data for an actor

**Physics World (`hdtSkyrimPhysicsWorld.cpp/h`)**
- Singleton wrapping Bullet's `btDiscreteDynamicsWorld`
- Manages simulation stepping, wind effects, and frame timing
- Handles suspend/resume during loading screens

**Skinned Mesh System (`hdtSkinnedMesh/` directory)**
- `SkinnedMeshWorld` - Custom Bullet world with skinned mesh support
- `SkinnedMeshBody` - Physics body that follows bone transforms
- `SkinnedMeshSystem` - Collection of bodies and constraints for one physics system
- `hdtCudaCollision.cu/cuh` - CUDA kernels for GPU collision detection
- `hdtCudaInterface.cpp/h` - CPU-side CUDA management

**Physics System Configuration (`hdtSkyrimSystem.cpp/h`)**
- `SkyrimSystem` - Physics system bound to a skeleton
- `SkyrimSystemCreator` - XML parser that creates physics systems from config files
- Reads physics definitions from XML files in mod packages

**Version-Specific Offsets (`Offsets.h`)**
- Contains hardcoded memory offsets for each supported game version
- `CURRENTVERSION` preprocessor define selects active offsets
- Critical for hooking game functions correctly

### Physics Data Flow

1. Game triggers armor attach event via hooked function
2. `ActorManager` receives event, finds/creates skeleton entry
3. XML physics definition loaded via `SkyrimSystemCreator`
4. `SkyrimSystem` created with bones, bodies, and constraints
5. System added to `SkyrimPhysicsWorld`
6. Per-frame: `FrameEvent` triggers physics step
7. Transforms read from game skeleton, physics simulated, results written back

### Key Preprocessor Defines

- `CURRENTVERSION` - Set by build configuration to select game version (see `Offsets.h`)
- `CUDA` - Enables CUDA code paths
- `ANNIVERSARY_EDITION` - Defined for AE builds (v1.6.318+)
- `SKYRIMVR` - Defined for VR builds

## Configuration

Runtime configuration in `configs/configs.xml`:
- Physics simulation frequency (`min-fps`)
- Maximum active skeletons
- CUDA enable/disable
- Wind effects settings
- Constraint solver parameters

## Adding Support for New Game Versions

1. Add new version constant in `Offsets.h`
2. Add offsets for all hooked functions (find via IDA/Ghidra with signatures)
3. Add build configurations in `.vcxproj` files
4. Update `CURRENTVERSION` mapping logic

## Console Commands

In-game console commands via `smp <command>`:

| Command | Description |
|---------|-------------|
| `smp` | Show tracked skeletons, armors, collision meshes |
| `smp reset` | Full physics reset (reloads config + meshes) |
| `smp reload` | Hot-reload configs.xml without resetting meshes |
| `smp on/off` | Enable/disable physics simulation |
| `smp list` | List active physics systems |
| `smp detail` | Detailed physics system info |
| `smp timing [N]` | Profile N frames (default 200), prints mean/std timing |
| `smp metrics` | Toggle continuous metrics logging to hdtSMP64.log |
| `smp stats` | Show current performance stats |
| `smp gpu` | Toggle CUDA collision (CUDA builds only) |
| `smp dumptree` | Dump targeted reference's node tree |

## Static Analysis

Run cppcheck locally (excludes Bullet library, uses parallel processing):

```powershell
# Windows PowerShell
.\scripts\cppcheck.ps1
```

```bash
# Git Bash / Linux
./scripts/cppcheck.sh
```

CI runs the same analysis automatically on push/PR.
