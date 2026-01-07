# hdtSMP64 Architecture

This document provides an overview of the hdtSMP64 codebase architecture for developers and contributors.

## System Overview

hdtSMP64 is a physics simulation plugin for Skyrim SE/AE/VR that provides cloth and hair physics using the Bullet Physics engine. It integrates with SKSE (Skyrim Script Extender) to hook into game events and apply real-time physics simulation to character meshes.

```
┌─────────────────────────────────────────────────────────────────┐
│                        SKSE Plugin Layer                        │
├─────────────────────────────────────────────────────────────────┤
│  Hooks.cpp          │  main.cpp           │  ActorManager.cpp  │
│  (Game events)      │  (Plugin entry)     │  (Actor tracking)  │
└─────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Physics World Layer                        │
├─────────────────────────────────────────────────────────────────┤
│  SkyrimPhysicsWorld │  SkyrimSystem       │  SkyrimBone        │
│  (World management) │  (Per-actor system) │  (Bone physics)    │
└─────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Skinned Mesh Layer                           │
├─────────────────────────────────────────────────────────────────┤
│  SkinnedMeshWorld   │  SkinnedMeshSystem  │  SkinnedMeshBody   │
│  (Custom Bullet)    │  (Constraint groups)│  (Collision mesh)  │
└─────────────────────────────────────────────────────────────────┘
                               │
                    ┌──────────┴──────────┐
                    ▼                     ▼
        ┌───────────────────┐   ┌───────────────────┐
        │   CPU Backend     │   │   CUDA Backend    │
        │  (SSE/AVX SIMD)   │   │  (GPU compute)    │
        └───────────────────┘   └───────────────────┘
```

## Directory Structure

```
hdtSMP64/
├── hdtSMP64/                    # Main plugin source
│   ├── BulletCollision/         # Modified Bullet collision code
│   ├── BulletDynamics/          # Modified Bullet dynamics code
│   ├── LinearMath/              # Bullet math utilities
│   ├── hdtSkinnedMesh/          # Custom skinned mesh physics
│   │   ├── hdtSkinnedMeshWorld.cpp/h
│   │   ├── hdtSkinnedMeshSystem.cpp/h
│   │   ├── hdtSkinnedMeshBody.cpp/h
│   │   ├── hdtSkinnedMeshBone.cpp/h
│   │   ├── hdtCudaCollision.cu/cuh    # CUDA kernels
│   │   └── hdtCudaInterface.cpp/h     # CUDA management
│   ├── ActorManager.cpp/h       # Per-actor physics management
│   ├── hdtSkyrimPhysicsWorld.cpp/h    # Main physics world
│   ├── hdtSkyrimSystem.cpp/h    # Per-skeleton system
│   ├── Hooks.cpp/h              # Game function hooks
│   ├── Offsets.h                # Version-specific memory offsets
│   ├── config.cpp/h             # Runtime configuration
│   ├── XmlReader.cpp/h          # Physics XML parsing
│   └── main.cpp                 # Plugin entry point
├── hdtSSEUtils/                 # Utility library
├── tests/                       # Unit tests (Catch2)
├── configs/                     # Default configuration files
└── docs/                        # Documentation
```

## Core Components

### 1. Plugin Entry (main.cpp)

The SKSE plugin entry point. Responsibilities:
- Register with SKSE
- Initialize logging
- Set up event listeners (frame events, armor events)
- Start the physics world

### 2. Game Hooks (Hooks.cpp)

Uses Microsoft Detours to hook into game functions:
- **Armor attach/detach**: Triggered when armor is equipped/unequipped
- **Menu events**: Pause physics during loading screens
- **Frame events**: Trigger physics updates each frame

### 3. Actor Manager (ActorManager.cpp)

Tracks all actors (NPCs and player) with physics-enabled equipment:

```cpp
class ActorManager {
    struct Skeleton {
        NiNode* skeleton;           // Game skeleton root
        std::vector<Ref<SkyrimSystem>> systems;  // Physics systems
        float distance;             // Distance from camera
        bool isPlayer;
    };

    std::vector<Skeleton> m_skeletons;
    int m_activeSkeletons;  // Performance limit
};
```

Key functions:
- `onEvent(ArmorAttachEvent)`: Create physics for new armor
- `setSkeletonsActive()`: Select which skeletons get physics updates

### 4. Physics World (hdtSkyrimPhysicsWorld.cpp)

Singleton wrapping Bullet's discrete dynamics world:

```cpp
class SkyrimPhysicsWorld : public SkinnedMeshWorld {
    void doUpdate(float interval);      // Main physics step
    void readTransform(float dt);       // Read bone transforms from game
    void writeTransform();              // Write results back to game

    btVector3 m_windSpeed;              // Environmental wind
    float m_timeTick;                   // Physics timestep
    int m_maxSubSteps;                  // Max iterations per frame
};
```

### 5. Skinned Mesh System (hdtSkinnedMesh/)

Custom physics bodies that follow skeletal animation:

```
SkinnedMeshWorld
    └── SkinnedMeshSystem (one per physics XML)
        ├── SkinnedMeshBone[] (physics-enabled bones)
        ├── SkinnedMeshBody[] (collision bodies)
        └── Constraints[] (connections between bones)
```

Key classes:
- **SkinnedMeshBone**: A bone that can be driven by physics
- **SkinnedMeshBody**: Collision geometry attached to bones
- **ConstraintGroup**: Grouped constraints solved together

### 6. CUDA Backend (hdtCudaInterface.cpp, hdtCudaCollision.cu)

Optional GPU-accelerated collision detection:

```cpp
class CudaInterface {
    bool hasCuda();                     // Check GPU availability
    void doCollision(...);              // GPU collision detection

    // Device memory
    CudaBuffer<Vertex> m_vertices;
    CudaBuffer<Triangle> m_triangles;
};
```

Collision kernels run on GPU when:
- CUDA build variant selected
- Compatible NVIDIA GPU present (compute 5.0+)
- Sufficient vertex count to benefit

## Data Flow

### Per-Frame Update

```
1. FrameEvent triggered by game
        │
        ▼
2. SkyrimPhysicsWorld::onEvent()
   - Check if paused/loading
   - Calculate time delta
        │
        ▼
3. doUpdate(interval)
   - Accumulate time
   - Calculate substeps needed
        │
        ▼
4. readTransform()
   - For each SkyrimSystem:
     - Read bone world transforms from game skeleton
     - Update physics body positions
        │
        ▼
5. stepSimulation()
   - Bullet physics step
   - Collision detection (CPU or CUDA)
   - Constraint solving
        │
        ▼
6. writeTransform()
   - For each SkyrimSystem:
     - Write physics results back to game bones
     - Apply to rendered mesh
```

### Armor Attach Flow

```
1. Game equips armor on actor
        │
        ▼
2. Hook intercepts AttachModel()
        │
        ▼
3. ActorManager::onEvent(ArmorAttachEvent)
   - Find or create Skeleton entry
   - Load physics XML for armor
        │
        ▼
4. SkyrimSystemCreator::createSystem()
   - Parse XML configuration
   - Create bones, bodies, constraints
        │
        ▼
5. SkyrimPhysicsWorld::addSkinnedMeshSystem()
   - Register with physics world
   - Initialize collision pairs
```

## Configuration

### Runtime Config (configs/configs.xml)

```xml
<configs>
    <max-skeletons>20</max-skeletons>
    <min-fps>60</min-fps>
    <max-substeps>4</max-substeps>
    <cuda-enabled>true</cuda-enabled>
</configs>
```

### Physics XML (per-armor/hair)

```xml
<system name="hair_physics">
    <bone name="hair_01" mass="0.5">
        <shape type="sphere" radius="0.1"/>
    </bone>
    <constraint type="ballsocket" bodyA="head" bodyB="hair_01">
        <pivot>(0, 0, -5)</pivot>
    </constraint>
</system>
```

## Threading Model

```
Main Thread (Game)
    │
    ├── Frame Event ──────────► Physics Read Transform
    │                               │
    │                               ▼
    │                          Background Task
    │                               │
    │                          Physics Simulation
    │                               │
    │                               ▼
    └── Frame Sync ◄──────────── Write Transform
```

- Physics simulation runs on a background task
- Main thread reads transforms, triggers simulation
- Frame sync waits for simulation to complete before rendering

## Build Variants

Configuration naming: `{VERSION}_{CUDA}_{AVX}`

| Component | Options | Notes |
|-----------|---------|-------|
| VERSION | SE, VR, V1_6_xxx | Game version |
| CUDA | CUDA, NOCUDA | GPU acceleration |
| AVX | NoAVX, AVX, AVX2, AVX512 | CPU SIMD level |

## Key Preprocessor Defines

| Define | Purpose |
|--------|---------|
| `CURRENTVERSION` | Active game version (e.g., V1_6_1170) |
| `CUDA` | Enable CUDA code paths |
| `ANNIVERSARY_EDITION` | AE-specific code (v1.6.318+) |
| `SKYRIMVR` | VR-specific code |
| `BT_USE_SSE_IN_API` | Bullet SSE optimizations |

## Extension Points

### Adding New Constraint Types

1. Create class in `hdtSkinnedMesh/` inheriting from `btTypedConstraint`
2. Add XML parsing in `XmlReader.cpp`
3. Register in `SkinnedMeshSystem::addConstraint()`

### Supporting New Game Versions

1. Add version constant in `Offsets.h`
2. Find and add all required offsets (use IDA/Ghidra)
3. Add build configuration in vcxproj files
4. Update `CURRENTVERSION` mapping

## Performance Considerations

- **Skeleton limiting**: Only N closest skeletons get physics
- **Substep clamping**: Max 4 physics steps per frame
- **CUDA threshold**: GPU only beneficial for high vertex counts
- **Collision groups**: Reduce unnecessary collision checks
- **SIMD**: AVX2/AVX512 for CPU-bound calculations
