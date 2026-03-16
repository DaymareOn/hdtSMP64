## Code Review: DaymareOn/hdtSMP64 master — CUDA removal + CPU SIMD/threading approach

I read the full source tree and decoded the key headers. Here's what the code actually looks like and where to focus. [github](https://github.com/DaymareOn/hdtSMP64/tree/master/src)

---

### 1. Current CUDA entanglement — what to actually remove

The CUDA code is guarded by `#ifdef CUDA` throughout `hdtSkinnedMeshBody.h`, but it's not just in `.cu` files; it splits data structure definitions: [api.github](https://api.github.com/repos/DaymareOn/hdtSMP64/contents/src/hdtSkinnedMesh/hdtSkinnedMeshBody.h)

```cpp
// Still live in hdtSkinnedMeshBody.h
#ifdef CUDA
    std::shared_ptr<Bone[]>      m_bones;
    std::shared_ptr<VertexPos[]> m_vpos;
    std::shared_ptr<CudaBody>    m_cudaObject;
#else
    std::vector<Bone>      m_bones;
    std::vector<VertexPos> m_vpos;
#endif
```

**Action:** Remove the `#ifdef CUDA` branches entirely; consolidate on the `std::vector` path. Then remove `hdtCudaCollision.cu`, `hdtCudaCollision.cuh`, `hdtCudaInterface.cpp/.h`, `hdtCudaPlanarStruct.cuh`, and the `clkernel/` folder (OpenCL remnants). Strip CUDA from `CMakeLists.txt`. This is a clean surgery, not a risky refactor — the CPU path was always there in the `#else` branches.

---

### 2. Threading — world-level loop is completely serial

`hdtSkinnedMeshWorld.h` shows: [api.github](https://api.github.com/repos/DaymareOn/hdtSMP64/contents/src/hdtSkinnedMesh/hdtSkinnedMeshWorld.h)

```cpp
void readTransform(float timeStep) {
    for (int i = 0; i < m_systems.size(); ++i)
        m_systems[i]->readTransform(timeStep);
}
void writeTransform() {
    for (int i = 0; i < m_systems.size(); ++i)
        m_systems[i]->writeTransform();
}
```

Both loops are **fully serial at the world level** even though `SkinnedMeshSystem` already has PPL `task`/`task_group` internally. Each system's NiNode reads/writes are independent, so this is low-hanging fruit: wrap both loops in a `concurrency::parallel_for` or a PPL task group. The main risk is NiNode write-back thread safety — check if Skyrim's NiNode is safe to write from non-game-threads (it generally is for bone transforms when using the separate physics thread FSMP already has). [api.github](https://api.github.com/repos/DaymareOn/hdtSMP64/contents/src/hdtSkinnedMesh/hdtSkinnedMeshSystem.h)

---

### 3. `canCollideWith` — linear search in a hot path

In `hdtSkinnedMeshBody.h`: [api.github](https://api.github.com/repos/DaymareOn/hdtSMP64/contents/src/hdtSkinnedMesh/hdtSkinnedMeshBody.h)

```cpp
bool canCollideWith(const SkinnedMeshBone* bone) const {
    if (m_canCollideWithBones.size())
        return std::find(m_canCollideWithBones.begin(),
                         m_canCollideWithBones.end(), bone)
               != m_canCollideWithBones.end();
    return std::find(m_noCollideWithBones.begin(),
                     m_noCollideWithBones.end(), bone)
           == m_noCollideWithBones.end();
}
```

This is called for every broadphase candidate pair per frame. Replace both `std::vector<SkinnedMeshBone*>` with `std::unordered_set<SkinnedMeshBone*>` (already used for the tag sets in the same class) or a sorted vector + `std::binary_search`. For typical configs with ~10–20 entries this won't be dramatic, but it's a correctness-at-scale fix too.

---

### 4. Data layout for SIMD

With CUDA removed, `m_bones` and `m_vpos` become plain `std::vector`. The next step is to convert them to **SoA** layout for SIMD-friendly iteration in `internalUpdate()` and vertex skinning:

- Current AoS: `vector<Bone>` where each `Bone` holds position/rotation/etc together.
- Target SoA: separate contiguous arrays for `x[]`, `y[]`, `z[]`, `w[]` so you can process 4 or 8 at a time with AVX2 intrinsics.

The `btMatrix4x3T vertexToBone` in `SkinnedBone` is already likely SSE-friendly given the custom `_mm_setzero_ps()` usage in `CollisionShape`'s constructor — that's a good sign the original code was at least aware of SIMD alignment. Ensure `m_bones` and `m_vpos` vectors are `alignas(32)` (AVX2) allocated.

---

### 5. Virtual dispatch in the hot path

`SkinnedMeshSystem::readTransform`, `writeTransform`, and `internalUpdate` are all **virtual**. These are called in the per-frame inner loop. Consider: [api.github](https://api.github.com/repos/DaymareOn/hdtSMP64/contents/src/hdtSkinnedMesh/hdtSkinnedMeshSystem.h)

- Making the world-level dispatch use the concrete type directly where possible (most systems are `hdtSkyrimSystem` in practice).
- Or using a non-virtual `internalUpdateImpl()` that the base calls via CRTP, removing the vtable hit from the innermost loop.

---

### 6. `GroupConstraintSolver` + `btConstraintSolverPoolMt`

`hdtSkinnedMeshWorld` already has a `btConstraintSolverPoolMt* m_solverPool` — this is the right multithreaded solver path. Verify it's actually wired up to Bullet's `btDiscreteDynamicsWorldMt` (the class inherits from it), and that the pool thread count is tuned to `std::thread::hardware_concurrency() - 1` or similar to leave headroom for the Skyrim render thread. [api.github](https://api.github.com/repos/DaymareOn/hdtSMP64/contents/src/hdtSkinnedMesh/hdtSkinnedMeshWorld.h)

---

### 7. Recommended removal + refactor order

| Priority | Task                                                                    | Risk                               |
| -------- | ----------------------------------------------------------------------- | ---------------------------------- |
| 1        | Strip all `#ifdef CUDA` blocks + remove `.cu`/`.cuh`/`clkernel` files   | Low — CPU path was always present  |
| 2        | Fix `canCollideWith` linear scan → `unordered_set`                      | Very low                           |
| 3        | Parallelize world-level `readTransform`/`writeTransform` loops with PPL | Medium — test NiNode thread safety |
| 4        | Align + SoA-ify `m_bones`/`m_vpos` for AVX2 skinning                    | Medium — data layout refactor      |
| 5        | Remove virtual dispatch from `internalUpdate` inner loop                | Low–Medium                         |
| 6        | Tune `btConstraintSolverPoolMt` thread count                            | Low                                |

The `clkernel/` folder (OpenCL kernels) should also be deleted alongside the CUDA files — it's the same dead GPU path just in a different API. [github](https://github.com/DaymareOn/hdtSMP64/tree/master/src/hdtSkinnedMesh)
