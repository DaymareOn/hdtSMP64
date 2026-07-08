# ANALYSIS: `collisionperf` branch

Analysis of the changes on the `collisionperf` branch vs `dev`, and what each brings.

- **Date:** 2026-06-01
- **Scope:** `git diff dev HEAD` — net changes relative to `dev`, not to intermediate states within
  the branch.
- **Theme:** collision-detection performance and profiling instrumentation, with supporting fixes.

---

## Headline outcome (measured)

At **realistic load** (~22 active skeletons, 16 ms budget), the per-frame SMP impact sits at
**~3 ms** (`[SMP Metrics]` line, which is `QueryPerformanceCounter`-based and independent of the
profiler). The narrow phase is **no longer the bottleneck** — after the rewrite, the genuine collision
cost is small and the remaining per-frame time is dominated by unavoidable count-driven work, not
per-triangle math.

A measurement caveat for reading any profile of this branch: the fine-grained per-node-pair
`BT_PROFILE` scopes (`dispatch_body`, `filter_lists`, entered millions of times/frame) add enough
enter/leave overhead to **inflate the profiler tree by ~3×** — that inflation is an artifact of the
instrumentation, not collision work. They are therefore left **disabled** in the final state; only the
coarse per-step scopes are active.

The conclusion: per-call costs are now floored; further wins for high-triangle meshes must come from **reducing triangle/pair count**, not cheaper per-item work.

---

## Changes by theme

### 1. Narrow-phase rewrite — `checkCollide` (sphere-vs-triangle)

**What:** The `PerTriangleShape` narrow-phase test was rewritten:
- **Point-in-triangle via barycentric sign at the sphere centre**, not the projected point. Using the
  identity that the three sub-triangle area vectors sum to `raw_normal`, the third weight is derived
  (`len2 - da - db`) instead of computed — eliminating one cross product and one dot. The projection
  cancels out of the determinant (translation along the normal preserves barycentric coords), so the
  expensive projection is **not needed on reject paths**.
- **Deferred work:** `sqrt`/normalize, the unit normal vector (sign-tracked through the plane test and
  built once), margin/penetration setup, and the projection are all deferred **past the cheap rejects**
  (degenerate `len2`, both barycentric edges, the contact-plane test). Reject-heavy calls skip them.
- **`da`-first early-out** before computing `db` (and its subtract/cross/dot).

**What it brings:** Large reduction in per-call work on the common reject paths; the math is **exact**. Equivalence to the original was verified flip-by-flip for all penetration cases.

**Files:** `src/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp`. **Commits:** `5fa68fb` + this branch.

### 2. Per-triangle face-normal precompute

**What:** Each triangle's unnormalized face normal (`raw_normal`) and squared length (`len2`) are
precomputed once per frame in `PerTriangleShape::internalUpdate` and stored in `m_faceNormal`
(xyz = normal, w = len2), indexed in parallel with `m_colliders`. `checkCollide` reads them instead of
recomputing a cross product + dot **per collision pair**.

**What it brings:** Moves O(pairs) work to O(triangles) — a triangle's normal is computed once and
reused across every sphere tested against it. Profiled as `HDTSMP_skinUpdate` (~1.1 ms/frame at
realistic load), now visible as its own line.

**Files:** `hdtSkinnedMeshShape.{h,cpp}`, `hdtSkinnedMeshAlgorithm.cpp` (read path), `hdtDispatcher.cpp`
(invocation). **Commits:** `e92ec86`, `b5d6b7b`.

### 3. Broad-phase SoA AABB batch + FMA helper

**What:** In `dispatch`, the inner `i->collideWith(*j)` loop was replaced with an SoA batch:
`listB`'s AABBs are gathered into per-axis lanes via a 4×4 `_MM_TRANSPOSE4_PS`, then one `i` box is
tested against **4 (SSE)** or **8 (AVX2, `#if defined(__AVX2__)`)** `j` boxes per iteration, with a
scalar remainder. A small `nmsub_ps` FMA helper (in `hdtBulletHelper.h`) fuses `c - a*b` to a single
`vfnmadd` on FMA-capable builds, plain `mul`+`sub` otherwise.

**What it brings:** Cheaper broad-phase overlap testing on larger candidate lists, portable across all
four AVX variants. Overlap semantics match `Aabb::collideWith` exactly at the `>=` boundary, xyz-only
(no w contamination), and reject on NaN.

**Files:** `hdtSkinnedMeshAlgorithm.cpp`, `hdtBulletHelper.h`.

### 4. Inner parallelism → serial (rely on outer body-pair parallelism)

**What (vs dev):** dev fans out each body-pair's node-pairs with a nested `tbb::parallel_for_each`
(gated at `pairs.size() >= 32`) inside `dispatch`'s `operator()`. This branch disables that nested
fan-out (commented out); node-pairs now run serially (`for (auto& i : pairs) func(i)`). The outer
body-pair parallelism (`HDTSMP_collision_pair_checks`) still distributes work across threads.

**Why:** After the per-pair work was optimized to ~nothing, the nested fork/join's **barrier-wait
overhead dominated** — profiling showed most of `dispatch`'s time was threads idling at the inner
join, not computing. Going serial removes that overhead and the `isolate`/`thread_local` re-entrancy
hazard it required.

**What it brings:** Less scheduling overhead at realistic load; simpler code. (The fan-out only became
worth removing *because* this branch made per-pair work tiny — see the "Why". Validated directionally
versus dev's nested-parallel `dispatch`: recorded parallelism rose and barrier-wait fell, though
absolute numbers were noisy due to varying scene population and, until fixed, the profiler observer
effect.)

**Files:** `hdtSkinnedMeshAlgorithm.cpp` (lines ~651–660).

### 5. Tunable tree depth

**What:** `MaxTreeDepth = 4` constant added; `insertCollider` uses it instead of dev's hard-coded `4`
(dev's loop was `i < keyCount && i < 4`). Behaviorally **identical** to dev — the literal is just
named/tunable now. The comment records the A/B result: **4 is the sweet spot** — depth 3 was within
noise, depth 2 markedly worse (per-pair narrow-phase cost ~tripled, outweighing the ~20% pair-count
drop).

> Note: `MaxCollisionPairs = 6024` is **already present in dev** — this branch does **not** change it.
> (The earlier "4024 → 6024" framing measured against the merge-base `cbfe57f`, not against dev.)

**Files:** `hdtCollider.h`, `hdtCollider.cpp`. **Commit:** `e131e37`.

### 6. Profiling instrumentation

**What:** Coarse `BT_PROFILE` scopes added across the per-frame pipeline at **bounded** (per-step /
per-body) granularity: `processCollision`, `checkCollide`, `BVH`, `dispatch`, `sort`, `doMerge`,
`apply`, plus `HDTSMP_skinUpdate`, `systemInternalUpdate`, `broadphase`, `readTransform`,
`writeTransform`. The **per-node-pair** scopes (`dispatch_body`, `filter_lists`, millions of calls/
frame) are **commented out** — they distorted the measurement (~3× inflation) via enter/leave
overhead.

**What it brings:** A faithful per-frame call-hierarchy breakdown in the log (`smp profile`), near-zero
cost when profiling is off (the `gBtProfileEnabled` gate from the Bullet overlay-port patch), without
the observer effect that fine-grained scopes caused.

**Files:** `hdtSkinnedMeshAlgorithm.cpp`, `hdtDispatcher.cpp`, `hdtSkinnedMeshWorld.{h,cpp}`,
`hdtSkyrimPhysicsWorld.cpp`. **Commits:** `409e5a8`, `11849fa`, `f8bc83d`.

---

## What this branch does NOT address

For collision meshes with **excessive triangle counts**, the per-call costs are now floored and the remaining cost scales with triangle/pair count. Reducing that (content decimation, build-time culling/clustering, spatial broad-phase, collision LOD) is **not** implemented here.

## Correctness & validation notes

- **Math rewrites are exact**, not approximations. The FMA path differs from the non-FMA path by ≤1 ULP — expected cross-variant divergence, harmless.
- **Measurement caveats:** decisions were driven by the `[SMP Metrics]` line (profiler-independent) on
  realistic load; the profiler tree over-states cost when high-frequency scopes are enabled. Cross-load
  comparisons are noisy (scene population drifts); the robust signals were relative (parallelism ratio,
  per-phase shares) plus the metric line.
- **Recommended pre-merge validation:** build all four AVX variants in Release, verify cloth/hair
  visually (these changes touch contact generation), and read one profiling-OFF realistic capture as
  the true shipped cost.

## File-by-file summary

| File | Change |
|------|--------|
| `src/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp` | Narrow-phase rewrite, SoA broad-phase batch, serial inner, profiling scopes |
| `src/hdtSkinnedMesh/hdtBulletHelper.h` | `nmsub_ps` FMA helper (+ minor) |
| `src/hdtSkinnedMesh/hdtCollider.{h,cpp}` | `MaxTreeDepth` named constant (was literal `4`) |
| `src/hdtSkinnedMesh/hdtSkinnedMeshShape.{h,cpp}` | `m_faceNormal` precompute storage + fill |
| `src/hdtSkinnedMesh/hdtDispatcher.cpp` | skin-update scope; invokes face-normal precompute |
| `src/hdtSkinnedMesh/hdtSkinnedMeshWorld.{h,cpp}` | per-phase profiling scopes (systemUpdate, broadphase, read/writeTransform) |
| `src/hdtSkyrimPhysicsWorld.cpp` | `updateActiveState` profiling scope |
