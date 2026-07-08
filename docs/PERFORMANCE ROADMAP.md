# PERFORMANCE ROADMAP — `collisionperf`

Candidate optimizations beyond the work documented in `COLLISIONS PERFORMANCE ANALYSIS.md`.
That document's conclusion — per-call narrow-phase costs are floored; further wins must come from
reducing counts and from the remaining per-frame lines — is the starting point of this one.

- **Date:** 2026-07-08
- **Branch state analyzed:** `collisionperf` retargeted onto `feat/skse-menu-framework`, including
  commits `d913b27` (per-node-pair profile scopes kept disabled) and `023c96f` (node-pair dispatch
  serial again) which restore the analysis doc's §6/§4 measured final state that the retarget had lost.
- **Method:** four independent deep code reviews (skin/update path, pair-count generation, frame
  orchestration, build/toolchain), findings verified against source. No new measurements were taken;
  every item below is **measure-first** unless marked otherwise.
- **Baseline:** ~3 ms/frame SMP impact at ~22 active skeletons (`[SMP Metrics]`), of which
  `HDTSMP_skinUpdate` ~1.1 ms. Fixed 1/60 s stepping; render-rate rig interpolation.
- **Status:** research backlog. Per project process, an item gets a GitHub issue when (and only
  when) it is picked up; cross-reference the issue number here at that point.

---

## 1. Ground truth: where a frame's work comes from

### 1.1 Pipeline (verified call chain)

1. **Body-pair level** — Bullet `btDbvtBroadphase` over one merged AABB per mesh
   (`m_tree.aabbAll`, set in `hdtSkinnedMeshBody.cpp:173`) → `needsCollision` filter
   (`hdtDispatcher.cpp:24-33,101`) → `m_pairs`.
2. **Per body-pair** — `collapseCollideL` boolean tree-overlap test (`hdtCollider.cpp:240-298`);
   if true, `checkCollisionL` **re-walks the same two trees** to emit leaf-node pairs
   (bail at `MaxCollisionPairs`, `hdtCollider.cpp:43-45`).
3. **Per node-pair** — `filter_lists` builds `listA`/`listB` (`hdtSkinnedMeshAlgorithm.cpp:~645`),
   then `dispatch` runs the `listA×listB` O(N·M) narrow phase.
4. **Every substep recomputes all of it** — no caching, no manifold warm-start
   (`hdtSkyrimPhysicsWorld.cpp:42-46`, `hdtDispatcher.cpp:10-22`).

The collider tree is a **bone-index trie, not a spatial BVH**: `insertCollider` descends one level
per skinning bone (`hdtCollider.cpp:7-19`); keys are `getBoneIdx()` sorted by weight
(`hdtSkinnedMeshShape.cpp:83-90` vertices, `:219-256` triangles). The measured "depth 4 optimal"
(`hdtCollider.h:16`) tuned the depth of this bone-keyed tree only — alternative *keyings* were
never tested.

### 1.2 Frequency model (what runs when)

| Work | Rate | Thread |
|---|---|---|
| `applyGravity`, `applyWind`, `updateActiveState`, `snapshotInterpolation`, `stepSimulation` | ~60 Hz (per step-group, outside the substep `while`) | background (TBB) |
| Collision + solver (per substep) | 60 Hz × substeps | background (TBB) |
| `applyInterpolatedTransform` → `writeTransform` ("Apply") | **every render frame** (`Hooks.cpp:226-233`) | **main thread**, after `m_tasks.wait()` |
| `ActorManager::setSkeletonsActive` | **every render frame** | **game thread** |

### 1.3 Strategic frame

The `[SMP Metrics]` line (`hdtSkyrimPhysicsWorld.cpp:440-447`) splits **Setup / Wait / Apply**:

- **Apply and game-thread work are unhidden** — they cost every render frame directly
  (at 144 Hz, 144×/s). Game-thread work isn't even counted by the SMP metric.
- **Background-step work is hidden under Wait** until the step overruns its 1/60 s budget.
  Savings there buy *headroom* (more skeletons / heavier outfits before FPS drops), not lower
  frame time on light scenes.

Rank accordingly: unhidden wins first at equal effort.

---

## 2. Tier 1 — unhidden frame-time (main/game thread, every render frame)

### 2.1 Cache interpolation endpoints as quaternions; blend with nlerp

- **Where:** `hdtSkinnedMeshBone.h:42-54`; consumed per dynamic bone per render frame from
  `writeTransform`.
- **Why it costs:** `interpolatedWorldTransform(alpha)` calls `getRotation()` (matrix→quaternion:
  sqrt + branches) on **both** endpoints plus a `slerp` (acos/sin) every render frame — yet both
  endpoints only change once per physics step; only `alpha` changes between frames.
  ~2 extractions + 1 slerp × ~1–2 k bones × render rate ≈ 0.2–0.4 ms/frame on the main thread.
- **Change:** cache `prevQuat`/`prevOrigin` at snapshot and `curQuat`/`curOrigin` once per step
  (physics thread); per render frame, nlerp the cached quaternions. At 60 Hz physics the
  inter-step angles are tiny, so nlerp is visually identical to slerp.
- **Impact:** M (scales with refresh rate × bone count). **Risk:** L–M (A/B nlerp on fast camera
  whips). **Effort:** S–M.

### 2.2 Parallelize `writeTransform` by skeleton group; delete the self-copy and the dead write

- **Where:** `hdtSkinnedMeshWorld.h:51-58` (serial `for` over systems — the analogous
  `readTransform` at `:46-48` is already `tbb::parallel_for`); per-bone body in
  `hdtSkyrimBone.cpp:93-105`.
- **Why it costs:** the "Apply" metric is serial main-thread work across all skeletons while the
  workers idle (it runs after `m_tasks.wait()`). Per bone it also performs
  `m_node->world = m_node->world` (`:105`, a full self-assignment already flagged in-code) and a
  dead `m_currentTransform` write (`:99-100`, overwritten by the next `readTransform` before any
  reader).
- **Change:** partition systems **by skeleton root** (the grouping `updateActiveState` already
  computes) and `parallel_for` across the groups — not across systems: multiple systems share a
  skeleton and `updateTransformUpDown` walks shared ancestors (race). Remove the two dead writes.
- **Impact:** M–H (largest unhidden serial block). **Risk:** M (NiNode reentrancy — the per-skeleton
  partition is the safety argument). **Effort:** M.

### 2.3 Throttle `setSkeletonsActive` sorting and LOS raycasts

- **Where:** `ActorManager.cpp:371-531`, invoked every `FrameEvent` (`:185-194`); full
  `std::ranges::sort` (`:416-437`); per-skeleton scene raycasts — wind obstruction (`:509`) and
  `isInPlayerView` (`:1089`).
- **Why it costs:** up to ~22 skeletons × up to 2 scene raycasts per render frame + O(n log n)
  sort, on the **game thread**, invisible to the SMP metric. The active set changes slowly
  relative to render rate.
- **Change:** keep cheap per-frame distance/frustum checks; throttle the sort + LOS raycasts to
  ~physics rate or round-robin one skeleton per frame; cache the active set between recomputes.
  Exempt `forceKeepNear` from throttling.
- **Impact:** M. **Risk:** M (activation popping if throttled too coarsely near the camera).
  **Effort:** M. **Measure with a full-frame capture, not the SMP line.**

---

## 3. Tier 2 — background-step throughput (headroom at high load)

### 3.1 Parallelize the serial `systemInternalUpdate` prologue

- **Where:** `hdtSkinnedMeshWorld.cpp:178-184` — plain serial `for (system) system->internalUpdate()`;
  inside: per-bone transform updates then per-mesh `updateBoundingSphereAabb`, including **full
  skins of every ≤10-collider body** on this serial path (`hdtSkinnedMeshBody.cpp:234,264-265`).
- **Why:** entirely single-threaded (~100–300 µs) while all other cores idle — an Amdahl tax paid
  before the parallel `HDTSMP_skinUpdate` even starts.
- **Change:** two parallel phases preserving the bones→meshes dependency: `parallel_for_each`
  bone updates across all systems, then mesh AABB updates. Mirror `readTransform`'s structure.
- **Impact:** M–H on that line. **Risk:** L (bones/meshes independent across systems).
  **Effort:** S–M.

### 3.2 Repack `Vertex` 48 B → 32 B (u16 bone indices, fp16 weights — lossless)

- **Where:** `hdtVertex.h:7-30` (`m_skinPos`, fp32 `m_weight[4]`, u32 `m_boneIdx[4]`); hot loads in
  the skin loop `hdtSkinnedMeshBody.cpp:64-168` (`calcVertexStateFMA` `:28-38`); populate at
  `hdtSkyrimSystem.cpp:772-790`.
- **Why:** the skin loop is **memory-bound** — bone matrices fit L1; the 48 B/vertex read stream
  dominates. Weights are *already fp16 in the source data* and expanded via `_mm_cvtph_ps` at
  build (`hdtSkyrimSystem.cpp:772-790`), so storing fp16 is bit-lossless vs today. Bone indices
  are bounded far below 65 536 (asserted at `:788`). New layout = 32 B → 2 vertices per cache
  line, −33 % read traffic.
- **Change:** repack; in the loop expand weights `_mm_cvtph_ps(_mm_loadl_epi64(...))` and indices
  `_mm_cvtepu16_epi32` (spare ALU in a memory-bound loop). Build-time guard `bones < 65536`; keep
  a scalar expand path for the noavx variant (no F16C).
- **Impact:** M–H if bandwidth-bound (decide with one roofline/L2-miss sample), L if
  latency-bound. **Risk:** numerically ~zero; blast radius = ~6 read sites. **Effort:** L.

### 3.3 Spatial (or bone-then-spatial hybrid) collider-tree keying

- **Where:** `hdtCollider.cpp:7-19`, keys from `hdtSkinnedMeshShape.cpp:83-90,219-256`; consumed by
  the walk (`hdtCollider.cpp:58-67`) and `filter_lists`.
- **Why:** bone-index buckets don't partition space; sibling leaf AABBs overlap heavily, inflating
  **both** the emitted node-pair count and the per-pair `|listA|×|listB|` — the costs compound,
  and this is what pushes single mesh pairs toward the 6024 cap.
- **Change:** at `finishBuild` (topology fixed; only positions animate), split by median/SAH over
  rest-pose collider centroids into balanced leaves; keep the existing bottom-up refit
  (`hdtCollider.cpp:158-204`) unchanged. Narrow-phase math untouched → contacts stay exact.
- **Counter-argument to test honestly:** bone grouping stays *coherent under deformation* — a
  limb's vertices move together, while a rest-pose spatial split across a joint smears that leaf's
  refit AABB when the joint bends. A hybrid (bone at level 0, spatial below) may beat both.
  **Re-run the depth-2/3/4 A/B under any new keying**, and measure leaf-AABB quality across a
  walk/run cycle, not at rest pose.
- **Impact:** potentially H (attacks the count problem directly). **Risk:** M. **Effort:** L–M
  (build-time only).

### 3.4 Set per-bone gravity once, not per step

- **Where:** `hdtSkinnedMeshWorld.cpp:212-224` — every step, every bone:
  `setGravity(m_gravity * m_gravityFactor)`. Both inputs are load-time constants
  (`hdtSkyrimPhysicsWorld.cpp:15`; `hdtSkyrimSystem.cpp:668`); only the scale branch of
  `hdtSkyrimBone.cpp:34-49` can invalidate (its `setMassProps` doesn't refresh rigid-body gravity —
  the reason it's currently re-pushed every step).
- **Change:** set at `addSkinnedMeshSystem` (`hdtSkinnedMeshWorld.cpp:87-91`); re-set only in the
  `scaleChanged` branch; `applyGravity()` reduces to the base-class call.
- **Impact:** L–M (~1–2 k cold-memory writes/step removed). **Risk:** L. **Effort:** S.

### 3.5 `updateActiveState`: dirty-flag instead of per-step rebuild

- **Where:** `hdtSkyrimPhysicsWorld.cpp:262-310` — per step builds
  `unordered_map<NiNode*, Group>` with per-group `unordered_set<BSFixedString>` tags, for inputs
  (`m_tags`, `m_disableTag`, `m_disablePriority`, membership) that change only on armor
  attach/detach/reload.
- **Change:** dirty-flag set by `addSkinnedMeshSystem`/`removeSkinnedMeshSystem` (`:312-343`);
  cache each body's resolved `m_disabled`. Steady state → zero work.
- **Impact:** L–M. **Risk:** L (invalidate on *all* membership changes). **Effort:** S–M.

### 3.6 Dispatcher hygiene: reuse per-frame vectors; gate unconditional small-body skins

- **Where:** `hdtDispatcher.cpp:77,84` — `bodies` / `extra_vertex_shapes` are fresh heap vectors
  every frame + `std::sort`/`std::unique` (`:123-127`). Small bodies are fully skinned
  unconditionally on the serial path (`hdtSkinnedMeshBody.cpp:264-265`) even with nothing nearby.
- **Change:** promote to reused members (like `m_pairs`) or dedup with a per-body generation
  stamp; for small bodies reuse the already-computed `worldBoundingSphere` as the broad-phase
  proxy and defer the exact skin to the parallel phase, or gate on last-frame overlap.
- **Impact:** L–M. **Risk:** small-body gate trades tighter AABBs for possibly more candidate
  pairs — **measure pair count**. **Effort:** S (vectors) / M (gate).

### 3.7 Chunk the skin loop of large bodies

- **Where:** `hdtDispatcher.cpp:135-144` — parallel grain = one whole body; a 10 k-vertex cloak
  skins on one thread and becomes the phase's tail.
- **Change:** above a vertex threshold, `tbb::parallel_for(blocked_range)` over the vertex loop
  (embarrassingly parallel: bones read-only, disjoint writes); ideally a global `(body, range)`
  chunk list for balance. Distinct from the removed narrow-phase nested parallelism — that was
  tiny per-task work; this is thousands of vertices per task.
- **Impact:** M (worst-case tail; H if one mesh dominates). **Risk:** L. **Effort:** M.
  **Measure the per-body vertex distribution first.**

### 3.8 Fuse the leaf AABB rollup into the collider-AABB build

- **Where:** `hdtSkinnedMeshShape.cpp:76,150` write every `m_aabb[i]`, then `m_tree.updateAabb()`
  (`hdtCollider.cpp:158-204`) immediately **re-streams the whole array** (leaves are contiguous
  ranges — `hdtCollider.cpp:323-345`). Companion `PerVertexShape::internalUpdate` similarly
  re-reads all of `m_vpos` for AABBs the skin loop had in-register.
- **Change:** leaf-driven update — build collider AABBs and accumulate each leaf's `aabbMe`
  in-register in one pass; `updateAabb` keeps only the parent rollup. Optionally emit per-vertex
  AABBs from the skin loop (margin already rides the w-lane).
- **Impact:** M for large meshes (200–320 KB/body/frame of re-fetch removed). **Risk:** M —
  exactness-sensitive; must preserve the "don't merge into `aabb[0]`" fix
  (`hdtCollider.cpp:180-181`); verify AABBs bit-identical. **Effort:** M.

---

## 4. Tier 3 — bigger bets (feature-level, visual-risk)

### 4.1 Distance-based collision LOD

- **Where:** per-skeleton `m_distanceFromCamera2` already computed every frame
  (`ActorManager.cpp:988-1007`) but **unused by collision**; today it only drives the binary
  attach/detach budget (`:444-578`). Rig interpolation can hide reduced rates.
- **Change:** per-system LOD tier set in `updateActiveState`: (a) beyond X, drop tri-vs-tri to
  sphere-only (skip one `processCollision` sub-pass, `hdtSkinnedMeshAlgorithm.cpp:~857`);
  (b) beyond Y, drop inter-actor body-pairs at the `needsCollision` filter (`hdtDispatcher.cpp:101`);
  (c) far tier runs collision every other substep. Needs hysteresis; likely config-gated.
- **Impact:** H in crowds. **Risk:** M (mid-distance interpenetration, cloak clipping between
  NPCs). **Effort:** M (plumbing distance to the mesh side is the bulk).

### 4.2 Temporal reuse of the surviving pair set across substeps

- **Where:** everything rebuilt per substep (§1.1 step 4); at 1/60 s bones move sub-millimeter,
  so the surviving node-pair set is near-identical step to step. Worst during multi-substep
  catch-up.
- **Change (lightest):** per body-pair, cache last step's surviving `pairs`; re-test only those
  leaf overlaps and run the full `checkCollisionL` every N steps or when the body's `aabbAll`
  moves beyond a threshold. Invalidate on tree rebuild/remap.
- **Impact:** H during catch-up, M at steady state. **Risk:** M — staleness can miss a
  just-forming contact (late-collision shimmer); bound the cache age. **Effort:** M.

### 4.3 Real bounding-sphere body-pair pre-cull + single tree walk

- **Where:** the mesh-level broadphase uses one giant merged AABB per mesh
  (`hdtSkinnedMeshBody.cpp:173`), so long meshes form corner-overlap body-pairs that never truly
  interact; `isBoundingSphereCollided` (`:268-274`) is a misnamed no-op that tests no sphere; the
  double tree walk is flagged in-code (`hdtDispatcher.cpp:151-153`).
- **Change:** (a) O(bones) `worldBoundingSphere`-vs-sphere reject before the tree walk;
  (b) merge `collapseCollideL` into `checkCollisionL` (single descent serving both callers).
- **Impact:** M. **Risk:** L (behavior-preserving). **Effort:** M.

### 4.4 `MaxCollisionPairs` cliff — priority-ordered budget only

- **Where:** `checkCollisionL` truncates the walk at 6024 in arbitrary DFS order
  (`hdtCollider.cpp:43-45`); the caller then drops the **entire body-pair**
  (`hdtSkinnedMeshAlgorithm.cpp:~616`) → cloth fully penetrates: a hard visual cliff.
- **History constraint:** the in-code comment records that *processing* the truncated set was
  tried and "only cause[d] the collision to become significantly more tangle[d] up" — so the
  naive "process what we collected" variant is contradicted by prior in-game experience.
  Only the **priority-ordered** variant is on the table: emit pairs deepest-overlap-first (or
  nearest-K) so a budget is spent on the most-penetrating contacts.
- **Impact:** M (robustness; bounds the worst case). **Risk:** M (ordering cost).
  **Effort:** M. Note: 3.3 reduces how often the cap is reached at all.

---

## 5. Tier 4 — build/toolchain (honest ceiling: single-digit %)

| Item | Evidence | Change | Impact / Risk / Effort |
|---|---|---|---|
| **`tbbmalloc_proxy`** | per-frame manifold alloc/free under parallel dispatch (`hdtDispatcher.h:23-30`) hits the locked CRT heap; nothing links tbbmalloc today | link `tbbmalloc_proxy`, ship its DLL | M if heap-lock contention is real, else L / L / **S** — sample for `RtlpAllocateHeap` first |
| **PGO** | zero wiring (`.gitignore:104` only); PROFILE config is vestigial (flags identical to RELEASE, never registered); deterministic `smp_replay` exists (benchmarksuite branch) | `/GENPROFILE` build, replay-train, `/USEPROFILE` into RELEASE | M (5–12 % on the branchy narrow phase) / M (pgd staleness, 4-variant matrix) / L |
| **`/Ob3` on deps** | triplets add only `/arch` — deps build at vcpkg default `/O2 /Ob2 /fp:precise`, no WPO (`cmake/triplets/*.cmake`) | add `/Ob3` to `VCPKG_CXX_FLAGS` | L / L / S |
| **`/GL` on Bullet** | Bullet is static + hot (solver pool, Dbvt) but native COFF → our `/LTCG` can't inline it | `/GL` in triplets | M / **gotchas below** / M |
| **AVX-512 16-wide batch** | the SoA broad-phase batch caps at 8-wide `__AVX2__` (`hdtSkinnedMeshAlgorithm.cpp:~526`); avx512 DLL reuses the 256-bit path | `__AVX512F__` 16-wide path | **L — broad phase isn't the bottleneck; deprioritized** |

**Cross-cutting gotchas (fix alongside any triplet change):**
1. **CI cache key omits triplets** — `build.yml:168` hashes `vcpkg.json` + `cmake/ports/**` but
   **not `cmake/triplets/**`**. Correctness holds (vcpkg ABI hash covers it) but expect silent
   full rebuilds; add the triplets glob to the hash.
2. **Toolset pin vs `/GL`** — deps pin MSVC 14.44 (`_pin_toolset.cmake:15-16`) while the plugin
   may link with 14.50: `/GL` objects are toolset-locked → `LNK1257`. `/GL` on deps couples dep
   and plugin toolsets.

**Rejected:** `/fp:fast` (globally or per-TU on our math) — the branch's narrow-phase rewrite is
verified *exact* with ≤1 ULP cross-variant divergence; `/fp:fast` reassociation forfeits that
guarantee for an unmeasured gain. FMA is already explicit (`nmsub_ps`, `hdtBulletHelper.h:96-103`).

---

## 6. Confirmed non-issues (do not re-litigate)

- **`BT_PROFILE` gate cost when profiling is off:** inline `gBtProfileEnabled` load + predicted
  branch per *coarse* scope (a few dozen/frame) — sub-microsecond total. The runtime profiler
  stays. (Per-node-pair scopes stay disabled for measurement fidelity — analysis doc §6.)
- **Island batching / solver knobs:** tiny islands are already accumulated to
  `m_minimumSolverBatchSize` (128) by the stock `btSimulationIslandManagerMt`; solver iteration
  count is already exposed (`GlobalConfig.h:44`, clamp 4–128 in the menu) and couples to
  stiff-spring behavior — not a free knob.
- **TBB inlining:** oneTBB ships as a DLL (vcpkg forces dynamic) — permanently outside any LTCG
  boundary; only Bullet is a `/GL` candidate.
- **8-wide AVX2 skinning:** doesn't fit the data flow (per-vertex bone gather; xyzw-native 4-wide
  with margin in w). The skin wins are bandwidth/parallelism (§3.1, 3.2, 3.7), not width.
- **Minor rescans measured as small:** kinematic-kinematic constraint rescan per substep
  (`hdtSkinnedMeshWorld.cpp:427-435`), translation-offset triple pass
  (`hdtSkyrimPhysicsWorld.cpp:207-239`), ~6 `sin`/bone/step wind advection — all small vs the
  tiers above.

---

## 7. Measurement protocol

- Drive decisions from the profiler-independent **`[SMP Metrics]`** line
  (`hdtSkyrimPhysicsWorld.cpp:440-447`) on a realistic ~22-skeleton scene, profiling **off**.
- Tier 1 items show up in **Apply** (and §2.3 only in a **full-frame** capture — the SMP metric
  doesn't see game-thread cost). Tier 2/3 items show up as **Wait/step headroom**: validate by
  raising load (skeleton count) until the step overruns, then compare.
- Offline A/B: `smp_replay` + capture fixtures on the `benchmarksuite` branch; deterministic
  input via the devbench integration plan. Same tooling doubles as the PGO training workload.
- Count instrumentation (cheap counters, not profiler scopes): emitted node-pairs per body-pair,
  `|listA|×|listB|` sizes, and **how often the 6024 cap is hit** in crowd scenes — §3.3/§4.4 are
  sized by those numbers.
- Any contact-affecting change (§3.3, §3.8, §4.x): visual cloth/hair capture A/B plus, where
  applicable, bit-exactness diffs (AABBs, contact sets) before/after.

## 8. Suggested sequencing

1. **Quick wins, no measurement gate:** §3.4 gravity, §3.5 activeState dirty-flag, §3.6 vectors
   (+ delete §2.2's two dead writes while there).
2. **Unhidden frame time:** §2.1 interpolation caching → §2.2 parallel writeTransform → §2.3
   ActorManager throttling.
3. **Step throughput:** §3.1 parallel prologue → §3.2 vertex repack (after one bandwidth-vs-latency
   sample) → §3.7 chunked skinning if the tail is real.
4. **Measured experiments:** §3.3 tree keying (with the §7 counters), then §3.8, §4.3.
5. **Features:** §4.1 LOD, §4.2 temporal reuse — each behind config, each with visual A/B.
6. **Toolchain, opportunistic:** tbbmalloc sample → `/Ob3` deps → PGO prototype (avx2 only) →
   `/GL` Bullet last (both gotchas first).
