# PLAN: Speed up collision for high-triangle-count collision meshes

- **Status:** DRAFT — not approved for implementation (see GSTACK REVIEW REPORT below).
- **Created:** 2026-06-01
- **Branch:** `collisionperf`
- **Owner:** DaymareOn

---

## Problem

A `PerTriangleShape` stores **one `Collider` per triangle**. Every per-frame and per-collision
cost in the pipeline scales with that count:

- **Skin update** (`HDTSMP_skinUpdate`): recomputes `m_vpos`, per-collider AABB, and the precomputed
  face normal for *every* triangle, every frame.
- **BVH** (`ColliderTree::checkCollisionL`): more triangles ⇒ more leaves ⇒ more node-pairs.
- **Broad phase**: pair count grows **super-linearly** when two dense meshes overlap (this is the
  multiplier we saw explode to millions of pairs/frame in the stress capture).
- **Narrow phase** (`checkCollide`): more calls.

The cost is dominated by **count**, not per-item work. So the remaining lever for dense collision
meshes is to **reduce how many triangles are skinned/tested**, or to **cull collision pairs
spatially** — not to make each triangle cheaper.

## Goals

- Reduce per-frame cost for meshes with excessive collision-triangle counts, at realistic load.
- Preserve collision quality (cloth/hair behaviour) within an acceptable, documented tolerance.
- Prefer low-risk, build-time or content-time mitigations over hot-path changes.

## Non-goals

- Re-optimizing the narrow phase — diminishing returns vs the 16 ms budget.
- Changing the simulation/solver.
- Any change that silently drops collisions a player would notice (must be opt-in or provably inert).

## Background (measurement methodology to reuse)

- Use `smp profile <sampleFrames> <printFrames>`; read **settled** windows, not the one right after
  `smp reset`.
- The `[SMP Metrics]` line (Setup/Wait/Apply) is `QueryPerformanceCounter`-based and **independent of
  `BT_PROFILE`** — it's the un-distorted truth. Beware that high-frequency `BT_PROFILE` scopes inflate
  the tree (observer effect); keep per-node/per-triangle scopes OUT of any measurement build.
- Distinguish **realistic load** (a handful of nearby SMP actors, budget intact) from the **extreme/
  torture** setup (many actors, physics uncapped). Decisions ship on realistic-load numbers; the
  torture run is only for amplifying a signal.
- Each item below must show a before/after on a **load-matched** realistic scene.

---

# This branch (`collisionperf`)

Two items remain in scope for this branch: collision LOD (item 5, the current focus) and the spatial
broad-phase rewrite (item 4). Sequencing: **5 before 4** — 5 is medium effort with immediate crowd
headroom and reuses existing structures; 4 is the largest effort/risk and deserves its own design doc.

## Item 5 — Collision LOD / POV-coverage culling  ← CURRENT FOCUS

**Goal:** Spend the collision budget where it matters; reduce fidelity for distant / less-visible /
less-important actors instead of paying full cost for everyone. A **fidelity** tier on top of the
existing binary **on/off** gating.

**Existing levers to build on:**
- `ActorManager` active-skeleton tracking + the `activeSkeletons` count (the current on/off gate).
- Budget signal: `processTimeInMainLoop` vs `budgetTime` (per frame, already measured).
- The `m_useBoundingSphere` gate (already skips re-skinning bodies far from any contact).
- `PerTriangleShape`'s `PerVertexShape` companion — a ready-made *cheaper representation* of the same
  body, ideal as a downgrade target.

### Priority snapshot — every proposed evolution, ranked by value ÷ effort

The whole plan at a glance, best bang-for-buck first. This is a *prioritisation* signal only — the **build
order** still respects dependencies (A needs item 3's LOD chain; B / D3 / E2 need A's tiers):

1. **1.1 — collision-density diagnostic + authoring guidance** — value M / effort XS. A one-time build
   warning + docs that surface the dominant *content-side* win (nothing in code makes N triangles free).
   Cheapest change, points at the biggest lever.
2. **D (D1) — rate-decoupled fixed step + interpolation** — value H / effort M. *Global*: caps physics at
   ~60 Hz and interpolates, so every high-fps user stops paying ~2.4× for a visually identical result. No
   content dependency, benefits everyone.
3. **A — POV-coverage fidelity tiering** — value H / effort M. The core LOD mechanism (full → decimated →
   cull); large crowd savings, but its intermediate tiers depend on item 3.
4. **C — importance / visibility weighting** — value L–M / effort S. Mostly *already* in the culling rank;
   small work to add combat-target/companion biases. Folds into A.
5. **E (E2) — inter-actor pair cull by POV tier** — value H (crowds) / effort S–M. Drops cross-actor pairs
   when both actors are low-POV — the N² crowd cost the existing cull chain doesn't touch.
6. **2.1 — prune provably-inert triangles at build** — value M / effort S–M. Shrinks every per-frame cost
   with zero fidelity change; the correctness analysis is the bulk of the work.
7. **B — adaptive budget controller** — value H / effort M. Moves the band cuts each frame to fit
   `budgetTime`; extends the existing auto-adjust. Second-order (needs A's tiers).
8. **3 — build-time decimation / clustering** — value H / effort M. Produces the LOD chain A downgrades to —
   the enabler for A's intermediate tiers (other branch).
9. **D (D3 / D5) — staggered stride + rest-gating** — value H (crowds) / effort M. Per-actor temporal LOD on
   top of D1; idle and strided crowds get cheap. Needs D1 + tiers.
10. **F — occlusion culling (depth buffer)** — value H (dozens-of-warriors) / effort L. The crowd headline,
    but a new GPU module + version-specific depth hook → highest effort/risk in item 5.
11. **2.2 — drop unreachable / interior triangles** — value M / effort M–L. Lower confidence; high
    false-positive risk → likely opt-in.
12. **4 — spatial broad-phase rewrite** — value H ceiling / effort L. Biggest rewrite and risk; only
    justified if dense overlapping meshes are a *common* target.

### The LOD system is one mechanism built from components A–F

These are **not competing alternatives** — they are layers of a single system that combine, with the
**POV-coverage ranking** (from the culling system) as the shared spine:

- **A — fidelity tiering** (the mechanism): per-skeleton tier → per-body collision-mesh LOD level (full →
  decimated triangle LODs → culled).
- **B — adaptive budget controller**: *where* the band cuts sit on the ranking, moved each frame to fit
  `budgetTime`.
- **C — importance / visibility**: shapes the ranking metric itself (largely already in the culling rank).
- **D — temporal rate**: *how often* each tier re-collides (an orthogonal fidelity axis).
- **E — inter-actor cull**: a crowd-specific broad-phase cull layered on top.
- **F — occlusion culling**: depth-buffer visibility that culls parts hidden behind other actors (the crowd
  headline); feeds A's freeze and E's pair cull.

### Component A — POV-coverage tiering (the fidelity mechanism)

**Tier metric: POV coverage, not raw distance.** What matters is the **fraction of the view the skeleton
occupies** (apparent/projected size), not metric distance — a large actor far away and a small actor up
close can warrant the same fidelity. This is exactly the prominence ranking the **culling system already
computes**: `ActorManager::setSkeletonsActive` derives per-skeleton `m_distanceFromCamera2` and
`m_cosAngleFromCameraDirectionTimesSkeletonDistance`, sorts skeletons by `distance / cos(angle)` (central
+ near first), and applies frustum (`NodeInFrustum`) + line-of-sight (`Actor_CalculateLOS`). **LOD reuses
that same metric** — it is just *more bands* on the existing ranking, where today's binary active/inactive
cut becomes the bottom band.

**Granularity (resolved):** decide **per actor (skeleton)**, apply **per body (mesh)**.
A tier is computed **once per skeleton per frame** (the culling pass already produces the ranking).
Deciding per shape or per body would be redundant and could leave one actor incoherent (one mesh at full
detail, the next coarse); the player reads an actor as a unit, so tier boundaries are per-actor. The
fidelity *representation*, however, is per body — each body selects **which collision-mesh LOD level** to
use — so each body maps the single actor tier to its own LOD chain.

| Level | Role |
|-------|------|
| Actor / skeleton (`ActorManager::Skeleton`) | **Decide** — POV-coverage rank (reuse the culling metric) → tier |
| System (armor/XML, `SkinnedMeshSystem`) | pass-through; optional per-system override for high-value items |
| Body / mesh (`SkinnedMeshBody`) | **Apply** — map actor tier → this body's LOD level; cache the tier here for cheap dispatch reads |
| Collision mesh / collider | mechanism only (the LOD mesh selected by the body's choice) |

**Tiers as bands of the POV-coverage ranking** (most prominent → least). Each tier is a **decimated
collision-mesh LOD** — fewer, larger *triangles*, which still collide correctly by vertex-face (just
coarser). The collision algorithm is unchanged; only the mesh fed to it gets coarser:
- **T0 (largest POV):** LOD0 — the full authored collision mesh.
- **T1:** LOD1 — moderately decimated mesh.
- **T2:** LOD2 — coarsely decimated mesh.
- **T3 (smallest POV / off-screen):** cull — no physics (today's inactive cut).
- The **player's own actor** is pinned to T0 (already forced first / distance 0 in the culling sort).
- A body that ships only one collision mesh (no LOD chain) has just **T0 → cull** until LODs are generated.

> **Dependency (cross-branch):** these tiers need a *chain* of decimated collision meshes per body to
> downgrade to. Generating that chain is **item 3 (build-time decimation), which lives in the other
> branch.** So component A is coupled to item 3: without the LOD chain there is nothing between full and
> cull, and A degenerates to today's binary culling. The decimation effort must produce a runtime-
> selectable LOD chain (not just a single coarser mesh) for component A to have intermediate tiers.

**Per-frame data flow:**
1. The culling pass (`setSkeletonsActive`) already ranks skeletons by POV coverage and computes the
   active/inactive cut. Extend it to assign a **tier** per skeleton from band thresholds on that same
   ranking, with **hysteresis** (separate enter/exit thresholds + dwell time) to stop popping/flicker.
2. The tier is stamped onto each `SkinnedMeshBody` under that skeleton (a cached field), so dispatch
   never recomputes it or pointer-chases up to the actor.
3. In `SkinnedMeshAlgorithm::processCollision(body0, body1, …)`, each body picks its representation from
   its own cached tier. The two bodies of a pair may carry different tiers (different actors) — each uses
   its own.

**Why this reuses existing structure:** the **per-skeleton POV ranking + frustum/LOS** is already computed
by `setSkeletonsActive`; LOD only adds intermediate bands on it. The narrow-phase is untouched — each LOD
level is just a smaller triangle mesh fed through the same `checkCollide` (vertex-face), so collisions
stay *correct* at every tier, only coarser.

*Pros:* savings scale with the decimation ratio; reuses the culling ranking; coherent per-actor
transitions; POV coverage already folds in prominence/visibility (much of **C** is subsumed); **collisions
remain valid (vertex-face) at every tier** — no fake sphere/point-cloud approximation. *Cons:* depends on
precomputed LOD collision meshes (item 3, other branch) — without them there is nothing between full and
cull; visible coarsening if a tier is selected too near; a LOD swap mid-contact can pop (vertices move) —
likely needs a settle frame; needs hysteresis to avoid flicker.

### Component B — adaptive budget controller (where the band cuts sit)

**Role in the system:** A defines *what* each tier does and the POV ranking defines the *order*; **B
decides where the band boundaries fall**, dynamically, so total collision cost fits `budgetTime`. Instead
of fixed POV thresholds, B moves the cuts each frame in response to measured load.

**The bottom cut already works this way.** The culling controller (`m_autoAdjustMaxSkeletons` in
`setSkeletonsActive`) already auto-adjusts `maxActiveSkeletons` — the active/inactive (T3) cut — from
`processTimeInMainLoop` vs `budgetTime`, moving it ±1–2/frame with clamps. **B generalises that single
knob into multiple cuts** on the same ranked list.

**Mechanism (v1, minimal):** keep the existing active cut and add **one more** — the *triangle cut*
separating T0 (full triangles) from T1 (spheres). Since skeletons are already sorted by POV coverage, the
bands are just count cutoffs on the sorted list:

```
[ top … triangleCut )      → T0 (triangles)
[ triangleCut … activeCut ) → T1 (spheres)        (a T2 self-only cut can be added later)
[ activeCut … end ]        → T3 (culled)
```

Drive both cuts with the **same damped ±step controller** the culling already uses, in a **priority
order**:
- **Over budget** (`processTime > budgetTime`): lower `triangleCut` first — demote the least-prominent
  triangle users to spheres (big saving, small visual cost) — and only lower `activeCut` (cull) once the
  triangle band is exhausted.
- **Under budget** (with headroom): raise `activeCut` (un-cull) / give spheres their triangles back, T1→T0
  last.

This degrades gracefully: keep **more actors with *some* physics (spheres)** before dropping to *fewer at
full fidelity*, and cull only as a last resort.

**Cost/savings estimate:** to choose how far to move a cut, B needs the saving a demotion buys. Options:
(a) reuse the per-skeleton timing the metrics already track (`averageTimePerSkeletonInMainLoop`); (b) a
collider-count proxy (triangle vs companion-sphere count). Start with (a) if available, else (b).

**Stability:** budget measurements are noisy frame-to-frame, so — like the existing controller — move cuts
**gradually** (±1–2 skeletons/frame, clamped) and add **hysteresis** so a skeleton near a boundary doesn't
flip tier every frame. This budget-level hysteresis is *on top of* component A's per-skeleton hysteresis.

**Unify, don't duplicate:** B should *extend* `m_autoAdjustMaxSkeletons` into a single multi-cut
controller, not run a second controller alongside it — two controllers chasing the same budget oscillate.

*Pros:* never blows the budget; self-tunes across hardware; reuses the existing auto-adjust machinery and
the POV ranking; graceful crowd degradation. *Cons:* tier assignment varies frame-to-frame → flicker risk
(mitigated by damping + hysteresis); needs a per-skeleton cost estimate for precise cuts.

**Open questions (B):**
- Control variable: count cutoffs on the ranked list (simple, reuses existing) vs a cost-budget walk
  (precise, needs a cost model). Recommend counts for v1.
- Budget target & headroom: target a *fraction* of `budgetTime` so a promotion doesn't immediately
  re-trigger a demotion (hysteresis on the budget signal itself).
- Demotion/promotion order once there are more than two bands (T0→T1→T2→cull).
- Per-skeleton cost-estimate source: measured time vs collider-count proxy.

### Components C–F — further layers of the same system

**C. Importance / visibility weighting (not pure distance).**
Prioritize by what the player perceives: player + nearby/companion/targeted actors always full;
off-screen (behind camera / outside frustum) actors minimal; combat-relevant boosted. Distance is one
input among several.
- *Pros:* spends budget where it's noticed; off-screen savings are visually "free".
- *Cons:* needs game-state queries (frustum/on-screen, combat target) → more engine integration;
  off-screen actors re-entering view must re-initialise cleanly (state continuity).

**D. Temporal LOD (reduced collision update rate).**
Reduce *time* resolution instead of *space*: lower-priority actors advance their physics fewer times per
second, interpolating in between. Orthogonal to A/F; layers on top of them.

*What already exists (the catch-up accumulator):* [`doUpdate`](src/hdtSkyrimPhysicsWorld.cpp#L95-L132) is a
fixed-step catch-up accumulator. `tick = min(m_averageInterval, m_timeTick)` caps the *fixed* step at 1/60
(`m_timeTick = 1/min_fps`); [`stepSimulation`](src/hdtSkinnedMesh/hdtSkinnedMeshWorld.cpp#L150-L176) runs
`floor(remaining/tick)` catch-up steps then **one variable final step down to a 1/300 s floor**; afterwards
`m_accumulatedInterval = 0` consumes *all* accumulated time, and **no two-state buffer is kept**.

The consequence that motivates D: at high fps `averageInterval` shrinks, so `tick` shrinks, and the engine
does **one ever-smaller step every frame** (down to the 1/300 floor). Per-step cost barely depends on dt, so
a 144 fps user pays ~2.4× the physics work of a 60 fps user for a visually identical result. The issue is
not that Bullet can't step below 1/60 — it already does, every frame — it's that **doing so every frame is
wasted work**.

*D1 — rate-decoupled fixed step + interpolation (the framework).* Cap the *rate* (not just the step size) at
~60 Hz: step only when `accumulated ≥ fixedStep`, **carry the remainder** (stop zeroing the accumulator),
and **interpolate render frames between the last two solved states** by `remainder/fixedStep`. This is the
standard "fix your timestep" pattern. It is *global* (every high-fps user benefits) and, more importantly,
it is the **substrate** temporal LOD rides on — per-tier rate lives *inside* this accumulator. Cost: keep a
previous+current pose buffer and a blend pass; change the accumulator-reset logic.

*D2 — bigger steps (lossy; not a per-actor knob).* Running a body at dt > 1/60 is cheaper but the code
itself warns the [ERP/CFM solvers are dt-sensitive](src/hdtSkinnedMesh/hdtSkinnedMeshWorld.cpp#L164-L166):
a bigger step changes *stiffness/damping* — it **alters dynamics**, where D1/D3 preserve them. And there is
**one global world + one solver pass**, so per-body dt inside a shared solver isn't something Bullet does
cleanly. So D2 is only a **global low-fps fallback** or a **deepest-tier** lever — never a fine per-actor
dial. The per-actor lever is stride (D3), not dt.

*D3 — phase-staggered per-body stride (the practical per-actor LOD).* Keep one global ~60 Hz world step, but
each lower-tier body integrates+collides only every *N* steps (*N* from its POV tier), **staggered by body
index** so the cost is amortized rather than a periodic hitch where every strided body updates on the same
frame. On skipped steps the body holds its last *secondary displacement* — and since SMP only displaces the
game-skinned pose (see **F**), "held" does not drift; the cloth still rides the animated skeleton, it just
stops adding secondary motion. Interpolate that displacement between a body's own updates. This is the real
temporal LOD, living *inside* D1's framework; the **stagger** is what keeps per-frame cost flat.

*D5 — rest / motion gating (orthogonal axis: motion, not distance).* Skip the solve for a body whose anchor
bones moved < ε and whose cloth is near rest since last step — a sleep state that wakes on bone motion or new
contact. Bullet sleeps rigid-body islands; SMP cloth almost certainly does not. Idle NPCs standing in a crowd
are a large fraction of crowd cost and need no re-solve. Composes with D3 (far *and* resting = cheapest).

*D6 — ride the existing async producer/consumer.* The sim already runs on a background TBB task
([`m_tasks.run(... doUpdate2ndStep ...)`](src/hdtSkyrimPhysicsWorld.cpp#L129)) — the natural home for D1's
interpolation buffer. The main thread interpolates the latest *finished* background state; if a heavy frame
means the background can't finish in time, the effective rate drops and interpolation stretches → graceful
degradation instead of a stall.

*D7 — interpolate vs extrapolate (a per-tier decision).* Interpolation is smooth but buffers state and adds
up to one fixed-step of latency; extrapolation has no latency but overshoots/snaps on direction changes. So
the **tier** picks the policy — never apply temporal LOD to the player.

*Combined picture (tiered):*

```
Near / player : step live at render rate            — no temporal LOD, zero latency
Mid           : D1 accumulator + interpolation, D3 staggered stride (e.g. 30 Hz)
Far           : D3 strided + D5 rest-gated; optional global D2 bigger step in the deepest tier
```

All on the existing async task (**D6**), tiers from the same POV ranking as A/B/C/F.

*Pros:* near-linear savings with no spatial fidelity loss; D1 alone helps every high-fps user; preserves
dynamics (D1/D3) where it matters. *Cons:* interpolation latency / temporal aliasing on fast motion (gated
to non-player tiers); a pose-buffer + blend pass to add; rest-gating needs a reliable wake condition.

*Open questions (D):*
- Fixed base rate — lock to 60 Hz, or expose it (relates to `m_timeTick`/`min_fps`)?
- Stagger scheme: round-robin by body index vs hash, and how *N* maps to POV tier.
- Wake condition for D5 (bone-motion ε, new broadphase contact) and how it interacts with D3's stride.
- Interpolation state lives on which object — per-body pose snapshots vs reusing `m_rig` interpolation
  transforms already present ([hdtSkyrimBone.cpp](src/hdtSkyrimBone.cpp#L55-L73))?
- Confirm the accumulator-reset change doesn't regress the existing low-fps catch-up (maxSubSteps) path.

**E. Inter-actor collision culling (crowd-specific).**

*What already exists (the per-frame cull chain):*
1. **Per-body markout AABB** — `SkinnedMeshBody::updateBoundingSphereAabb()` builds each skinned bone's
   `worldBoundingSphere` and merges their AABBs into the body's whole-body AABB (`m_bulletShape.m_aabb`).
2. **Bullet broadphase** (`btDbvtBroadphase`) pairs bodies whose markout AABBs overlap
   (`performDiscreteCollisionDetection` feeds it `getCollisionShape()->getAabb()`).
3. **`needsCollision`** rejects: same body, both-kinematic, and tag/bone rules (`canCollideWith` both ways).
4. **`collapseCollideL`** — a quick collider-tree (`aabbAll`) overlap test between the two bodies before the
   full `processCollision`.
5. **Skeleton-level culling** — `setSkeletonsActive` attaches/detaches whole skeletons (the POV ranking +
   `maxActiveSkeletons`), so inactive actors are **removed from the world** entirely (not in the broadphase).

So far/separated bodies are *already* culled by (1)+(2)+(4), and far actors by (5). `btDbvt` is spatial,
not O(n²), so "two actors far apart" is already handled — generic distance culling is **not** the gap.

*Dead/broken code:* `SkinnedMeshBody::isBoundingSphereCollided(rhs)` is **never called**, and despite its
name it doesn't test spheres — it just returns the `canCollideWith` tag rule. The per-bone
`worldBoundingSphere` data is computed every frame but used **only** to build the AABB.

*Where/when we could cull MORE than now:*
- **(E1) A real body-vs-body bounding-sphere reject.** The per-bone `worldBoundingSphere`s already exist; a
  sphere test is tighter than the axis-aligned markout AABB and cheaper than the `collapseCollideL` tree
  walk. Insert it in `dispatchAllCollisionPairs` *before* `collapseCollideL` to drop AABB-overlapping-but-
  actually-separated pairs without traversing trees. **Modest** — partly redundant with `collapseCollideL`;
  worth it only if the tree walk is expensive enough to pre-empt (measure).
- **(E2) Inter-actor pair cull by POV tier (the real win).** Today *every* pair that passes AABB+tags+tree
  is fully processed, including **cross-actor** pairs in a crowd (one NPC's dress vs another's body). Tag
  each `SkinnedMeshBody` with its owning **actor/skeleton**, and in `dispatchAllCollisionPairs` **skip a
  pair when the two bodies belong to *different* actors and both actors are at a low POV tier** (from A/B's
  ranking) — while keeping **intra-actor self-collision** always. This is collision the engine *does*
  compute today and that E would newly cull, exactly in the crowded, low-POV case where it's least visible.
  That is E's unique value; generic distance is already covered by the existing chain.

*Files:* `src/hdtSkinnedMesh/hdtDispatcher.cpp` (`dispatchAllCollisionPairs` pair filter), a per-body
actor/skeleton id on `SkinnedMeshBody`, and the POV tier from `ActorManager` (shared with A/B).

*Open questions:*
- Is `collapseCollideL` already cheap enough that **E1** buys nothing? (measure — it may be.)
- POV-tier threshold below which inter-actor pairs are dropped; hysteresis to avoid pop-in.
- Cleanest source of the owning-skeleton id on a `SkinnedMeshBody` for the same-actor test.
- Confirm **intra-actor self-collision is never culled** (a character's own cloth must keep colliding).

**F. Occlusion culling (depth-buffer visibility) — the crowd headline.**

*Driving use case:* an exterior battle of dozens of warriors. Everything on-screen is in-frustum, so frustum
culling keeps the whole crowd; Skyrim's exterior occlusion is negligible, so the engine's own cull state
adds nothing outdoors. The one signal that captures **warriors occluding each other** lives in the **GPU
depth buffer** — so a depth-based occlusion test is the *necessary* crowd signal, not a nicety.

*What exists and why it isn't enough:* `isInPlayerView()` ([ActorManager.cpp:1022](src/ActorManager.cpp#L1022))
already does frustum + a single `Actor_CalculateLOS` ray from the actor to the camera. In a crowd that ray
is unreliable — it threads between two front warriors and reports "visible" when the actor is 90 % hidden,
or clips a shield and reports "occluded" when a limb pokes out. It is also **one bool for the whole actor**:
a partly-hidden actor has *every* part simulated. A depth test of the actor's bounding volume is accurate
for the whole-actor case *and* extends to per-part.

*Pipeline:*
1. **Depth hook** — obtain Skyrim's main depth SRV and run a compute pass in the SE/AE renderer. This is the
   hard, version-specific plumbing; **Skyrim Community Shaders demonstrates exactly this** (depth-resource
   access + compute passes). Reimplement from the technique (or vendor, license permitting) — **no runtime
   dependency on SCS**.
2. **Hi-Z pyramid** — build a max-depth mip chain from the depth buffer each frame (a few compute dispatches)
   so a bounding volume can be tested at O(1) against a single mip.
3. **Per-body query** — project each body's bounding volume to NDC, pick a mip by screen-extent, compare the
   volume's nearest depth against the Hi-Z max → one visible/occluded bit. **The input data already exists:**
   per-bone `worldBoundingSphere` and per-body `m_bulletShape.m_aabb` from
   [updateBoundingSphereAabb](src/hdtSkinnedMesh/hdtSkinnedMeshBody.cpp).
4. **Async readback** — write bits to a small buffer, copy to staging, `Map` it 1–2 frames later from a ring.
   Never block the CPU on the GPU.

*Latency & hysteresis:* a cull decision tolerates 1–2 frame-old data. Use **asymmetric hysteresis** — eager
to re-enable (reveal fast), lazy to disable (hide slow), since keeping a hidden part live one extra frame is
nearly free but a pop is very visible — plus a "fresh-in-frustum ⇒ on" guard for fast camera whips.

*Granularity — stable vs volatile (prioritise the stable one):*
- **Whole-actor / back-rank occlusion** — the front rank hides the back ranks, which stay hidden for many
  frames: a **large, stable** cull that also fixes the crude LOS ray. This is most of the savings; build it
  first.
- **Per-part self-occlusion** within a visible actor flickers as warriors weave — a *volatile, incremental*
  layer; tune conservatively or ship later.

*Freeze policy (cheap, already proven):* SMP does **not** fix vertices in world space — each frame the base
vertex positions are the game's skinned pose, and physics only *displaces* them additively. So "freeze an
occluded part" is identical to the **existing skeleton-cull path**: stop applying the physics displacement;
the vertices keep tracking the body via the game's own skinning, losing only the secondary sway. No
worldspace drift, no special kinematic-follow machinery, and resume/settle is the same behaviour skeleton
culling already uses in production. The one added rule: cull a part **only when fully occluded**, so nothing
on the visible silhouette depends on the dropped collision (never cull self-collision a visible edge needs).

*What it does NOT cover:* occlusion only removes "hidden behind another warrior." Two crowd costs remain, and
the occluded bit feeds both — **visible-but-tiny** far warriors (on-screen, un-occluded, a few pixels) →
screen-area / POV LOD (**A**); and the **N² inter-actor collision pairs** (**E2**) → an occluded body drops
its *pairs*, not just its sim. So the maxim "animate what we see" is really "animate what we see *at
meaningful screen size*."

*Files:*
- New GPU module — depth hook, Hi-Z compute, query buffer, async readback ring (+ a compute shader), e.g.
  `src/hdtOcclusionCuller.{h,cpp}` and a `shaders/` entry.
- `src/ActorManager.cpp` — `isInPlayerView` / `setSkeletonsActive` consume the depth-occlusion result
  (replacing/augmenting the single LOS ray) and stamp per-body occluded bits.
- `src/hdtSkinnedMesh/hdtSkinnedMeshBody.h` — per-body `occluded` flag (alongside A's `lodTier`).
- `src/hdtSkinnedMesh/hdtDispatcher.cpp` — skip occluded bodies' pairs (E2 tie-in).

*Open questions (F):*
- Depth-SRV access stability across SE / AE / VR; **SCS license** compatibility if vendoring vs a clean
  reimplementation from the technique.
- **TAA / dynamic-resolution / upscaling**: the depth buffer may be jittered or at a different resolution;
  the query must use the matching depth + view-proj matrices for that frame.
- Conservative mip-selection so a silhouette-edge body is not false-culled (bias toward "visible").
- Whole-actor first vs per-part; hysteresis constants (enter/exit thresholds + dwell).

### How the components combine (one system)

The shared spine is the **POV-coverage ranking** from the culling system. On it:
- **C** shapes the *rank* (view-centeredness, frustum, LOS — already in the culling metric; extra
  importance like combat-target/companion just biases it).
- **B** sets the *band cuts* on that rank, adaptively, to fit `budgetTime` (extends the existing
  `maxActiveSkeletons` auto-adjust into multiple cuts).
- **A** maps each band to a per-body *collision-mesh LOD level* (full → decimated triangle LODs → cull).
- **D** adds an orthogonal *temporal* reduction: its D1 rate-decoupling helps every high-fps user globally,
  and its D3 staggered per-body stride lowers the update rate of the lower bands.
- **E** is a separate *crowd* cull (inter-actor) layered on top.
- **F** supplies the accurate *visibility* signal (depth-buffer occlusion) that sharpens C's rank, drives a
  freeze of fully-hidden parts via A's mechanism, and lets E2 drop occluded pairs — the headline crowd win.

**Build order:** A + C first (mechanism + rank — much of C is already present), then B (make the cuts
adaptive). For the dozens-of-warriors target, **F** is the crowd headline (whole-actor occlusion first),
with **E2** and **D** layered on as crowds still need it.

### Open questions for eng review

- **Does the culling metric capture true apparent *size*?** It currently ranks by `distance / cos(angle)`
  — distance + view-centeredness, but **not** the skeleton's bounding radius. For literal "fraction of
  the POV," a giant far away should outrank a human far away; if that matters, fold the skeleton bounding
  radius into the rank (`coverage ≈ radius / distance`). Decide whether the existing rank is sufficient.
- Band thresholds + hysteresis (separate enter/exit + dwell time) to prevent tier flicker/popping.
- Bodies that ship no LOD chain (a single collision mesh): leave as **T0 → cull** only, or require the
  decimation step to generate LODs for every body?
- LOD-swap continuity: swapping to a coarser/finer mesh moves vertices and can pop a live contact — likely
  needs a settle frame on the swap.
- Interaction with the `m_useBoundingSphere` *contact-proximity* gate (orthogonal to POV LOD, but confirm
  they compose).
- Does the existing bounding-sphere broadphase already cover approach **E**? (measure before building it.)

### Files to modify (approach A, v1)

- `src/ActorManager.cpp` — extend `setSkeletonsActive` to assign a per-skeleton **tier** from POV-coverage
  band thresholds (+ hysteresis) on the ranking it already computes; stamp the tier onto each body.
- `src/hdtSkinnedMesh/hdtSkinnedMeshBody.h` — cached `lodTier` field on `SkinnedMeshBody`.
- `src/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp` — `processCollision(body0, body1, …)` reads each
  body's tier to select its representation.
- `src/hdtSkyrimPhysicsWorld.cpp` — (v2/optional) feed the budget signal for adaptive thresholds (**B**).

**Effort:** M (v1: A with C folded in). Higher if B/D/E are added.

---

## Item 4 — Spatial broad-phase acceleration (vs. the current bone-keyed tree)

**Goal:** Attack the broad-phase pair explosion at its root for dense overlapping meshes.

**Approach:** The current `ColliderTree` groups colliders by **shared skinning bone** (up to
`MaxTreeDepth` levels), not by **position** — which is why dense overlapping meshes still emit huge
pair counts. Introduce a **spatial** acceleration structure (spatial BVH, or a uniform grid over the
mesh AABB) so node-pairs are rejected by position rather than bone membership.

**Files to modify:**
- `src/hdtSkinnedMesh/hdtCollider.h` / `hdtCollider.cpp` — tree build, `checkCollisionL`, `optimize`,
  `remapColliders` (or a parallel spatial structure alongside the bone tree).
- Possibly `src/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp` (dispatch consumes the pairs).

**Risk/open questions:** **Largest effort, highest ceiling.** Significant rewrite of the midphase. Must
preserve the kinematic-pair culling (`dynChild`/`dynCollider`) and the `thread_local`/`isolate`
re-entrancy contract. Skinned meshes deform each frame, so a spatial structure must rebuild/refit per
frame — its build cost must not exceed the pair savings. Only justified if dense collision meshes are a
*common* target, not a one-off.

**Effort:** L (own design doc + scalar-vs-new equivalence tests).

---

# Deferred to another branch — collision-mesh triangle/vertex reduction

These items reduce the collider **count** at the mesh / build level. They belong with the
collision-mesh-reduction effort (candidate branch `copilot/reduce-collision-mesh-vertices`), **not**
`collisionperf`. Captured here for completeness; move to that branch's plan when it's picked up.
Suggested order within that effort: **1.1 → 2.1 → 3 → 2.2**.

## 1.1 — Decimated collision proxy + load-time density diagnostic

**Goal:** Encourage/diagnose low-poly collision meshes decoupled from the render mesh (the dominant,
content-side win — nothing in code makes N triangles free).

**Approach:**
- Authoring guidance (docs): collision meshes should be decimated proxies (target low hundreds of
  triangles), not the visual mesh; prefer sphere (`PerVertexShape`) colliders for volumes.
- Engine aid: emit a **one-time warning** at build when a `PerTriangleShape`'s collider count exceeds a
  configurable threshold, naming the mesh, so authors discover dense collision meshes. Diagnostic only —
  no behaviour change.

**Files to modify:**
- `src/hdtSkinnedMesh/hdtSkinnedMeshShape.cpp` — `PerTriangleShape::finishBuild`: log if
  `m_colliders.size() > kCollisionTriangleWarnThreshold`.
- New docs section (authoring guide) — location TBD by review.

**Risk/open questions:** None functional (logging only). Threshold value needs a sensible default.

**Effort:** XS.

## 2.1 — Prune provably-inert triangles at build

**Goal:** Permanently remove triangles that can never contribute to a contact, shrinking every
per-frame cost with zero observable fidelity change.

**Approach:** Extend the existing `clipColliders` pass (which already drops colliders whose bone weights
are all sub-threshold). Add criteria for triangles that can never collide with **any** partner — e.g.
their bone set has no `canCollideWith` partner under the body's tag/bone collision rules.

**Files to modify:**
- `src/hdtSkinnedMesh/hdtSkinnedMeshShape.cpp` — `SkinnedMeshShape::clipColliders` / `PerTriangleShape::finishBuild`.
- Reads `m_owner->m_skinnedBones`, `canCollideWith` (in `hdtSkinnedMeshBody.h`).

**Risk/open questions (MUST be vetted in eng review):**
- ⚠️ **"All bones kinematic" is NOT a valid prune criterion.** A kinematic triangle is still a valid
  *target* for a dynamic collider from another body (the runtime path uses all of `a`'s colliders when
  `b` is dynamic — only kinematic↔kinematic *pairs* are skipped). Pruning all-kinematic triangles would
  break dynamic-hits-kinematic-surface collisions. The valid criterion is "no possible collision
  partner exists," which requires analysing the collision-tag/bone graph, not just kinematic flags.
- Collision rules can be affected by runtime tag changes (re-equip / `smp reset` rebuilds the tree, so
  build-time pruning is re-evaluated then — acceptable, but confirm).

**Effort:** S–M (correctness analysis is the bulk).

## 2.2 — Drop unreachable / interior / back-facing triangles at build

**Goal:** Remove triangles on the interior of a closed collision mesh that no external collider can
ever reach.

**Approach:** Build-time geometric analysis — e.g. triangles fully enclosed by the mesh hull, or
back-facing relative to all plausible approach directions. One-time build cost.

**Files to modify:**
- `src/hdtSkinnedMesh/hdtSkinnedMeshShape.cpp` — new build-time analysis pass in `finishBuild`.

**Risk/open questions:** Reliable "unreachable" detection is non-trivial and easy to get wrong (skinned
deformation can expose "interior" triangles). High false-positive risk ⇒ dropped real collisions.
Needs a conservative, well-tested heuristic; likely opt-in. **Lower confidence than 2.1.**

**Effort:** M–L.

## 3 — Build-time decimation / clustering of dense collision meshes

**Goal:** When a dense input mesh can't be fixed in content, build a **coarser collider set** from it.

**Approach:** In the build path, merge adjacent near-coplanar triangles into larger triangle colliders
(or voxel/vertex-cluster), producing fewer, larger `Collider`s. Fewer leaves ⇒ fewer pairs ⇒ less skin
update.

**Files to modify:**
- `src/hdtSkinnedMesh/hdtSkinnedMeshShape.cpp` — `PerTriangleShape::addTriangle` / `finishBuild`
  (insert a clustering pass before `m_tree.optimize()` / `exportColliders`).

**Risk/open questions:** Fidelity tradeoff (larger triangles = coarser contacts). Merged triangles need
a consistent bone/weight assignment (skinning of a merged collider is ill-defined when source triangles
have different bones). Define the merge tolerance and bone-merge rule; make aggressiveness configurable.

**Effort:** M.

---

## Validation (applies to every item)

1. Build all four AVX variants (Release); confirm a clean link.
2. Load-matched realistic scene: capture `[SMP Metrics]` impact before/after (un-distorted measure).
3. Visual check: cloth/hair behaves correctly (these touch collider sets / contact generation).
4. For the mesh-reduction items / item 4: confirm no collisions are silently dropped (compare contact
   counts / behaviour vs baseline on the same frame).
5. Sampling-profiler pass (VS CPU Usage or VTune) for items that change hot-path structure (item 4), to
   avoid relying on instrumentation that can distort at high call counts.

## Open risks (cross-cutting)

- `MaxCollisionPairs` (currently 6024) aborts a whole body-pair's collisions when exceeded — count
  reductions (2.1/3, and item 4) *reduce* the chance of hitting it (a correctness win), but verify dense
  meshes weren't silently relying on / tripping it before.
- Tag/bone collision rules can change at runtime via `smp reset` (rebuilds trees) — build-time pruning
  (2.1/2.2) and decimation (3) are re-evaluated then; confirm no stale state.

---

## GSTACK REVIEW REPORT

| Reviewer | Phase | Status |
|----------|-------|--------|
| — | CEO (scope) | pending |
| — | Design | pending |
| — | Eng | pending |
| — | DevEx | pending |

> This PLAN is DRAFT. Per repo policy, it may **not** drive implementation until the Eng phase shows
> "clean." Item 5's approach is chosen (A — decide per skeleton, apply per body; C folded in); its
> remaining open questions are listed under the item. Items 2.1/2.2/4 carry explicit correctness caveats
> the Eng review must resolve first. Open the per-item GitHub issues at approval time.

## Files to Create/Modify (summary, for plan-completion audit)

**This branch (`collisionperf`):**
- `src/ActorManager.cpp` — 5 (actor tier / importance classification)
- `src/hdtSkyrimPhysicsWorld.cpp` — 5 (budget / fidelity selection)
- `src/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp` — 5 (shape selection in dispatch), 4 (pair consumption)
- `src/hdtSkinnedMesh/hdtCollider.h` / `hdtCollider.cpp` — 4 (spatial broad-phase)

**Other branch (collision-mesh reduction):**
- `src/hdtSkinnedMesh/hdtSkinnedMeshShape.cpp` — 1.1 (warn), 2.1 (prune), 2.2 (unreachable), 3 (cluster)
- Authoring docs (location TBD) — 1.1 guidance

---

## Parked questions / notes (revisit later)

- **POV metric: apparent *size* vs distance + angle.** The culling rank used by component A
  (`distance / cos(angle)` in `ActorManager::setSkeletonsActive`) folds in distance and view-centeredness
  but **not** the skeleton's bounding radius — so it isn't literally "fraction of the POV occupied." A
  giant and a human at the same distance/angle currently rank equally. If true apparent size should drive
  the LOD tiers, fold the skeleton bounding radius into the rank (`coverage ≈ radius / distance`, i.e. the
  subtended solid angle). **Decide later** whether the existing rank is a good-enough proxy or needs the
  size term. (Surfaced while designing item 5, component A.)
