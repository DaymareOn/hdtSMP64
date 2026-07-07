# World collision — review findings & <5ms roadmap (#394)

Goal: detection + collider add/remove + collision for ALL smp-enabled actors, total FSMP
cost < 5 ms/frame. Source from a 6-reader adversarial review (2026-07). Costs are estimates
from the cost model; validate the collision-cost ones with the offline benchmark.

## Cost model (current)
Per frame ≈ `A×0.05ms (LOS rays) + S×[ Σ_obstruction F(V) + Σ_nearActor (1ns × P × (T+V)) ]`.
One 100k-vert / 200k-tri obstruction + 1 actor on it ≈ **5–6 ms** (whole budget), +~4 ms per
extra nearby actor. Re-crop while walking = **30–60 ms main-thread hitch, ~2×/s**.

Dominant terms:
- **Flat single-leaf ColliderTree** — obstruction triangles are all bound to ONE bone, so the
  tree collapses to one leaf; each actor-leaf pair linearly scans ALL T triangle AABBs. ~4 ms/actor.
- **Static kinematic body re-skinned every substep** — O(V) skin + O(T) AABB/normal rebuild
  despite never moving. ~0.7–1.4 ms/obstruction/substep.
- **Broadphase AABB = full 2M-mesh modelBound** — every actor in the cell pairs with the
  obstruction even 4000 units away, forcing the above.
- **Re-crop = O(2M) main-thread scan + rebuild under the physics lock**, 2×/s.

## Correctness bugs (feature silently doesn't work)
- **Giant-triangle drop** — crop keeps a triangle only if a VERTEX is in the 158u sphere; a big
  floor triangle with all 3 verts >158u away is dropped → actor has NO floor while overlay says OK.
  Fix: sphere-vs-triangle (closest-point) test. [impl-bugs + lifecycle]
- **Empty-crop churn** — zero kept triangles → addObstruction pops the entry, destroying the
  just-filled O(2M) GPU-read cache; re-reads every ~6 frames forever. Fix: keep entry+cache,
  mark "empty at builtAt", only retry on move. [lifecycle]
- **Owner-only crop** — only the owner actor's surroundings get a collider; every other actor in
  the cell collides with nothing. Fatal for the all-actor goal. Fix: per-actor crops from a shared
  cache, or union-of-spheres crop with an interested-actor set. [detection + cost-model]
- **Cache destroyed on 90-frame timeout** — a probe gap (occlusion / budget / player-only) expires
  the obstruction and its ~36MB cache → next hit re-reads GPU. Fix: decouple cache lifetime from
  collider lifetime (LRU keyed by BSTriShape*). [lifecycle]
- **Glow never renders** — factory-built TESEffectShader sets kGreyscaleToColor with no membrane
  palette texture → fill samples black → invisible under additive blend; form also skips
  Load/InitItemImpl. Fix: clone a vanilla Skyrim.esm EFSH's data+textures (LookupForm / GetFormArray),
  recolor edge; or drop kGreyscaleToColor and drive color from the fill color keys. [glow]

## Detection redesign (replaces 6-axis LOS rays)
World geometry never moves, so discovery is a distance test, not a raycast. On
TESCellAttachDetachEvent, enumerate the cell's Static/MovableStatic/Tree/Furniture references
(TESObjectCELL references under spinLock) → cache {NiPointer<NiAVObject> root3D, worldBound}.
Per frame per actor, sphere-test candidates: A=10 × C=3000 ≈ 0.03–0.06 ms, every direction,
every frame, no ray blindness. Enter/exit hysteresis replaces the probe-driven timeout.

## Ordered plan
- **Stage 1 (correctness, no core-physics risk):** sphere-vs-triangle crop; no-churn on empty/failed
  crop; tight broadphase AABB from crop bounds (not modelBound); gate debug colliderTris capture
  behind viz; cache parsed obstruction.xml template; skip updateTransformUpDown for static objects.
- **Stage 2 (steady-state <5ms; validate offline):** one-time uniform grid over the cached mesh so
  re-crop is O(patch); spatial ColliderTree keys for obstruction triangles (kills flat-leaf O(P×T));
  skip per-substep re-skin of static kinematic bodies. Benchmark each via a synthetic obstruction body.
- **Stage 3 (all-actor correctness):** candidate-cache detection + per-actor / union crop; retire LOS rays.
- **Stage 4:** off-thread build + step-boundary swap queue to remove the remaining re-crop hitch.

## Benchmark (offline, no game)
Only `src/hdtSkinnedMesh/*` compiles against the shim (ShimPCH). Build a synthetic `replay::Document`
(template: `tests/benchmark/tests/test_replay_pipeline.cpp`) = 1 armor body + 1 kinematic per-triangle
obstruction body (50k-vert patch) → measure per-frame collision cost (proves the flat-leaf vs
spatial-key fix) and add-system cost. Crop math must be extracted into an engine-free function to be
benchmarked directly. Benchmark suite lives on the `benchmarksuite` branch — cherry-pick
`tests/benchmark/`, `src/Replay/`, CMake hunks into `world-collision-perf`.
