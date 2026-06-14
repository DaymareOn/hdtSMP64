# SMP Physics Benchmark & Replay Tool — How it works

This explains, in plain terms, **what** the benchmark is, **how** it works, **what it's good for**,
and — just as important — **what it does not measure**. For the click-by-click build/record/run steps,
see [BENCHMARK_HOWTO.md](BENCHMARK_HOWTO.md).

---

## 1. The one-sentence idea

While you play, the mod can **record** everything the cloth/hair physics does each tick into a file.
Later, a little console program (`smp_replay`) **plays that recording back** and runs the *exact same*
physics — but with no Skyrim and no plugin loaded — so we can time it over and over.

> Same math, no game.

## 2. Why this is even possible

The heavy physics work (Bullet's `stepSimulation`) is self-contained: it does maths on numbers and
never reaches into Skyrim's memory. Skyrim only touches the physics in **four small spots** (reading a
bone's position, writing it back, getting ready to read, and wind). So we draw a clean line right at
those spots: the recording captures the numbers that cross the line, and the replay feeds the same
numbers back into the same code.

To pull that off, the shared physics code (`src/hdtSkinnedMesh/*`) is compiled **twice**:

- into the real mod, against the real Skyrim types, and
- into `smp_replay`, against a tiny **stand-in** ([tests/benchmark/shim/ShimPCH.h](../tests/benchmark/shim/ShimPCH.h))
  that fakes just the handful of Skyrim types the physics code mentions.

Because the tool uses the stand-in instead of the real Skyrim library, the two never collide — yet the
physics code itself is **the same code, character for character**. That's the whole trick.

## 3. How the pieces fit

```
        RECORD (in-game, while you play)              REPLAY (smp_replay.exe, no game)
        ┌──────────────────────────────────┐          ┌──────────────────────────────────┐
  game  │ flip the record switch            │          │ open the file, check it is sane   │
  state │  - snapshot every outfit's physics│  a file  │ rebuild the world from it         │
        │  - log outfits coming/going       │  ───────▶│  for each recorded tick:          │
        │  - per tick: timestep, wind,      │          │   apply the outfit changes        │
        │    where each driver bone moves   │          │   set wind, drive the bones       │
        └──────────────────────────────────┘          │   run the physics tick + time it  │
              the SAME physics code runs               └──────────────────────────────────┘
              on both sides, unchanged
```

### 3.1 The recording file

It's one binary file, described in [src/Replay/hdtReplayFormat.h](../src/Replay/hdtReplayFormat.h), and
**both the recorder and the replay read that one description** so they can't disagree.

| Part | What it holds |
|---|---|
| **Header** | a magic tag, a version number, which build made it, the thread count |
| **Physics settings** | gravity, the solver knobs, the tick length, wind settings, … |
| **Outfit snapshots** | one full snapshot per outfit present when recording began |
| **Outfit log** | outfits appearing/disappearing, tagged with the tick it happened on |
| **Per-tick stream** | one entry per physics tick |

An **outfit snapshot** is everything needed to rebuild that outfit's physics offline: its collision
shapes, its bones, its cloth meshes (the actual collider points), and its constraints (the joints).

A **per-tick entry** holds the time left in the tick, the tick length, the (smoothed) wind, a reset
flag, and where each *driver* bone (the ones the animation moves) is told to go that tick. The
non-driver bones are computed by the physics, so they aren't stored.

**Safety:** a recording is untrusted input. Before believing any count or size in the file, the reader
checks it's sane and that the bytes are actually there, so a broken or hostile file is turned away with
a clean error instead of crashing or being trusted.

### 3.2 The replay side

- **`ReplayBone`** ([tests/benchmark/hdtReplaySystem.h](../tests/benchmark/hdtReplaySystem.h)) is a real
  physics bone that, each tick, reads its recorded target and pushes itself there using the **shared**
  `applyKinematicTarget` — the *same* method the in-game bone uses. That's the guarantee that the
  offline maths matches the game's: it's the same code, not a copy.
- **`ReplaySystem`** rebuilds a whole outfit from a snapshot (the reverse of what the recorder saved).
- **`ReplayWorld`** runs the same per-tick sequence the game does: drive the bones → recentre → run the
  physics → put it back.

### 3.3 The two timing passes

- **Pass 1 — the headline.** The internal profiler is **off** so the timing is clean. Each tick is split
  into three parts — **read** (driving the bones), **step** (the actual physics solve), **write**
  (composing the result) — and reported as **median / p95 / mean / min / max**, plus a **total**, over
  the timed ticks (the first few "warm-up" ticks are thrown away).
- **Pass 2 — the breakdown.** The profiler is **on** and adds up time per labelled chunk inside the
  solve (constraint solving, collision detection, wind, …). It runs single-threaded so the numbers add
  up. **These numbers are inflated by the profiler itself, so they're never the headline** — that's the
  whole reason there are two passes. Use Pass 2 to see *where* the time goes, not *how much*.

## 4. The build/read cost readout (`[Build]`)

Before the timing passes, the tool prints a **`[Build]`** block: how long the game spent *building* each
outfit — reading and parsing the physics XML, pulling skin data out of the NIF, and assembling the
colliders. This is the one-off cost paid when an actor loads in or equips something.

It is **measured in the game** at record time and saved per outfit, then reported here — it is *not*
re-run offline (that build path is wired into the game too deeply to run without it). The block splits
into *startup* outfits (already there when recording began) and *during capture* outfits (loaded
mid-recording). See the limitation note in §6.

## 5. What it's good for

- **Repeatable, no-fuss numbers** for the physics cost — no launching the game, no loading a save, no
  eyeballing a frame counter. Runs in seconds, and in CI.
- **Catching breakage.** `smp_replay --selftest` runs the unit tests on every CI build; with a recorded
  reference, `--check-golden` also checks the physics still produces the same result.
- **Finding hot spots.** The Pass 2 breakdown shows which part of the solve eats the time.
- **Comparing two recordings** — see §6.

## 6. What it does NOT measure (read this before trusting a number)

The tool measures the **physics maths**, not all of Skyrim. So:

- **`read`/`write` are only the physics half.** In the game, reading a bone also reads the scene node,
  and writing pushes back into the scene graph — neither exists offline. So the offline `write` number
  is tiny and is **not** what the game spends there. Read `read`/`write` as "the physics part of that
  step," not "the game's cost."
- **Some "which bodies are on" work isn't replayed yet.** The game decides, each tick, which bodies are
  active (the disable-tag/priority dedup) and that costs time and changes how much collision happens.
  The replay currently assumes **everything is on**, so it can over-count collision work — more so in
  crowded scenes. (Planned: capture the inputs and recompute it.)
- **Build cost is measured, not replayed.** As in §4, the build/read cost is a *recorded* number from
  the game, not an offline re-run. So you can't A/B a build-code change offline — you'd record again.
- **The parity check runs single-threaded.** Multi-threaded physics isn't bit-for-bit repeatable, so the
  "does it still match" check runs on one thread with a small tolerance. The speed headline can still use
  many threads.
- **A few capture details are approximate.** Capsule colliders are rebuilt on one axis; constraints save
  their joints and limits but not every exotic motor/bounce override.
- **A recorded reference file isn't committed yet.** The committed parity check
  (`tests/benchmark/fixtures/parity.bin`) needs a real in-game recording, which has to be produced once.

## 7. Comparing two recordings (FSMP versions, hardware, modlists)

Think of the tool as a **fair experiment**, and a recording as a **frozen input**. A comparison is only
fair if everything *except the one thing you're testing* stays the same. That single idea tells you how
much each kind of comparison is worth:

- Change only the **code** (two FSMP builds) or only the **machine** (two CPUs), and feed **one shared
  recording** to both → the input is identical, so the difference you measure is real.
- Change the **scene** (two modlists) → each side needs its *own* recording, so the input isn't identical
  anymore. The number is a **rough ranking, not a controlled result**.

Rules for any comparison, or you're just measuring noise:

- Compare the same thing on both sides: median vs median, p95 vs p95, and the same `--threads`,
  `--warmup`, `--frames`.
- Use **Pass 1 only**. Pass 2 is profiler-inflated and single-threaded — good for *where* the time goes,
  never for an A-vs-B number.
- It's the **physics cost, not your FPS**. A 20%-faster `step` is 20% off SMP's slice of the frame, not
  20% more frames per second.
- Run each side a few times on a quiet machine, and ignore differences smaller than the run-to-run
  wobble.

### 7.1 Two FSMP versions — ✅ the strongest case

Build `smp_replay` from each version, run **the same recording** through both, compare the `step`
median/p95.

- **Why it's clean:** identical input, so any difference is the code.
- **Keep the same:** machine, the AVX/optimization preset, the command-line flags.
- **One catch:** the reader demands an exact file-version match. If the two versions use different file
  versions, record with a build both can read (or only compare versions that share the format). A version
  that *refuses* the file is an incompatibility, not a result.
- **Not a flaw, just reality:** the recorded targets drive the driver bones identically, but if version B
  solves the free-moving bones differently, they drift apart over the run and the collision work can
  diverge — which is exactly the honest cost of each version on the same input.

### 7.2 Two machines — ✅ clean, and cleaner than comparing FPS

Same **version**, same **build**, same **recording**; run on machine A vs machine B.

- This is the **cleanest CPU comparison the project can give**: no GPU, no disk loading, no game thread —
  just the physics against the CPU and memory. An in-game FPS compare can't isolate that; this can.
- **Single-thread number:** compare `step` median at `--threads 1`. **Scaling:** sweep `--threads` on each
  machine and compare the curves — that answers "do this CPU's extra cores actually help SMP?"
- **Keep the same (this one bites):** the **AVX level must match** (`-avx` / `-avx2` / `-avx512` change the
  maths), as must the compiler/optimization. If both CPUs support the same instructions, run the *same*
  exe on both. Watch out for background load and thermal throttling.

### 7.3 Two modlists — ⚠️ useful, but only as fair as your recording

Different modlists mean different armors, bone counts, and colliders, so each makes a **different
recording** — the input is no longer the same.

- **What it tells you:** "a busy moment of setup A costs about X ms/tick; setup B about Y." Good for
  *ranking* which setup is heavier.
- **What it can't:** a true A-vs-B, because you never replayed the same input.
- **Make it fairer:** record both as alike as you can — same cell, same actors, same camera, a fixed idle
  (no walking/combat), same length.
- **The §6 caveats hit hardest here:** with "everything on" assumed, a crowded modlist's collision cost is
  over-counted, and by a different amount per setup. Treat modlist differences as a direction, not a ratio.

### 7.4 Quick reference

| Comparison | Same recording? | Confidence | Keep the same | Read |
|---|---|---|---|---|
| **FSMP versions** | yes — one shared | high | machine, AVX/opt, flags; file must load on both | `step` median/p95 |
| **Machines** | yes — one shared | high | version, build, AVX, flags | `step` median @ `--threads 1`; sweep `--threads` for scaling |
| **Modlists** | no — one each | rough | same cell/actors/camera/idle/length; same build & flags | `step` median as a *ranking*, not a ratio |

## 8. Where the code lives

| Path | Role |
|---|---|
| [src/Replay/hdtReplayFormat.h](../src/Replay/hdtReplayFormat.h) | the recording file layout + read/write + safety checks (shared by both sides) |
| [src/Replay/hdtReplayCapture.*](../src/Replay/hdtReplayCapture.h) | the in-game recorder + snapshot maker (shared-core only, so it also builds for the tool) |
| [src/hdtSkinnedMesh/hdtSkinnedMeshBone.*](../src/hdtSkinnedMesh/hdtSkinnedMeshBone.cpp) | `applyKinematicTarget` — the shared bone-driving maths |
| [tests/benchmark/shim/ShimPCH.h](../tests/benchmark/shim/ShimPCH.h) | the tiny Skyrim-types stand-in |
| [tests/benchmark/hdtReplaySystem.*](../tests/benchmark/hdtReplaySystem.h) | `ReplayBone` / `ReplaySystem` / `ReplayWorld` |
| [tests/benchmark/replayRunner.*](../tests/benchmark/replayRunner.h) | load + per-tick replay loop + the golden check |
| [tests/benchmark/replayProfiler.*](../tests/benchmark/replayProfiler.h) | the Pass 2 per-chunk profiler |
| [tests/benchmark/main.cpp](../tests/benchmark/main.cpp) | the command line, `--selftest`, the report |
| [tests/benchmark/tests/](../tests/benchmark/tests/) | the unit tests |
| [.github/workflows/benchmark.yml](../.github/workflows/benchmark.yml) | the CI build + tests |
