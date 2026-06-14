# SMP Physics Benchmark — How to use it

A hands-on guide to building, recording, and running the `smp_replay` tool. For *how it works* and
*what the numbers mean*, see [BENCHMARK_ANALYSIS.md](BENCHMARK_ANALYSIS.md).

There are two things you can do:

1. **Replay a real recording** — record one while playing, then time it offline as often as you like.
2. **Run the self-test** — no recording needed; just checks the tool and the physics code still work.
   This is what CI runs on every push.

---

## 1. What you need

- Windows, the same MSVC compiler used to build the mod, and CMake ≥ 3.22.
- The project's vcpkg dependencies (Bullet, TBB, and `doctest`) — these come in automatically when you
  configure the project.
- CommonLibSSE is needed to *configure* the project (the mod itself needs it), but the `smp_replay`
  program only uses Bullet + TBB + doctest.

## 2. Build the tool

The tool builds **automatically with the mod** (the `BUILD_BENCHMARK` switch is on by default), and the
build drops `smp_replay.exe` right next to `hdtSMP64.dll` in the same `SKSE/Plugins/` folder the DLL
goes to. So just build the mod the usual way:

```powershell
cmake --preset vs2022-windows-avx2
cmake --build out/build/vs2022-windows-avx2 --config Release
```

Afterwards you'll find `smp_replay.exe` in two places: in the build tree
(`out/build/vs2022-windows-avx2/tests/benchmark/Release/`), and — because the build copies it — next to
the DLL in your `SKSE/Plugins/` folder. The released mod ships it the same way, so anyone who installs
the mod gets the tool too. Swap the preset name for your platform (`vs2022-windows`, `-avx`, `-avx2`,
`-avx512`).

Don't want it? Turn the switch off for a leaner, DLL-only build:
`cmake --preset vs2022-windows-avx2 -DBUILD_BENCHMARK=OFF`.

## 3. Run the self-test (no recording needed)

```powershell
# via CTest
ctest --test-dir out/build/vs2022-windows-avx2 -C Release --output-on-failure

# or directly
.\out\build\vs2022-windows-avx2\tests\benchmark\Release\smp_replay.exe --selftest
```

You should see:

```
[doctest] test cases: 15 | 15 passed | 0 failed | 0 skipped
[doctest] Status: SUCCESS!
```

These check the recording file (round-trip + the safety checks), the shared bone-driving maths, the
record-then-replay round-trip, a full run with collision, outfits coming and going, and that two runs
match. A non-zero exit means something broke.

## 4. Record while playing

### Easiest: the `smp record` console command

Open the console (`~`), then type:

```
smp record [seconds] [max_mb]
```

- It **arms** the recording and tells you where the file will be saved, then **starts on its own when
  you close the console** (so the console itself isn't recorded).
- It **stops on its own** after `seconds` (default **10**) or once the file would reach `max_mb`
  (default **256**), whichever comes first, then saves.
- The file is a time-stamped `hdtSMP_capture_<timestamp>.bin` next to the plugin log
  (`Documents/My Games/Skyrim Special Edition/SKSE/`). The armed message prints the exact path, and a
  second message prints when it's done.

Example — record about 15 seconds, stop early if it hits 128 MB:

```
smp record 15 128
```

Close the console and play; you'll see "Replay recording started." and, at the end, "Replay recording
finished … Saved to: …".

### For scripts: the Papyrus switch

For automated recording (an MCM toggle, a test quest), call the global script **`DynamicHDT`**:

```papyrus
; signature
bool Function SetReplayCapture(bool enabled, string path, int frameCap, bool golden) global native
```

```papyrus
; START recording to a file (frameCap 0 = no limit).
DynamicHDT.SetReplayCapture(true, "C:/Temp/smp_capture.bin", 600, false)
; ... let the physics run ...
; STOP and save.
DynamicHDT.SetReplayCapture(false, "", 0, false)
```

Good to know (both ways):
- **`path`** is taken when you start; the file is written when you stop. Use a full path you can write
  to (a relative path is relative to Skyrim's working folder).
- **The frame/size limit is a safety net.** The recording is held in memory and stops itself at the
  limit so a forgotten recording can't grow forever. `smp record` then saves automatically; the Papyrus
  way saves when you call `SetReplayCapture(false)`.
- **`golden = true`** (Papyrus only) also records where every free-moving bone ended up each tick — only
  needed to make a reference file for the parity check (§6). Leave it off for plain speed recordings.
  (`smp record` always makes a plain, non-golden recording.)
- For a parity reference, record with the game pinned to a single physics thread if you can, so the
  single-threaded replay can reproduce it exactly (multi-threaded physics isn't bit-for-bit stable).

Copy the resulting `.bin` to wherever you'll run the tool.

## 5. Replay it and read the numbers

```powershell
smp_replay.exe C:\Temp\smp_capture.bin --threads 1 --warmup 30
```

Output:

```
loaded C:\Temp\smp_capture.bin
  format version : 3
  git sha        : dev-abc1234
  capture threads: 8
  initial systems: 12
  scene-log evts : 3
  frames         : 600
  replay threads : 1

[Build] captured in-game build/read cost — XML parse + NIF skin extraction + construction
  (measured in-game; the build path is engine-coupled, so it is recorded, not re-run offline)
  startup        12 systems | total 18.400 ms | median 1.2000 | p95 3.1000 | max 4.5000
  during capture 3 systems | total 5.100 ms | median 1.6000 | p95 2.7000 | max 2.9000

[Pass 1] headline ms/frame (over 570 timed frames, 30 warmup discarded)
  (read/write are the physics-core halves only; Skyrim NiNode I/O is not present offline)
  read   median 0.0123 | p95 0.0150 | mean 0.0125 | min 0.0110 | max 0.0300
  step   median 1.8400 | p95 2.4100 | mean 1.9000 | min 1.6000 | max 3.9000
  write  median 0.0040 | p95 0.0060 | mean 0.0042 | min 0.0035 | max 0.0120
  total  median 1.8560 | ...

[Pass 2] per-scope breakdown (single-threaded; total time per BT_PROFILE scope)
  scope                                  total ms      calls      ms/call
  solveConstraints                        ...
  replay_step                             ...
  performDiscreteCollisionDetection       ...
  ...
```

### Reading it

**The header** just echoes the recording: how many outfits/ticks, how many threads the *game* used
when recording vs how many the *replay* used, and which build made it.

**The `[Build]` block** is the one-off cost of *building* each outfit in-game — reading and parsing the
physics XML, pulling skin data from the NIF, and assembling the colliders. It's **measured in the game**
and saved (it can't be re-run offline), split into *startup* (outfits already there when recording
began) and *during capture* (outfits that loaded mid-recording — an actor walking in or equipping). Use
it to gauge load/equip hitches; it's separate from the per-tick cost below.

**Pass 1 (the headline — the number you compare):**
- Each tick is split into three parts:
  - **`read`** — driving the bones to where the recording says.
  - **`step`** — the actual physics solve. **This is almost always the number that matters.**
  - **`write`** — composing each bone's result.
  - **`total`** = read + step + write.
- **`median`** is your headline; **`p95`** shows the worst typical ticks (spikes); `min`/`max` bound the
  range. Prefer `median`/`p95` over `mean` for frame-time work.
- **`--warmup N`** throws away the first N ticks so cold-start costs don't skew the numbers. Bump it if
  the first ticks look noisy.
- **Heads up:** `read` and `write` are only the *physics* half. The game's cost of reading/writing the
  scene graph doesn't exist offline, so offline `write` is tiny and is *not* the game's real cost. Treat
  `read`/`write` as "the physics part of that step." (More in [BENCHMARK_ANALYSIS.md §6](BENCHMARK_ANALYSIS.md).)

**Pass 2 (the breakdown — for finding hot spots, not for headline numbers):**
- Each row is a labelled chunk of the solve: total time, how many times it ran, and time per call.
- It runs **single-threaded with the profiler on**, so the absolute times are **inflated**. Use it to
  see *where* the time goes and the *relative* sizes — never quote a Pass 2 number as the result. Pass 1
  is the result.

### Comparing builds or machines

Run the **same recording** through two builds and compare the Pass 1 `step` median/p95. To see how well
it uses multiple cores, vary `--threads` (the headline honours it; Pass 2 is always single-threaded).
For the full do's and don'ts, see [BENCHMARK_ANALYSIS.md §7](BENCHMARK_ANALYSIS.md).

To compare two different **in-game setups** (e.g. different culling), record one for each and compare
them — those are capture-time choices, so each is its own recording.

## 6. Parity check (did the physics stay correct?)

If you recorded with `golden = true`, you can check the offline replay still produces the same
free-moving-bone result the game did:

```powershell
smp_replay.exe C:\Temp\parity.bin --threads 1 --check-golden --tolerance 0.01
```

Output:

```
[parity] 34200 comparisons | 0 mismatches | max error 0.004100 | tolerance 0.010000
PARITY OK
```

It exits non-zero on any mismatch. `--tolerance` is in Skyrim world units; run single-threaded so it's
repeatable. To make CI check this, drop the recording at `tests/benchmark/fixtures/parity.bin` —
[tests/benchmark/CMakeLists.txt](../tests/benchmark/CMakeLists.txt) turns on a parity test when that
file exists. See [tests/benchmark/fixtures/README.md](../tests/benchmark/fixtures/README.md).

## 7. Command-line options

| Argument | Meaning |
|---|---|
| `<capture.bin>` | path to a recording made in-game |
| `--frames N` | how many ticks to run; `0` = the whole recording; loops it if `N` is larger |
| `--threads N` | how many worker threads; `1` = single-threaded (use for parity) |
| `--warmup N` | throw away the first `N` timed ticks from the headline (default `0`) |
| `--check-golden` | parity mode: compare against the recorded result; non-zero exit on mismatch (needs a `golden=true` recording) |
| `--tolerance F` | parity tolerance in world units (default `0.01`) |
| `--selftest` | run the unit tests and exit |

Exit codes: `0` success / parity OK; `1` load error, empty recording, or parity failure; `2` bad
arguments.

## 8. CI

[.github/workflows/benchmark.yml](../.github/workflows/benchmark.yml) builds `smp_replay` and runs the
tests on every push (the self-test always; the parity check when `fixtures/parity.bin` exists). The mod
build itself also builds the tool (it's on by default) and ships it alongside the DLL — see §2.

## 9. If something goes wrong

- **`bad file magic` / `unsupported format version` / `unexpected end of file`** — the file isn't a
  valid recording, or was made by a different file version. Record again with a matching build.
- **`--check-golden requires a capture recorded with golden outputs`** — record again with
  `golden = true`.
- **Pass 2 prints nothing** — nothing was timed (e.g. a zero-tick recording). Check the header's
  `frames` count.
- **`write` looks impossibly small** — that's expected; see the read/write note above.
- **`smp_replay` target not found** — you didn't configure with `-DBUILD_BENCHMARK=ON`.
