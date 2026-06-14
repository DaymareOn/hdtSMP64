# Replay harness fixtures

This directory holds committed capture fixtures used by the CI parity gate (PLAN D8/§15).

## `parity.bin` (seam-parity golden) — not yet committed

The parity gate replays a real in-game capture single-threaded and asserts every dynamic bone's
output matches the captured golden within tolerance. That requires a capture **recorded in-game**
(the harness cannot synthesize a physically meaningful one), so the binary is produced once and
committed:

1. In-game, enable replay capture with golden recording via the Papyrus toggle
   (`DynamicHDT.SetReplayCapture(path, frameCap, sizeCap, golden = true)`), drive some physics
   (e.g. let an actor with SMP-enabled gear move for a few seconds), then disable it to flush.
2. Capture it **single-threaded** if possible (or accept the configured tolerance), so the golden is
   reproducible by the single-threaded replay (multithreaded Bullet is not bitwise-deterministic —
   §12).
3. Copy the flushed file here as `parity.bin` and commit it.

Once `parity.bin` exists, `tests/benchmark/CMakeLists.txt` automatically registers the
`smp_replay_parity` CTest, which runs:

```
smp_replay fixtures/parity.bin --threads 1 --check-golden
```

and fails the build on any golden mismatch.

## Smoke-testing without a fixture

The harness self-test (`smp_replay --selftest`, always run in CI) covers the format, the
applyKinematicTarget seam, the capture↔replay snapshot round-trip, scene-log churn, an end-to-end
synthetic replay, and determinism — none of which need a committed binary.

To eyeball the perf readout on a synthetic capture you can also point `smp_replay <file>` at any
capture produced in-game.
