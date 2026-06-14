#pragma once

// replayRunner.h - the replay loop shared by the smp_replay main() and the pipeline self-test.
// Loads/holds a rebuilt world and steps it frame-by-frame exactly as the game does (§10), applying
// scene-log churn (D3) and recording per-frame wall-clock for the perf readout (T5/§11).

#include "Replay/hdtReplayFormat.h"
#include "hdtReplaySystem.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace hdt::replay
{
	// A rebuilt, ready-to-step world. Non-copyable/non-movable (it owns a Bullet world): construct in
	// place and pass by reference.
	struct LoadedWorld
	{
		ReplayWorld world;
		// systemId -> live system (smart pointer keeps it alive between an add and its remove event)
		std::map<uint32_t, RE::BSTSmartPointer<ReplaySystem>> systems;

		LoadedWorld() = default;
		LoadedWorld(const LoadedWorld&) = delete;
		LoadedWorld& operator=(const LoadedWorld&) = delete;
	};

	// Reads an entire file into memory. Throws ReplayFormatError on IO failure (fail closed).
	std::vector<uint8_t> readFile(const std::string& path);

	// Sets MXCSR FTZ/DAZ to match the game (hdtSkyrimPhysicsWorld.cpp:97), required for single-thread
	// parity (§12). Call once before stepping.
	void setFlushToZero();

	// Applies the solver/gravity/wind config and adds the initial systems to the world.
	void initWorld(LoadedWorld& lw, const Document& doc);

	// Applies any scene-log add/remove events scheduled at frameIndex (D3).
	void applySceneEvents(LoadedWorld& lw, const Document& doc, uint32_t frameIndex);

	// Per-frame wall-clock (ms) for each phase of the step, one entry per timed frame (index >=
	// warmup). These measure the PHYSICS-CORE portion of each phase only; the Skyrim-side cost of
	// read/write (NiNode I/O, updateTransformUpDown scene-graph propagation) does not exist offline.
	//   read  - walk-up: applyKinematicTarget driving the bones to the captured targets
	//   step  - offset recentre + stepSimulation (and updateActiveState once it is ported here)
	//   write - walk-off: composing each bone's output transform
	struct PhaseTimings
	{
		std::vector<double> read;
		std::vector<double> step;
		std::vector<double> write;

		std::size_t timedFrames() const { return step.size(); }
	};

	// Steps the world over the capture's frames (looping the stream if maxFrames exceeds it). Each
	// phase is wrapped in a BT_PROFILE scope; those are live only when prof::install() has enabled
	// profiling (the Pass 2 breakdown), and are no-ops otherwise (the Pass 1 headline pays nothing).
	PhaseTimings runReplay(LoadedWorld& lw, const Document& doc, std::size_t maxFrames, int warmup);

	// Result of the seam-parity check (D8): how many golden dynamic-bone outputs were compared and how
	// many drifted past the tolerance, plus the worst per-bone position error seen.
	struct GoldenResult
	{
		std::size_t comparisons = 0;
		std::size_t mismatches = 0;
		double maxError = 0.0;
	};

	// Replays one pass over the capture and, for each frame carrying golden outputs, compares every
	// dynamic bone's post-writeTransform pose against the captured golden within tolerance (Skyrim
	// world units). Used by the committed CI parity gate; run single-threaded for determinism (§12).
	GoldenResult runReplayCheckGolden(LoadedWorld& lw, const Document& doc, float tolerance);
}
