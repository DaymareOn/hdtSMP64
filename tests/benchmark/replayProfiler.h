#pragma once

// replayProfiler.h - per-scope profiler for the harness Pass 2 breakdown (§11).
//
// Bullet's own CProfileManager emits nothing in this vcpkg build, but every BT_PROFILE scope still
// routes through the settable btEnterProfileZone/btLeaveProfileZone hook. We install our own hook so
// the breakdown captures both the harness phases (replay_readTransform / replay_step /
// replay_writeTransform) and the core's internal scopes (performDiscreteCollisionDetection,
// solveConstraints, applyWind, ...). Not thread-safe by design: the profiling pass runs
// single-threaded so the enter/leave stack has no races.

namespace hdt::replay::prof
{
	// Installs the custom profile-zone hook and enables profiling (BT_PROFILE becomes live).
	void install();

	// Disables profiling (BT_PROFILE scopes go back to no-ops).
	void uninstall();

	// Clears accumulated per-scope stats.
	void reset();

	// Prints the accumulated scopes to stdout, sorted by total time descending.
	void report();
}
