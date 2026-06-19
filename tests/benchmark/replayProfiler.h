#pragma once

// replayProfiler.h - per-scope profiler for the harness Pass 2 breakdown (§11).
//
// Bullet's own CProfileManager emits nothing in this vcpkg build, but every BT_PROFILE scope still
// routes through the settable btEnterProfileZone/btLeaveProfileZone hook. We install our own hook so
// the breakdown captures both the harness phases (replay_readTransform / replay_step /
// replay_writeTransform) and the core's internal scopes (performDiscreteCollisionDetection,
// solveConstraints, applyWind, ...). Not thread-safe by design: the profiling pass runs
// single-threaded so the enter/leave stack has no races.

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace hdt::replay::prof
{
	// One profiled scope in the call tree. The tree mirrors the BT_PROFILE enter/leave nesting: a
	// scope becomes a child of whatever scope was open when it was entered, so the same name under two
	// different parents stays two distinct nodes (this is what makes a flamegraph possible - the flat
	// text report aggregates by name, but the pprof export needs the nesting). totalNs is INCLUSIVE
	// wall time (this scope plus everything called inside it, summed over every call); self-time is
	// derived later as totalNs minus the children's totalNs. unique_ptr children keep node addresses
	// stable while the tree grows under us mid-capture (the open-scope stack holds raw pointers).
	struct ScopeNode
	{
		std::string name;
		std::uint64_t totalNs = 0;
		std::uint64_t calls = 0;
		std::vector<std::unique_ptr<ScopeNode>> children;

		explicit ScopeNode(std::string a_name) :
			name(std::move(a_name)) {}

		/// Returns the child with this name, creating it on first use (linear scan; the per-scope fan-out
		/// is tiny, a handful of names, so a map would not pay for itself).
		ScopeNode& child(std::string_view a_name);
	};

	// Installs the custom profile-zone hook and enables profiling (BT_PROFILE becomes live).
	void install();

	// Disables profiling (BT_PROFILE scopes go back to no-ops).
	void uninstall();

	// Clears the accumulated call tree.
	void reset();

	// Prints the accumulated scopes to stdout, sorted by total time descending.
	void report();

	// The accumulated call tree (root is a synthetic "Root" whose children are the top-level scopes).
	// Valid after a profiled run, until the next reset(); used by the pprof export.
	const ScopeNode& tree();
}
