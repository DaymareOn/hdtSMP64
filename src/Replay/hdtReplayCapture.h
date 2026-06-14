#pragma once

// hdtReplayCapture.h - in-RAM capture buffer + flush-to-disk for the in-game DLL (D9).
//
// Lives only in the game DLL. It accumulates a replay::Document in RAM while capture is enabled
// (snapshot at start, scene-log events on actor churn, one Frame per physics step), enforces a
// frame/size cap that auto-stops capture as a fail-safe, and flushes the whole document to a single
// hand-rolled binary file when capture stops. The standalone smp_replay exe never compiles this
// file - it only reads the result through replay::deserialize() in hdtReplayFormat.h.

#include "hdtReplayConvert.h"
#include "hdtReplayFormat.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"

#include <mutex>

namespace hdt
{
	struct SkinnedMeshBone;
}

namespace hdt::replay
{
	// Bullet <-> POD converters live in hdtReplayConvert.h (shared with the replay side).

	/// Walks one live physics system and writes it into an on-disk Snapshot - the exact inverse of
	/// ReplaySystem::build. The idea: capture enough to rebuild the same world offline, addressing
	/// everything by index instead of by pointer.
	/// Steps:
	///   1. Collision shapes -> a table, de-duplicated by pointer so a shape shared by many bones is
	///      stored once. Compound shapes recurse children-first, so a child's table index always exists
	///      before the parent that references it.
	///   2. Bones -> copied in order, remembering each bone's address->index in a map so bodies and
	///      constraints can refer to bones by that index.
	///   3. Mesh bodies -> vertices, skinning, and the collider (per-vertex or per-triangle) copied out.
	///   4. Constraints -> type + the authored frames (recovered from the baked Bullet offsets) + limits.
	/// Operates purely on shared-core types, so it builds against the shim and is unit-testable.
	Snapshot buildSnapshot(SkinnedMeshSystem& system, uint32_t systemId);

	// Captures the current driver-bone targets + dynamic-bone outputs for one frame. `golden` adds the
	// post-step dynamic-bone poses for the D8 parity fixture.
	void captureFrameBones(const SkinnedMeshSystem& system, uint32_t systemId, Frame& frame, bool golden);

	// Thread-safe-ish capture buffer. The owner (SkyrimPhysicsWorld) is responsible for only
	// touching it under its simulation lock; the internal mutex guards the cap/active flags that
	// may be read from the toggle path.
	class CaptureBuffer
	{
	public:
		// Begins a capture. initialSystems is the scene snapshot taken at enable time (under the
		// simulation lock). frameCap / sizeCap are the fail-safe auto-stop limits (0 == unlimited).
		void begin(const SolverConfig& solver,
			std::vector<Snapshot> initialSystems,
			std::string gitSha,
			uint32_t configHash,
			uint32_t threadCountHint,
			bool golden,
			uint32_t frameCap,
			size_t sizeCap);

		// Records an add/remove scene-log event at the current frame index (D3). Ignored if inactive.
		void addSceneEvent(SceneEvent event);

		// Appends a fully-populated per-frame record. Returns false (and auto-stops) if a cap is hit.
		bool addFrame(Frame frame);

		bool active() const
		{
			std::lock_guard<std::mutex> l(m_lock);
			return m_active;
		}

		uint32_t currentFrameIndex() const
		{
			std::lock_guard<std::mutex> l(m_lock);
			return static_cast<uint32_t>(m_doc.frames.size());
		}

		size_t approxSizeBytes() const
		{
			std::lock_guard<std::mutex> l(m_lock);
			return m_approxSize;
		}

		// Whether a frame/size cap has been reached and capture auto-stopped.
		bool capReached() const
		{
			std::lock_guard<std::mutex> l(m_lock);
			return m_capReached;
		}

		// Serializes the accumulated document to a single binary file. Returns false on IO error.
		// Leaves the buffer intact (call reset() afterwards to free RAM).
		bool flushToFile(const std::string& path) const;

		// Serializes to an in-RAM byte buffer (used by tests / golden generation).
		std::vector<uint8_t> serialize() const;

		void reset();

	private:
		mutable std::mutex m_lock;
		Document m_doc;
		bool m_active = false;
		bool m_capReached = false;
		uint32_t m_frameCap = 0;
		size_t m_sizeCap = 0;
		size_t m_approxSize = 0;
	};
}
