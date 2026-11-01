#pragma once

#include "hdtBulletHelper.h"
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace hdt
{
	struct SkinnedMeshBone;

	/// What to record, read from the request file dropped into the trace directory.
	struct TraceRequest
	{
		std::string name;    ///< Output file stem, restricted to [A-Za-z0-9_-].
		std::string filter;  ///< Only bones whose name contains this text are recorded; empty records all bones.
		uint32_t steps = 0;  ///< Number of internal fixed steps to record.
	};

	/**
	 * @brief Records the world's bodies, constraints and per-step state to disk, so an external solver can replay them.
	 * @details Armed by a JSON request file (`Data/SKSE/Plugins/hdtSMP64/trace/request.json`, fields `name`, `steps`,
	 *          `filter`). Writes `<name>.json` (bodies, constraints, solver settings, layout) and `<name>.bin` (one
	 *          record per internal step: pre-step and post-step body state). Collision is not recorded. Every call
	 *          comes from the physics thread that owns the world.
	 */
	class TraceRecorder
	{
	public:
		/// Returns true and fills a_request when a request file was found; checks the disk only every few calls.
		bool pollRequest(TraceRequest& a_request);

		/// Starts recording a_bodies and the constraints joining them; a_constraints outside the body set are skipped.
		bool begin(const TraceRequest& a_request, std::vector<RE::BSTSmartPointer<SkinnedMeshBone>> a_bodies, const btAlignedObjectArray<btTypedConstraint*>& a_constraints, const btContactSolverInfo& a_solverInfo, const btVector3& a_gravity);

		bool active() const { return m_active; }

		/// Marks the start of a stepSimulation call; a_offset is the recentring offset the caller applied to every body, so positions can be chained across frames.
		void beginFrame(const btVector3& a_offset)
		{
			m_frameOffset = a_offset;
			++m_frame;
		}

		/// Records the state a fixed step of a_timeStep starts from.
		void recordPre(float a_timeStep);

		/// Records the state the step just recorded by recordPre produced, and finishes the trace after the last step.
		void recordPost();

	private:
		void finish();
		void writeHeader() const;

		bool m_active = false;
		uint32_t m_pollCounter = 0;
		uint32_t m_stepsRecorded = 0;
		uint32_t m_frame = 0;
		TraceRequest m_request;
		btVector3 m_frameOffset{ 0, 0, 0 };
		btVector3 m_gravity{ 0, 0, 0 };
		btContactSolverInfo m_solverInfo;
		std::vector<RE::BSTSmartPointer<SkinnedMeshBone>> m_bones;
		std::vector<btTypedConstraint*> m_constraints;
		std::unordered_map<const btRigidBody*, int> m_bodyIndex;
		std::vector<btVector3> m_bodyGravity;
		std::ofstream m_bin;
	};
}
