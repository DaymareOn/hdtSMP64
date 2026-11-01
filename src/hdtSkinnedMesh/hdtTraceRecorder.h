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
		uint32_t steps = 0;  ///< Number of internal fixed steps to record; capped at 36000.
		bool stop = false;   ///< `"stop": true` or `"steps": 0`: ends the running trace; the other fields are unused.
	};

	/**
	 * @brief Records the world's bodies, constraints and per-step state to disk, so an external solver can replay them.
	 * @details Armed by a JSON request file (`Data/SKSE/Plugins/hdtSMP64/trace/request.json`, fields `name`, `steps`,
	 *          `filter`). Writes `<name>.json` (bodies, constraints, solver settings, layout) and `<name>.bin` (one
	 *          record per internal step: pre-step and post-step body state). Collision is not recorded. Calls
	 *          come from the world's step, under its lock.
	 */
	class TraceRecorder
	{
	public:
		/// Returns true and fills a_request when a request file was found. While a trace runs only a stop request is consumed.
		bool pollRequest(TraceRequest& a_request);

		/// Starts recording the bodies matching the request filter and the constraints that touch them, plus the body on
		/// the other end of each such constraint, marked as a boundary body. Returns false and records nothing on failure.
		bool begin(const TraceRequest& a_request, std::vector<RE::BSTSmartPointer<SkinnedMeshBone>> a_bodies, const btAlignedObjectArray<btTypedConstraint*>& a_constraints, const btContactSolverInfo& a_solverInfo, const btVector3& a_gravity);

		bool active() const { return m_active; }

		/// Ends the running trace, keeping what was recorded.
		void stop();

		/// The world's wind for the coming stepSimulation; call before beginFrame.
		void setWorldState(bool a_windEnabled, const btVector3& a_windSpeed, float a_windTime);

		/// Starts a stepSimulation call; a_offset is the recentring offset applied to every body.
		void beginFrame(const btVector3& a_offset)
		{
			m_frameOffset = a_offset;
			++m_frame;
		}

		/// Records the state a fixed step starts from.
		void recordPre(float a_timeStep);

		/// Records the state the step produced; finishes the trace after the last step.
		void recordPost();

	private:
		void finish(const char* a_reason);
		void reset();
		void snapshot(const std::vector<btTypedConstraint*>& a_constraints);
		bool writeHeader() const;
		std::string describeBody(const SkinnedMeshBone& a_bone, bool a_boundary) const;
		std::string describeConstraint(btTypedConstraint& a_constraint) const;

		bool m_active = false;
		uint32_t m_pollCounter = 0;
		uint32_t m_stepsRecorded = 0;
		uint32_t m_frame = 0;
		TraceRequest m_request;
		btVector3 m_frameOffset{ 0, 0, 0 };
		btVector3 m_gravity{ 0, 0, 0 };
		bool m_windEnabled = false;
		btVector3 m_windSpeed{ 0, 0, 0 };
		float m_windTime = 0;
		struct WindState
		{
			bool enabled = false;
			btVector3 speed{ 0, 0, 0 };
			float time = 0;
		};
		WindState m_startWind;
		const char* m_ended = "running";
		btContactSolverInfo m_solverInfo;
		std::vector<RE::BSTSmartPointer<SkinnedMeshBone>> m_bones;
		std::vector<bool> m_boundary;
		std::unordered_map<const btRigidBody*, int> m_bodyIndex;
		std::vector<std::string> m_bodiesJson;
		std::vector<std::string> m_constraintsJson;
		size_t m_constraintCount = 0;
		std::ofstream m_bin;
	};
}
