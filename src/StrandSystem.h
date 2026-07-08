#pragma once

#include "FSMP_WigAPI.h"
#include "StrandConfig.h"
#include "StrandSolver.h"

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace hdt
{
	/// A groom: guide strands laid out strand-major in head-local space, plus render material.
	struct StrandGroom
	{
		std::uint32_t strandCount = 0;
		std::uint32_t vertsPerStrand = 0;
		std::vector<btVector3> restLocal;  // head-local, size == strandCount * vertsPerStrand
		std::array<float, 3> colorRoot = { 0.05f, 0.03f, 0.02f };
		std::array<float, 3> colorTip = { 0.14f, 0.09f, 0.05f };
		float roughness = 0.35f;
		float strandRadius = 0.15f;  // world units
	};

	/// Build a synthetic groom -- a hemispherical scalp cap of strands that drape down -- so the
	/// end-to-end pipeline can be exercised without a DCC-authored .tfx. Head-local Skyrim units.
	StrandGroom makeProceduralGroom();

	/// Parse a TressFX .tfx binary into a groom. Returns false (out untouched) on any malformed
	/// input: the 160-byte header is fully range-checked before a single vertex is trusted.
	bool loadTfx(const std::string& path, StrandGroom& out);

	/// One wig on one actor: a groom bound to an anchor (head) node, its CPU solver, a set of
	/// body colliders resolved from named skeleton nodes, and a published position snapshot the
	/// render-side API reads under the manager's publish lock.
	class StrandInstance
	{
	public:
		explicit StrandInstance(const StrandConfig& config);

		/// Resolve the anchor + collider nodes under skeletonRoot and prime the solver.
		/// Returns false if the head node is missing (the manager retries next frame).
		bool bind(RE::NiNode* skeletonRoot);
		bool bound() const { return static_cast<bool>(m_anchor); }

		/// Advance one frame: totalDt split into fixed ticks. Reads the anchor's world transform
		/// and current collider node positions, runs the solver, then refreshes the snapshot
		/// under publishLock. If the anchor has detached (cell reload) it self-unbinds.
		void step(btScalar totalDt, btScalar tick, std::mutex& publishLock);

		void fillDesc(FSMPWigInstanceDesc& d, std::uint32_t id) const;
		std::uint32_t copyPositions(float* dst, std::uint32_t capacityFloats) const;

	private:
		/// A collider anchored to skeleton nodes: sphere at a's origin when b is null, otherwise
		/// a capsule between a's and b's origins. NiPointers keep the nodes alive while bound.
		struct ColliderDef
		{
			RE::NiPointer<RE::NiNode> a;
			RE::NiPointer<RE::NiNode> b;
			btScalar radius;
		};

		/// Build a groom whose roots sit on the actor's real scalp: merge the world bounding
		/// spheres of all geometry under the head bone for the head's true centre + radius, seed
		/// roots over the upper (world-up) hemisphere, and grow each strand down-and-out. Returns
		/// false if the head geometry bound isn't ready yet (caller retries next frame).
		static bool buildScalpGroom(RE::NiNode* headBone, const StrandConfig& config, StrandGroom& out);

		StrandConfig m_config;
		StrandGroom m_groom;
		StrandSolver m_solver;
		StrandParams m_params;
		RE::NiPointer<RE::NiNode> m_anchor;
		std::vector<ColliderDef> m_colliders;
		std::vector<btVector3> m_published;  // guarded by the manager's publish lock
		btTransform m_lastHeadWorld = btTransform::getIdentity();
	};

	/// Owns one wig per SMP-active actor and drives them from the physics step. Singleton.
	class StrandManager
	{
	public:
		static StrandManager& instance();

		void setEnabled(bool e) { m_enabled = e; }
		bool enabled() const { return m_enabled; }

		/// Called from SkyrimPhysicsWorld::doUpdate2ndStep (worker thread, under the sim lock).
		/// Ensures a wig exists for every skeleton in `skeletons` (creating/binding new ones and
		/// dropping wigs whose actor is gone), then steps them all. `skeletons` are the SMP-active
		/// actor skeleton roots, so each head lookup is confined to one actor -- no player guessing.
		void step(btScalar totalDt, btScalar tick, const std::vector<RE::NiNode*>& skeletons);

		/// Request that all wigs be dropped before the next step (e.g. on game load).
		void reset() { m_resetRequested = true; }

		// --- render-thread snapshot access, all guarded by m_publishLock ---
		std::uint32_t instanceCount();
		std::uint32_t getDesc(std::uint32_t index, FSMPWigInstanceDesc* out);
		std::uint32_t copyPositions(std::uint32_t index, float* dst, std::uint32_t capacityFloats);

	private:
		StrandManager();

		std::atomic_bool m_enabled{ false };
		std::atomic_bool m_resetRequested{ false };
		StrandConfig m_config;  // author-editable groom/sim params, (re)loaded from wig.xml
		// One wig per actor, keyed by that actor's skeleton root node.
		std::unordered_map<RE::NiNode*, std::unique_ptr<StrandInstance>> m_instances;
		// Stable index order for the render-side API, rebuilt each step under m_publishLock.
		std::vector<StrandInstance*> m_order;
		std::mutex m_publishLock;  // protects m_instances/m_order shape + each instance's snapshot
	};
}
