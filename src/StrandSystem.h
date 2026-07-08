#pragma once

#include "FSMP_WigAPI.h"
#include "StrandConfig.h"
#include "StrandSolver.h"

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
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
		float strandRadius = 0.4f;  // ribbon half-width base, world units (matches StrandConfig::width)
	};

	/// Parse a TressFX .tfx binary into a groom. Returns false (out untouched) on any malformed
	/// input: the 160-byte header is fully range-checked before a single vertex is trusted.
	bool loadTfx(const std::string& path, StrandGroom& out);

	/// One wig on one actor: a groom bound to an anchor (head) node, its CPU solver, a set of
	/// body colliders resolved from named skeleton nodes, and a published position snapshot the
	/// render-side API reads under the manager's publish lock.
	class StrandInstance
	{
	public:
		/// authoredGroom is a shared, immutable .tfx groom for this wig (from StrandManager's cache),
		/// or null to fall back to the procedural scalp cap. Its geometry is copied per instance at
		/// bind(); render material always comes from `config`, never the shared groom.
		StrandInstance(const StrandConfig& config, std::shared_ptr<const StrandGroom> authoredGroom);

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
		std::shared_ptr<const StrandGroom> m_authoredGroom;  // null -> procedural scalp cap
		StrandGroom m_groom;
		StrandSolver m_solver;
		StrandParams m_params;
		RE::NiPointer<RE::NiNode> m_anchor;
		std::vector<ColliderDef> m_colliders;
		std::vector<btVector3> m_published;  // guarded by the manager's publish lock
		btTransform m_lastHeadWorld = btTransform::getIdentity();
	};

	/// One SMP-active actor as seen by the wig driver: its skeleton root (the wig anchor + map key),
	/// plus the actor and worn-wig formIDs stamped on its SMP system. formIDs are 0 when unknown.
	struct StrandActor
	{
		RE::NiNode* root = nullptr;
		std::uint32_t actorFormID = 0;
		std::uint32_t wigFormID = 0;
	};

	/// Owns one wig per SMP-active actor and drives them from the physics step. Singleton.
	class StrandManager
	{
	public:
		static StrandManager& instance();

		void setEnabled(bool e) { m_enabled = e; }
		bool enabled() const { return m_enabled; }

		/// Called from SkyrimPhysicsWorld::doUpdate2ndStep (worker thread, under the sim lock).
		/// Ensures a wig exists for every qualifying actor in `actors` (creating/binding new ones and
		/// dropping wigs whose actor is gone), then steps them all. Each entry carries its skeleton
		/// root plus formIDs, so config selection + head lookup are confined to one actor -- no player
		/// guessing, and no ActorManager access from the physics lock (formIDs are pre-stamped).
		void step(btScalar totalDt, btScalar tick, const std::vector<StrandActor>& actors);

		/// Request that all wigs be dropped before the next step (e.g. on game load).
		void reset() { m_resetRequested = true; }

		// --- render-thread snapshot access, all guarded by m_publishLock ---
		std::uint32_t instanceCount();
		std::uint32_t getDesc(std::uint32_t index, FSMPWigInstanceDesc* out);
		std::uint32_t copyPositions(std::uint32_t index, float* dst, std::uint32_t capacityFloats);

	private:
		StrandManager();

		/// Config for one actor, by priority: worn-wig file (wigs/<wigFormID>.xml) -> per-actor file
		/// (wigs/<actorFormID>.xml) -> the global wig.xml default. Per-actor/wig files inherit the
		/// global values and override only the tags they set. Results (hit and miss) are cached.
		const StrandConfig& configFor(std::uint32_t actorFormID, std::uint32_t wigFormID);

		/// Load and cache wigs/<id hex>.xml (inheriting the global config as its base). Returns the
		/// cached config, or nullptr if that file is absent. Never throws (malformed files fail closed).
		const StrandConfig* loadCachedConfig(std::uint32_t id);

		/// Load + cache the authored .tfx groom named by `cfg` (from grooms/), scaled by cfg.groomScale.
		/// Shared across every actor that uses the same file+scale. Returns null when no groom is set
		/// or the file is missing/invalid (caller then uses the procedural cap). Cached by "file|scale".
		std::shared_ptr<const StrandGroom> loadCachedGroom(const StrandConfig& cfg);

		std::atomic_bool m_enabled{ false };
		std::atomic_bool m_resetRequested{ false };
		StrandConfig m_config;  // author-editable groom/sim params, (re)loaded from wig.xml
		// Per-id config cache; nullopt marks a known-absent file so we don't re-stat it every frame.
		std::unordered_map<std::uint32_t, std::optional<StrandConfig>> m_configCache;
		// Authored-groom cache keyed by "file|scale"; a null value marks a known-bad/absent groom.
		std::unordered_map<std::string, std::shared_ptr<const StrandGroom>> m_groomCache;
		// One wig per actor, keyed by that actor's skeleton root node.
		std::unordered_map<RE::NiNode*, std::unique_ptr<StrandInstance>> m_instances;
		// Stable index order for the render-side API, rebuilt each step under m_publishLock.
		std::vector<StrandInstance*> m_order;
		std::mutex m_publishLock;  // protects m_instances/m_order shape + each instance's snapshot
	};
}
