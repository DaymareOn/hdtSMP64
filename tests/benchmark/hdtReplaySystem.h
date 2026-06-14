#pragma once

// hdtReplaySystem.h - the replay-side concrete types for smp_replay (D2, §10).
//
// ReplayBone is a concrete SkinnedMeshBone (the base readTransform/writeTransform are pure virtual)
// that, instead of reading a NiNode like SkyrimBone, applies the per-frame target captured in the
// file - through the SAME core SkinnedMeshBone::applyKinematicTarget the game uses, which is the
// fidelity guarantee (D6). ReplaySystem is a SkinnedMeshSystem rebuilt from a replay::Snapshot.
//
// This file is compiled only into the smp_replay exe, against the RE shim (ShimPCH.h) + Bullet.

#include "Replay/hdtReplayConvert.h"
#include "Replay/hdtReplayFormat.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshWorld.h"

#include <memory>

namespace hdt
{
	// Bullet <-> POD converters live in Replay/hdtReplayConvert.h (shared with the capture side).

	class ReplayBone : public SkinnedMeshBone
	{
	public:
		ReplayBone(const RE::BSFixedString& name, btRigidBody::btRigidBodyConstructionInfo& ci);

		// Driver bones (kinematic, with captured targets) apply their pending target each frame via
		// applyKinematicTarget; dynamic bones evolve purely under the solver, so their readTransform
		// is a no-op (they have no captured target).
		void readTransform(float timeStep) override;
		void writeTransform() override;

		// Set/cleared by the harness before each step from the frame's kinematicTargets.
		void setFrameTarget(const btQsTransform& target, bool reset);
		void clearFrameTarget() { m_hasTarget = false; }

		bool m_isDriver = false;  // kinematic bone that receives captured targets

	private:
		bool m_hasTarget = false;
		btQsTransform m_target;
		bool m_reset = false;
	};

	class ReplaySystem : public SkinnedMeshSystem
	{
	public:
		/// Rebuilds a live system from a captured Snapshot - the inverse of replay::buildSnapshot.
		/// Order is deliberate: collision shapes first (two passes, so a compound can point at children
		/// that already exist), then bones (so bodies and constraints can resolve their bone indices),
		/// then the mesh-body colliders, then the constraints. Returns a ready-to-add SkinnedMeshSystem.
		static RE::BSTSmartPointer<ReplaySystem> build(const replay::Snapshot& snapshot);

		uint32_t systemId() const { return m_systemId; }

		// Bone access by capture index (the per-frame stream addresses bones by index).
		ReplayBone* bone(uint32_t index)
		{
			return index < m_bones.size() ? static_cast<ReplayBone*>(m_bones[index].get()) : nullptr;
		}

		std::size_t boneCount() const { return m_bones.size(); }
		std::size_t bodyCount() const { return m_meshes.size(); }

		// Drives every bone that has a pending captured target for this frame.
		void readTransform(float timeStep) override;

	private:
		uint32_t m_systemId = 0;
	};

	// Thin standalone world for the harness. SkinnedMeshWorld's per-frame entry points (readTransform,
	// writeTransform) and the translation-offset recentring (which the game does in SkyrimPhysicsWorld)
	// are exposed/reimplemented here so the replay loop runs exactly the in-game sequence (§10/§12).
	class ReplayWorld : public SkinnedMeshWorld
	{
	public:
		using SkinnedMeshWorld::readTransform;
		using SkinnedMeshWorld::writeTransform;

		std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>>& systems() { return m_systems; }

		/// Before stepping, shifts the whole world so the *average* body position lands on the origin,
		/// and returns that average. Floating-point is most precise near zero, so keeping coordinates
		/// small makes the solver more accurate; restoreTranslationOffset adds the average back so
		/// nothing has really moved. Mirrors SkyrimPhysicsWorld::apply/restoreTranslationOffset.
		btVector3 applyTranslationOffset();
		/// Undoes applyTranslationOffset by adding `offset` back to every body (positions world-absolute again).
		void restoreTranslationOffset(const btVector3& offset);

		void setWind(const btVector3& wind) { m_windSpeed = wind; }

		// Reproduces the captured world/solver configuration (mirrors the SkyrimPhysicsWorld ctor +
		// hdtSkyrimPhysicsWorld.cpp:12-51) so replay solves with identical settings (§12).
		void applySolver(const replay::SolverConfig& s)
		{
			gDisableDeactivation = s.disableDeactivation != 0;
			setGravity(replay::toBt(s.gravity));
			auto& si = getSolverInfo();
			si.m_friction = s.friction;
			si.m_splitImpulse = s.splitImpulse != 0;
			si.m_splitImpulsePenetrationThreshold = s.splitImpulsePenetrationThreshold;
			si.m_erp2 = s.erp2;
			si.m_globalCfm = s.globalCfm;
			si.m_restitutionVelocityThreshold = s.restitutionVelocityThreshold;
			si.m_solverMode = s.solverMode;
			si.m_leastSquaresResidualThreshold = s.leastSquaresResidualThreshold;
			m_enableWind = s.enableWind != 0;
			m_windTime = 0.0f;
		}
	};
}
