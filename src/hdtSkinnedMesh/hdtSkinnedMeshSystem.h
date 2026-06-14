#pragma once

#include "hdtBulletHelper.h"
#include "hdtConstraintGroup.h"
#include <tbb/tbb.h>

namespace hdt
{
	struct SkinnedMeshBone;
	class SkinnedMeshBody;
	class SkinnedMeshShape;
	class SkinnedMeshWorld;
	class BoneScaleConstraint;

	class SkinnedMeshSystem :
		public RE::BSIntrusiveRefCounted
	{
		friend class hdt::SkinnedMeshWorld;

	public:
		virtual ~SkinnedMeshSystem() = default;

		virtual float prepareForRead(float timeStep) { return timeStep; }
		virtual void readTransform(float timeStep);
		virtual void writeTransform();

		void internalUpdate();

		void gather(std::vector<SkinnedMeshBody*>& bodies, std::vector<SkinnedMeshShape*>& shapes);

		bool valid() const { return !m_bones.empty(); }

		// Read-only views used by the replay capture snapshot builder (replay::buildSnapshot).
		const std::vector<RE::BSTSmartPointer<SkinnedMeshBone>>& bonesView() const { return m_bones; }
		const std::vector<RE::BSTSmartPointer<SkinnedMeshBody>>& meshesView() const { return m_meshes; }
		const std::vector<RE::BSTSmartPointer<BoneScaleConstraint>>& constraintsView() const { return m_constraints; }

		std::vector<std::shared_ptr<btCollisionShape>> m_shapeRefs;
		SkinnedMeshWorld* m_world = nullptr;

		// Per-system wind factor (full actor/skeleton, derived from obstructions). Hoisted from
		// SkyrimSystem to the core base so the world's applyWind needs no Skyrim-layer cast and the
		// replay harness can drive it; SkyrimSystem inherits it unchanged.
		float m_windFactor = 1.0f;

		bool block_resetting = false;
		std::vector<RE::BSTSmartPointer<SkinnedMeshBone>>& getBones() { return m_bones; };

	protected:
		std::vector<RE::BSTSmartPointer<SkinnedMeshBone>> m_bones;
		std::vector<RE::BSTSmartPointer<SkinnedMeshBody>> m_meshes;
		std::vector<RE::BSTSmartPointer<BoneScaleConstraint>> m_constraints;
		std::vector<RE::BSTSmartPointer<ConstraintGroup>> m_constraintGroups;

	private:
		typedef tbb::task_group task_group;
	};
}
