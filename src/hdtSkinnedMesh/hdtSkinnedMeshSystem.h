#pragma once

#include "hdtBulletHelper.h"
#include "hdtConstraintGroup.h"
#include <ppltasks.h>

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
		virtual void resetTransformsToOriginal();
		virtual void readTransform(float timeStep);
		virtual void writeTransform();

		void internalUpdate();

		void gather(std::vector<SkinnedMeshBody*>& bodies, std::vector<SkinnedMeshShape*>& shapes);

		bool valid() const { return !m_bones.empty(); }

		std::vector<std::shared_ptr<btCollisionShape>> m_shapeRefs;
		SkinnedMeshWorld* m_world = nullptr;

		bool block_resetting = false;
		std::vector<RE::BSTSmartPointer<SkinnedMeshBone>>& getBones() { return m_bones; };

	protected:
		std::vector<RE::BSTSmartPointer<SkinnedMeshBone>> m_bones;
		std::vector<RE::BSTSmartPointer<SkinnedMeshBody>> m_meshes;
		std::vector<RE::BSTSmartPointer<BoneScaleConstraint>> m_constraints;
		std::vector<RE::BSTSmartPointer<ConstraintGroup>> m_constraintGroups;

	private:
		typedef concurrency::task<void> task;
		typedef concurrency::task_group task_group;
	};
}
