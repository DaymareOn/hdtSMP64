#pragma once

#include <ppltasks.h>
#include "hdtBulletHelper.h"
#include "hdtConstraintGroup.h"

namespace hdt
{
	struct SkinnedMeshBone;
	class SkinnedMeshBody;
	class SkinnedMeshShape;
	class SkinnedMeshWorld;
	class BoneScaleConstraint;

	class SkinnedMeshSystem : public RefObject
	{
		friend class hdt::SkinnedMeshWorld;
	public:

		virtual void resetTransformsToOriginal();
		virtual void readTransform(float timeStep);
		virtual void writeTransform();

		void internalUpdate();
		//void internalUpdateCL();
		void gather(std::vector<SkinnedMeshBody*>& bodies, std::vector<SkinnedMeshShape*>& shapes);

		bool valid() const { return !m_bones.empty(); }

		std::vector<std::shared_ptr<btCollisionShape>> m_shapeRefs;
		SkinnedMeshWorld* m_world = nullptr;

		std::vector<Ref<SkinnedMeshBone>>& getBones() { return m_bones; };

	protected:
		std::vector<Ref<SkinnedMeshBone>> m_bones;
		std::vector<Ref<SkinnedMeshBody>> m_meshes;
		std::vector<Ref<BoneScaleConstraint>> m_constraints;
		std::vector<Ref<ConstraintGroup>> m_constraintGroups;

	private:
		typedef concurrency::task<void> task;
		typedef concurrency::task_group task_group;
	};
}
