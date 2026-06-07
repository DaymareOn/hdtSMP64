#include "hdtSkinnedMeshSystem.h"

#include "hdtBoneScaleConstraint.h"
#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshShape.h"

namespace hdt
{
	void SkinnedMeshSystem::readTransform(float timeStep)
	{
		if (this->block_resetting)
			return;

		for (const auto& bone : m_bones) {
			bone->readTransform(timeStep);
		}

		for (auto i : m_constraints) {
			i->scaleConstraint();
		}

		for (auto i : m_constraintGroups) {
			i->scaleConstraint();
		}
	}

	void SkinnedMeshSystem::writeTransform(float alpha)
	{
		for (int i = 0; i < m_bones.size(); ++i) {
			if (m_bones[i]->m_rig.isKinematicObject()) {
				continue;
			}

			m_bones[i]->writeTransform(alpha);
		}
	}

	void SkinnedMeshSystem::snapshotInterpolation()
	{
		// Only the dynamic bones are simulated (and written back), so only they need a snapshot;
		// kinematic bones are skipped here exactly as in writeTransform.
		for (int i = 0; i < m_bones.size(); ++i) {
			if (m_bones[i]->m_rig.isKinematicObject()) {
				continue;
			}

			m_bones[i]->snapshotInterpolation();
		}
	}

	void SkinnedMeshSystem::internalUpdate()
	{
		for (auto& i : m_bones)
			i->internalUpdate();

		for (auto& i : m_meshes)
			i->updateBoundingSphereAabb();
	}

	void SkinnedMeshSystem::gather(std::vector<SkinnedMeshBody*>& bodies, std::vector<SkinnedMeshShape*>& shapes)
	{
		for (auto& i : m_meshes) {
			bodies.push_back(i.get());
			shapes.push_back(i->m_shape.get());
			auto triShape = dynamic_cast<PerTriangleShape*>(i->m_shape.get());

			if (triShape) {
				shapes.push_back(triShape->m_verticesCollision.get());
			}
		}
	}
}
