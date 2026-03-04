#pragma once

#include "hdtConvertNi.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"

namespace hdt
{
	class SkyrimBone : public SkinnedMeshBone
	{
	public:

		SkyrimBone(IDStr name, RE::NiNode* node, RE::NiNode* skeleton, btRigidBody::btRigidBodyConstructionInfo& ci);

		void resetTransformToOriginal() override;
		void readTransform(float timeStep) override;
		void writeTransform() override;

		int m_depth;
		RE::NiNode* m_node;
		RE::NiNode* m_skeleton;

	private:
		int m_forceUpdateType;
	};
}
