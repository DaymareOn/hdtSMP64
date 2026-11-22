#pragma once

#include "hdtBulletHelper.h"

namespace hdt
{
	_CRT_ALIGN(16)
	struct SkinnedMeshBone
	{
		BT_DECLARE_ALIGNED_ALLOCATOR();

		SkinnedMeshBone(const char* a_name, btRigidBody::btRigidBodyConstructionInfo& a_info) :
			m_name(a_name),
			m_rig(a_info)
		{}

		RE::BSFixedString m_name;
		float m_gravityFactor = 1.0f;
		btRigidBody m_rig;
	};
}
