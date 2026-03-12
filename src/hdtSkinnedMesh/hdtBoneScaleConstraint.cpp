#include "hdtBoneScaleConstraint.h"

namespace hdt
{
	BoneScaleConstraint::BoneScaleConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b, btTypedConstraint* constraint)
	{
		m_boneA = a;
		m_boneB = b;
		m_scaleA = m_scaleB = 1;
		m_constraint = constraint;
	}

	BoneScaleConstraint::~BoneScaleConstraint()
	{
	}
}
