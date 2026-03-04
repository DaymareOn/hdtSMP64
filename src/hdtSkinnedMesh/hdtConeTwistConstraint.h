#pragma once
#include "hdtBoneScaleConstraint.h"

namespace hdt
{
	class ConeTwistConstraint : 
		public BoneScaleConstraint, 
		public btConeTwistConstraint
	{
	public:
		using btConeTwistConstraint::operator new;
		using btConeTwistConstraint::operator delete;
		using btConeTwistConstraint::operator new[];
		using btConeTwistConstraint::operator delete[];
	public:
		ConeTwistConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b, const btTransform& frameInA, const btTransform& frameInB);
		virtual ~ConeTwistConstraint() override = default;
		
		void scaleConstraint() override;
	};
}
