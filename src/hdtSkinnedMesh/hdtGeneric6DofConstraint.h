#pragma once

#include "hdtBulletHelper.h"
#include "hdtBoneScaleConstraint.h"

namespace hdt
{
	class Generic6DofConstraint : 
		public BoneScaleConstraint, 
		public btGeneric6DofSpring2Constraint
	{
	public:
		using btGeneric6DofSpring2Constraint::operator new;
		using btGeneric6DofSpring2Constraint::operator delete;
		using btGeneric6DofSpring2Constraint::operator new[];
		using btGeneric6DofSpring2Constraint::operator delete[];
	public:
		Generic6DofConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b, const btTransform& frameInA, const btTransform& frameInB);
		virtual ~Generic6DofConstraint() override = default;
		
		// override ()
		void scaleConstraint() override;
	private:
	};
}
