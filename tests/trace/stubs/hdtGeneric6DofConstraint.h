#pragma once

#include "hdtBulletHelper.h"

namespace hdt
{
	class Generic6DofConstraint : public btGeneric6DofSpring2Constraint
	{
	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();
		using btGeneric6DofSpring2Constraint::btGeneric6DofSpring2Constraint;
	};
}
