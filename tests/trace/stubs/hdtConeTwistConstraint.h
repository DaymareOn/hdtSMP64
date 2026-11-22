#pragma once

#include "hdtBulletHelper.h"

namespace hdt
{
	class ConeTwistConstraint : public btConeTwistConstraint
	{
	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();
		using btConeTwistConstraint::btConeTwistConstraint;
	};
}
