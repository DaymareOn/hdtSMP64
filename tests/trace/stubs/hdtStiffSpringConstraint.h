#pragma once

#include "hdtBulletHelper.h"

namespace hdt
{
	class StiffSpringConstraint : public btPoint2PointConstraint
	{
	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();
		using btPoint2PointConstraint::btPoint2PointConstraint;

		float m_minDistance = 0;
		float m_maxDistance = 0;
		float m_stiffness = 0;
		float m_damping = 0;
		float m_equilibriumPoint = 0;
	};
}
