#pragma once

#include "hdtSkinnedMeshSystem.h"

namespace hdt
{
	class GroupConstraintSolver
	{
	public:
		static btSingleConstraintRowSolver getResolveSingleConstraintRowGenericAVX();
		static btSingleConstraintRowSolver getResolveSingleConstraintRowLowerLimitAVX();

		std::vector<ConstraintGroup*> m_groups;
	};
}
