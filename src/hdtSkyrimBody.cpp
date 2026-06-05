#include "hdtSkyrimBody.h"
#include "hdtSkyrimSystem.h"

namespace hdt
{
	bool SkyrimBody::canCollideWith(const SkinnedMeshBody* rhs) const
	{
		auto body = (SkyrimBody*)rhs;
		if (m_disabled || body->m_disabled)
			return false;

		switch (m_shared) 
		{
		case SharedType::kPublic:
			break;
		case SharedType::kInternal:
			if (m_mesh->m_skeleton != body->m_mesh->m_skeleton)
				return false;
			break;
		case SharedType::kExternal:
			if (m_mesh->m_skeleton == body->m_mesh->m_skeleton)
				return false;
			break;
		case SharedType::kPrivate:
			if (m_mesh != body->m_mesh)
				return false;
			break;
		default:
			return false;
		}

		return SkinnedMeshBody::canCollideWith(rhs);
	}

	void SkyrimBody::internalUpdate()
	{
		if (m_disabled)
			return;
		SkinnedMeshBody::internalUpdate();
	}
}
