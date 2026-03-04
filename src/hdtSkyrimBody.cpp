#include "hdtSkyrimBody.h"
#include "hdtSkyrimSystem.h"

namespace hdt
{
	SkyrimBody::SkyrimBody()
	{
		m_mesh = nullptr;
		m_shared = SharedType::SHARED_PUBLIC;
	}


	SkyrimBody::~SkyrimBody()
	{
	}

	bool SkyrimBody::canCollideWith(const SkinnedMeshBody* rhs) const
	{
		auto body = (SkyrimBody*)rhs;
		if (m_disabled || body->m_disabled)
			return false;

		switch (m_shared)
		{
		case SharedType::SHARED_PUBLIC:
			break;
		case SharedType::SHARED_INTERNAL:
			if (m_mesh->m_skeleton != body->m_mesh->m_skeleton)
				return false;
			break;
		case SharedType::SHARED_EXTERNAL:
			if (m_mesh->m_skeleton == body->m_mesh->m_skeleton)
				return false;
			break;
		case SharedType::SHARED_PRIVATE:
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
		if (m_disabled) return;
		SkinnedMeshBody::internalUpdate();
	}
}
