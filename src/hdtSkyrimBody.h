#pragma once

#include "hdtConvertNi.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkyrimBone.h"

namespace hdt
{
	class SkyrimSystem;

	class SkyrimBody : public SkinnedMeshBody
	{
	public:
		SkyrimBody();
		~SkyrimBody();

		enum class SharedType
		{
			SHARED_PUBLIC,
			SHARED_INTERNAL,
			SHARED_EXTERNAL,
			SHARED_PRIVATE,
		};

		SkyrimSystem* m_mesh;
		SharedType m_shared;
		bool m_disabled = false;
		int m_disablePriority = 0;
		RE::BSFixedString m_disableTag;

		bool canCollideWith(const SkinnedMeshBody* body) const override;
		void internalUpdate() override;
	};
}
