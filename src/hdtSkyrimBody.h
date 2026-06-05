#pragma once

#include "hdtConvertNi.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkyrimBone.h"

namespace hdt
{
	class SkyrimSystem;

	class SkyrimBody : 
		public SkinnedMeshBody
	{
	public:
		SkyrimBody() = default;
		~SkyrimBody() = default;

		enum class SharedType
		{
			kPublic,
			kInternal,
			kExternal,
			kPrivate,
		};

		bool canCollideWith(const SkinnedMeshBody* body) const override;
		void internalUpdate() override;

		SkyrimSystem*     m_mesh = nullptr;
		SharedType        m_shared = SharedType::kPublic;
		bool              m_disabled = false;
		int               m_disablePriority = 0;
		RE::BSFixedString m_disableTag;
	};
}
