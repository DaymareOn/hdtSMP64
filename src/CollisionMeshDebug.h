#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace RE
{
	class TESObjectREFR;
}

namespace hdt
{
	namespace CollisionMeshDebug
	{
		enum class ApplyStatus
		{
			Applied,
			NoSelectedReference,
			SelectedReferenceIsNotActor,
			NoTrackedPhysics,
			NoCollisionMeshes
		};

		struct ApplyResult
		{
			ApplyStatus status = ApplyStatus::Applied;
			std::uint32_t actorFormID = 0;
			std::size_t systems = 0;
			std::size_t geometries = 0;
			bool enabled = false;
		};

		ApplyResult applyToSelectedActor(std::optional<bool> enabled, RE::TESObjectREFR* commandTarget);

		void disableAll();
	}
}
