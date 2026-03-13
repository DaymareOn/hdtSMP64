#pragma once

#include "DynamicHDT.h"

namespace hdt
{
	namespace papyrus
	{
		bool RegisterAllFunctions(const SKSE::PapyrusInterface* a_papy_intfc);

		bool ReloadPhysicsFile(RE::StaticFunctionTag* base, RE::Actor* on_actor, RE::TESObjectARMA* on_item, RE::BSFixedString physics_file_path, bool persist, bool verbose_log);

		bool SwapPhysicsFile(RE::StaticFunctionTag* base, RE::Actor* on_actor, RE::BSFixedString old_physics_file_path, RE::BSFixedString new_physics_file_path, bool persist, bool verbose_log);

		RE::BSFixedString QueryCurrentPhysicsFile(RE::StaticFunctionTag* base, RE::Actor* on_actor, RE::TESObjectARMA* on_item, bool verbose_log);

		namespace impl
		{
			bool ReloadPhysicsFileImpl(uint32_t on_actor_formID, uint32_t on_item_formID, std::string physics_file_path, bool persist, bool verbose_log);

			bool SwapPhysicsFileImpl(uint32_t on_actor_formID, std::string old_physics_file_path, std::string new_physics_file_path, bool persist, bool verbose_log);

			std::string QueryCurrentPhysicsFileImpl(uint32_t on_actor_formID, uint32_t on_item_formID, bool verbose_log);
		}

		// uint32_t FindOrCreateAnonymousSystem(RE::StaticFunctionTag* base, RE::TESObjectARMA* system_model, bool verbose_log);

		// uint32_t AttachAnonymousSystem(RE::StaticFunctionTag* base, RE::Actor* on_actor, uint32_t system_handle, bool verbose_log);

		// uint32_t DetachAnonymousSystem(RE::StaticFunctionTag* base, RE::Actor* on_actor, uint32_t system_handle, bool verbose_log);
	}
}
