#pragma once
#include "DynamicHDT.h"
#include <skse64/GameReferences.h>
#include <skse64/PapyrusVM.h>
#include <skse64/PapyrusNativeFunctions.h>

namespace hdt {
	namespace papyrus {

		bool RegisterAllFunctions(SKSEPapyrusInterface* a_papy_intfc);

		bool ReloadPhysicsFile(StaticFunctionTag* base, Actor* on_actor, TESObjectARMA* on_item, BSFixedString physics_file_path, bool persist, bool verbose_log);

		bool SwapPhysicsFile(StaticFunctionTag* base, Actor* on_actor, BSFixedString old_physics_file_path, BSFixedString new_physics_file_path, bool persist, bool verbose_log);

		BSFixedString QueryCurrentPhysicsFile(StaticFunctionTag* base, Actor* on_actor, TESObjectARMA* on_item, bool verbose_log);

		namespace impl {
			bool ReloadPhysicsFileImpl(UInt32 on_actor_formID, UInt32 on_item_formID, std::string physics_file_path, bool persist, bool verbose_log);

			bool SwapPhysicsFileImpl(UInt32 on_actor_formID, std::string old_physics_file_path, std::string new_physics_file_path, bool persist, bool verbose_log);

			std::string QueryCurrentPhysicsFileImpl(UInt32 on_actor_formID, UInt32 on_item_formID, bool verbose_log);
		}


		//UInt32 FindOrCreateAnonymousSystem(StaticFunctionTag* base, TESObjectARMA* system_model, bool verbose_log);

		//UInt32 AttachAnonymousSystem(StaticFunctionTag* base, Actor* on_actor, UInt32 system_handle, bool verbose_log);

		//UInt32 DetachAnonymousSystem(StaticFunctionTag* base, Actor* on_actor, UInt32 system_handle, bool verbose_log);
	}
}
