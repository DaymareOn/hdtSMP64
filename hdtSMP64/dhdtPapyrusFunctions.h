#pragma once
#include "DynamicHDT.h"
#ifndef SKYRIMVR
#include <skse64/skse64/GameReferences.h>
#include <skse64/skse64/PapyrusVM.h>
#include <skse64/skse64/PapyrusNativeFunctions.h>
#else
#include <sksevr/skse64/GameReferences.h>
#include <sksevr/skse64/PapyrusVM.h>
#include <sksevr/skse64/PapyrusNativeFunctions.h>
#endif

namespace hdt {
	namespace papyrus {

		bool RegisterAllFunctions(SKSEPapyrusInterface* a_papy_intfc);

		bool ReloadPhysicsFile(StaticFunctionTag* base, Actor* on_actor, TESObjectARMA* on_item, BSFixedString physics_file_path, bool persist, bool verbose_log);

		bool SwapPhysicsFile(StaticFunctionTag* base, Actor* on_actor, BSFixedString old_physics_file_path, BSFixedString new_physics_file_path, bool persist, bool verbose_log);

		BSFixedString QueryCurrentPhysicsFile(StaticFunctionTag* base, Actor* on_actor, TESObjectARMA* on_item, bool verbose_log);

		//UInt32 FindOrCreateAnonymousSystem(StaticFunctionTag* base, TESObjectARMA* system_model, bool verbose_log);

		//UInt32 AttachAnonymousSystem(StaticFunctionTag* base, Actor* on_actor, UInt32 system_handle, bool verbose_log);

		//UInt32 DetachAnonymousSystem(StaticFunctionTag* base, Actor* on_actor, UInt32 system_handle, bool verbose_log);
	}
}
