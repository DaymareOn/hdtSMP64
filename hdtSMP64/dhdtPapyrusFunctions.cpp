#include "dhdtPapyrusFunctions.h"

#define PAPY_FCN(a) (#a),(PAPYRUS_CLASS_NAME),a

bool RegisterFuncs(VMClassRegistry* registry)
{
	using namespace hdt::papyrus;

	registry->RegisterFunction(
		new NativeFunction5	<StaticFunctionTag, bool, Actor*, TESObjectARMA*, BSFixedString, bool, bool>(PAPY_FCN(ReloadPhysicsFile), registry));

	registry->RegisterFunction(
		new NativeFunction5	<StaticFunctionTag, bool, Actor*, BSFixedString, BSFixedString, bool, bool>(PAPY_FCN(SwapPhysicsFile), registry));

	registry->RegisterFunction(
		new NativeFunction3	<StaticFunctionTag, BSFixedString, Actor*, TESObjectARMA*, bool>(PAPY_FCN(QueryCurrentPhysicsFile), registry));

	return true;
}

bool hdt::papyrus::RegisterAllFunctions(SKSEPapyrusInterface* a_papy_intfc)
{
	return a_papy_intfc->Register(RegisterFuncs);
}

bool hdt::papyrus::ReloadPhysicsFile(StaticFunctionTag* base, Actor* on_actor, TESObjectARMA* on_item, BSFixedString physics_file_path, bool persist, bool verbose_log)
{
	if (!(on_actor && on_item)) {
		if (verbose_log)
			Console_Print("[DynamicHDT] -- Couldn't parse parameters: on_actor(ptr: %016X), on_item(ptr: %016X).", reinterpret_cast<UInt64>(on_actor), reinterpret_cast<UInt64>(on_item));
		return false;
	}
	auto p = ActorManager::instance()->reloadPhysicsFile(on_actor->formID, on_item->formID, physics_file_path.c_str());
	if (persist)
		Override::OverrideManager::GetSingleton()->registerOverride(on_actor->formID, p.second, std::string(physics_file_path));
	return p.first;
}

bool hdt::papyrus::SwapPhysicsFile(StaticFunctionTag* base, Actor* on_actor, BSFixedString old_physics_file_path, BSFixedString new_physics_file_path, bool persist, bool verbose_log)
{
	if (!on_actor) {
		if (verbose_log)Console_Print("[DynamicHDT] -- Couldn't parse parameters: on_actor(ptr: %016X).", reinterpret_cast<UInt64>(on_actor));
		return false;
	}

	bool result = ActorManager::instance()->swapPhysicsFile(on_actor->formID, old_physics_file_path.c_str(), new_physics_file_path.c_str());
	if (persist)
		Override::OverrideManager::GetSingleton()->registerOverride(on_actor->formID, old_physics_file_path.c_str(), new_physics_file_path.c_str());
	return result;
}

BSFixedString hdt::papyrus::QueryCurrentPhysicsFile(StaticFunctionTag* base, Actor* on_actor, TESObjectARMA* on_item, bool verbose_log)
{
	if (!(on_actor && on_item)) {
		if (verbose_log)
			Console_Print("[DynamicHDT] -- Couldn't parse parameters: on_actor(ptr: %016X), on_item(ptr: %016X).", reinterpret_cast<UInt64>(on_actor), reinterpret_cast<UInt64>(on_item));
		return false;
	}

	return ActorManager::instance()->queryCurrentPhysicsFile(on_actor->formID, on_item->formID).c_str();
}
//
//UInt32 hdt::papyrus::FindOrCreateAnonymousSystem(StaticFunctionTag* base, TESObjectARMA* system_model, bool verbose_log)
//{
//	
//	return UInt32();
//}
//
//UInt32 hdt::papyrus::AttachAnonymousSystem(StaticFunctionTag* base, Actor* on_actor, UInt32 system_handle, bool verbose_log)
//{
//	if (!on_actor || !system_handle) {
//		if (verbose_log)
//			Console_Print("[DynamicHDT] -- Couldn't parse parameters: on_actor(ptr: %016X), system_handle(%08X).", reinterpret_cast<UInt64>(on_actor), system_handle);
//		return false;
//	}
//
//
//
//	return UInt32();
//}
//
//UInt32 hdt::papyrus::DetachAnonymousSystem(StaticFunctionTag* base, Actor* on_actor, UInt32 system_handle, bool verbose_log)
//{
//	if (!on_actor || !system_handle) {
//		if (verbose_log)
//			Console_Print("[DynamicHDT] -- Couldn't parse parameters: on_actor(ptr: %016X), system_handle(%08X).", reinterpret_cast<UInt64>(on_actor), system_handle);
//		return false;
//	}
//
//	return UInt32();
//}
