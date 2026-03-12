#pragma once

#include <detours/detours.h>

namespace Hooks
{
	class BSFaceGenNiNodeHooks
	{
	public:
		static void ApplyBoneLimitFix();

		static void Hook()
		{
			REL::Relocation<uintptr_t> SkinSingleGeometryCode1 { REL::VariantID(26466, 27061, 0x03EBB30), REL::VariantOffset(0x108, 0x10F, 0x108) };	// 0x03dc1c0, 0x03F6770, 0x03EBB30 (SE/1.5.97.0, AE/1.6.640.0, VR/1.4.15.0)

			//
			auto& trampoline = SKSE::GetTrampoline();
			SKSE::AllocTrampoline(14);

			//
			logger::debug("Applying BSFaceGenNiNodeHooks hooks!");

			//
			REL::Relocation<uintptr_t> BSFaceGenNiNode__vtbl { RE::VTABLE_BSFaceGenNiNode[0] };
			BSFaceGenNiNode__vtbl.write_vfunc(0x3E, SkinAllGeometry__Hook);

			//
			_SkinSingleGeometry = trampoline.write_call<5>(SkinSingleGeometryCode1.address(), SkinSingleGeometry__Hook);
			REL::safe_write(_SkinSingleGeometry.address() + REL::Offset(0x96).offset(), static_cast<uint8_t>(0x7));

			//
			ApplyBoneLimitFix();

			//
			logger::debug("...success");
		}

	public:
		static void ProcessHeadPart(RE::BSFaceGenNiNode* const, RE::BGSHeadPart*, RE::NiNode*, bool);
		static void SkinAllGeometryCalls(RE::BSFaceGenNiNode* const, RE::NiNode*, bool);
		static void SkinSingleGeometry__Hook(RE::BSFaceGenNiNode* const, RE::NiNode*, RE::BSGeometry*, bool);
		static void SkinAllGeometry__Hook(RE::BSFaceGenNiNode* const, RE::NiNode*, bool a_unk);
		static void SkinAllGeometry(RE::BSFaceGenNiNode* const, RE::NiNode*, bool a_unk);

		// Detour on SetBoneName entry so any caller (known or unknown) cannot write beyond FMD.bones[7].
		// Complements the loop-count cap in ApplyBoneLimitFix.
		static void HookSetBoneName();
		static void SetBoneName_Hook(RE::BSFaceGenModelExtraData*, std::uint32_t, RE::BSFixedString*);

		static inline REL::Relocation<decltype(&BSFaceGenNiNodeHooks::SkinSingleGeometry__Hook)> _SkinSingleGeometry;
	protected:
		using SetBoneName_t = decltype(SetBoneName_Hook);
	private:
		// BSFaceGenModelExtraData::SetBoneName
		static inline SetBoneName_t* _SetBoneName { nullptr };
	};

	class MainHooks
	{
	public:
		static void Hook()
		{
			REL::Relocation<uintptr_t> UpdateHook1 { REL::VariantID(35551, 36544, 0x05B6D70), REL::VariantOffset(0x11F, 0x160, 0x11F) };  // 0x05AF3D0, 0x05E7EE0, 0x05B6D70 (SE/1.5.97.0, AE/1.6.640.0, VR/1.4.15.0)
			REL::Relocation<uintptr_t> UpdateHook2 { REL::VariantID(35565, 36564, 0x05BAB10), REL::VariantOffset(0x56D, 0x9DC, 0x611) };  // 0x05B2FF0, 0x05EC240, 0x05BAB10 (SE/1.5.97.0, AE/1.6.640.0, VR/1.4.15.0)

			logger::debug("Applying MainHooks hooks!");

			//
			auto& trampoline = SKSE::GetTrampoline();

			//
			SKSE::AllocTrampoline(14);
			_Update = trampoline.write_call<5>(UpdateHook1.address(), Update);

			//
			SKSE::AllocTrampoline(14);
			_Unk_sub = trampoline.write_call<5>(UpdateHook2.address(), Unk_sub);

			//
			logger::debug("...success");
		}

		static void Update(RE::Main* const);
		static void Unk_sub(void*);  // RE::BSBethesdaPlatform*
	public:
		static inline REL::Relocation<decltype(&Unk_sub)> _Unk_sub;
		static inline REL::Relocation<decltype(&Update)> _Update;
	};

	class ActorEquipManagerHooks
	{
	public:
		static void Hook()
		{
			//
			logger::debug("Applying ActorEquipManagerHooks hooks!");

			//
			DetourAttach((PVOID*)(&_func), (PVOID)func);

			//
			logger::debug("...success");
		}

		static bool func(RE::ActorEquipManager* const, RE::Actor*, RE::TESBoundObject*, RE::ExtraDataList*, std::uint32_t, const RE::BGSEquipSlot*, bool, bool, bool, bool, const RE::BGSEquipSlot*);
	protected:
		using func_t = decltype(func);
	private:
		static inline func_t* _func { (func_t*)REL::VariantID(37945, 38901, 0x06411A0).address() };  // 0x0638190, 0x0670210, 0x06411A0 (SE/1.5.97.0, AE/1.6.640.0, VR/1.4.15.0)
	};

	class BipedAnimHooks
	{
	public:
		static inline std::vector<std::string> BackupNodes{};
	public:
		static void Hook()
		{
			logger::debug("Applying BipedAnimHooks hooks!");

			//
			DetourAttach((PVOID*)(&_func), (PVOID)func);

			//
			logger::debug("...success");
		}

		static RE::NiAVObject* func(RE::BipedAnim* const, RE::NiNode*, RE::BSFadeNode*, uint32_t, void*, void*, void*);
	protected:
		using func_t = decltype(func);
	private:
		static inline func_t* _func { (func_t*)REL::VariantID(15535, 15712, 0x01DB9E0).address() };  // 0x01CAFB0, 0x01D83B0, 0x01DB9E0 (SE/1.5.97.0, AE/1.6.640.0, VR/1.4.15.0)
	};

    void Install();
}
