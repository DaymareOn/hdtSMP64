#pragma once

#include <atomic>

namespace hdt
{
	// When true, the scenegraph audit hook is active and will log calls that
	// detach FSMP-managed nodes (name prefix hdtSSEPhysics_AutoRename_Armor_ or
	// hdtSSEPhysics_AutoRename_Head_) from any caller, together with the
	// caller's module name and offset so the responsible DLL can be identified.
	//
	// Enabled via <auditScenegraph>true</auditScenegraph> inside <smp> in configs.xml.
	// When false the hook is not installed and there is zero runtime overhead.
	extern std::atomic<bool> g_auditScenegraph;
}

namespace Hooks
{
	class NodeAuditHooks
	{
	public:
		// Install the DetachChild hook if hdt::g_auditScenegraph is true.
		// Must be called inside a Detours transaction (DetourTransactionBegin /
		// DetourTransactionCommit) or open its own transaction as needed.
		static void Install();

	private:
		// Prefixes that identify scenegraph nodes managed by FSMP.
		static constexpr std::string_view kArmorPrefix{ "hdtSSEPhysics_AutoRename_Armor_" };
		static constexpr std::string_view kHeadPrefix{ "hdtSSEPhysics_AutoRename_Head_" };

		// NiNode::DetachChild(NiAVObject* child, NiPointer<NiAVObject>& childOut)
		//
		// Vtable slot (0-based index in the NiNode vtable array):
		//   SE / AE : 0x2C (44)
		//   VR      : 0x2D (45)
		//
		// Derivation:
		//   NetImmerseUtils.h documents GetObjectByName at vtable[0x2A] (SE/AE)
		//   and vtable[0x2B] (VR). NiNode adds virtual functions immediately
		//   after the last NiAVObject virtual in this order:
		//     AttachChild(NiAVObject*, bool)          -> [0x2B] SE / [0x2C] VR
		//     DetachChild(NiAVObject*, NiPointer<>&)  -> [0x2C] SE / [0x2D] VR  <-- hooked here
		//     DetachChild2(NiAVObject*)               -> [0x2D] SE / [0x2E] VR
		//     DetachChildAt(uint32, NiPointer<>&)     -> [0x2E] SE / [0x2F] VR
		//     DetachChildAt2(uint32)                  -> [0x2F] SE / [0x30] VR
		//
		// If this is ever wrong, adjust kSlot_SE_AE / kSlot_VR below and
		// verify by checking the disassembly of NiNode::DetachChild.
		static constexpr std::size_t kSlot_SE_AE = 0x2C;
		static constexpr std::size_t kSlot_VR    = 0x2D;

		using DetachChild_t = void(RE::NiNode*, RE::NiAVObject*, RE::NiPointer<RE::NiAVObject>&);

		__declspec(noinline) static void DetachChild_Hook(
			RE::NiNode*                    a_this,
			RE::NiAVObject*                a_child,
			RE::NiPointer<RE::NiAVObject>& a_childOut);

		static inline DetachChild_t* _DetachChild_Orig = nullptr;
	};
}
