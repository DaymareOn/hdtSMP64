#include "NodeHookAudit.h"

#include <detours/detours.h>

namespace hdt
{
	std::atomic<bool> g_auditScenegraph{ false };
}

namespace
{
	// Returns a human-readable "ModuleName.dll+0xOFFSET" string for the given
	// return address.  Only called when auditing is enabled, so cost does not
	// matter on the fast path.
	std::string getCallerInfo(void* a_retAddr)
	{
		HMODULE hMod = nullptr;
		const BOOL ok = ::GetModuleHandleExA(
			GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
				GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
			reinterpret_cast<LPCSTR>(a_retAddr),
			&hMod);

		if (!ok || !hMod) {
			return fmt::format("unknown({:p})", a_retAddr);
		}

		char path[MAX_PATH];
		const DWORD len = ::GetModuleFileNameA(hMod, path, MAX_PATH);
		if (len == 0) {
			const uintptr_t off =
				reinterpret_cast<uintptr_t>(a_retAddr) -
				reinterpret_cast<uintptr_t>(hMod);
			return fmt::format("unknown_mod+0x{:X}", off);
		}

		// Extract the filename portion of the path.
		const char* modName = path;
		for (DWORD i = len; i > 0; --i) {
			if (path[i - 1] == '\\' || path[i - 1] == '/') {
				modName = path + i;
				break;
			}
		}

		const uintptr_t offset =
			reinterpret_cast<uintptr_t>(a_retAddr) -
			reinterpret_cast<uintptr_t>(hMod);

		return fmt::format("{}+0x{:X}", modName, offset);
	}
}

namespace Hooks
{
	// __declspec(noinline) ensures the compiler cannot inline this function,
	// which is necessary for _ReturnAddress() to return the address of the
	// CALLER of the hooked NiNode::DetachChild (i.e. the code that called
	// DetachChild, not something inside our hook).
	__declspec(noinline) void NodeAuditHooks::DetachChild_Hook(
		RE::NiNode*                    a_this,
		RE::NiAVObject*                a_child,
		RE::NiPointer<RE::NiAVObject>& a_childOut)
	{
		if (a_child && hdt::g_auditScenegraph.load(std::memory_order_relaxed)) {
			// Thread-local recursion guard: prevents re-entrant logging from
			// triggering another audit event inside this same hook.
			static thread_local bool s_inHook = false;
			if (!s_inHook) {
				// RAII guard: ensures s_inHook is reset even if logging throws.
				struct Guard
				{
					bool& flag;
					Guard(bool& f) : flag(f) { flag = true; }
					~Guard() { flag = false; }
				} guard(s_inHook);

				// BSFixedString::c_str() is safe to call on a valid NiAVObject.
				const char* cname = a_child->name.c_str();
				if (cname && cname[0]) {
					const std::string_view name{ cname };
					if (name.starts_with(kArmorPrefix) || name.starts_with(kHeadPrefix)) {
						// _ReturnAddress() is an MSVC intrinsic that reads the return
						// address sitting on the stack, i.e. the address inside the
						// function that called NiNode::DetachChild.
						void* const retAddr = _ReturnAddress();
						const std::string callerInfo = getCallerInfo(retAddr);
						logger::info(
							"[FSMP-Audit] DetachChild: node=\"{}\" ptr={:p} parent={:p} caller={}",
							name,
							static_cast<const void*>(a_child),
							static_cast<const void*>(a_this),
							callerInfo);
					}
				}
			}
		}

		_DetachChild_Orig(a_this, a_child, a_childOut);
	}

	void NodeAuditHooks::Install()
	{
		if (!hdt::g_auditScenegraph.load(std::memory_order_relaxed)) {
			logger::debug("NodeAuditHooks: auditScenegraph is disabled - hook not installed");
			return;
		}

		logger::debug("Applying NodeAuditHooks (NiNode::DetachChild)...");

		// Determine the vtable slot for the target Skyrim version.
		// SE and AE both use slot 0x2C; VR inserts ApplyLocalTransformToWorld at
		// slot 26 (shifting all later entries by +1), so VR uses slot 0x2D.
		const std::size_t slot = REL::Module::IsVR() ? kSlot_VR : kSlot_SE_AE;

		// Read the function pointer from NiNode's primary vtable.
		REL::Relocation<std::uintptr_t> NiNode__vtbl{ RE::VTABLE_NiNode[0] };
		const auto funcAddr =
			*reinterpret_cast<std::uintptr_t*>(
				NiNode__vtbl.address() + sizeof(void*) * slot);
		_DetachChild_Orig = reinterpret_cast<DetachChild_t*>(funcAddr);

		// Install the Detour inside its own transaction so that the audit hook
		// can be added independently of the other hooks in Hooks::Install().
		DetourTransactionBegin();
		DetourUpdateThread(GetCurrentThread());
		DetourAttach(
			reinterpret_cast<PVOID*>(&_DetachChild_Orig),
			reinterpret_cast<PVOID>(DetachChild_Hook));
		const LONG err = DetourTransactionCommit();

		if (err != NO_ERROR) {
			logger::error("NodeAuditHooks: DetourTransactionCommit failed ({})", err);
		} else {
			logger::debug(
				"NodeAuditHooks: NiNode::DetachChild hooked at vtable slot 0x{:X}",
				slot);
		}
	}
}
