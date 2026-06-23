#include "ActorManager.h"

#include "Events.h"
#include "Hooks.h"

#include <cstdint>
#include <cstring>

namespace Hooks
{
	void BSFaceGenNiNodeHooks::ProcessHeadPart(RE::BSFaceGenNiNode* const a_this, RE::BGSHeadPart* headPart, RE::NiNode* a_skeleton, bool a_unk)
	{
		//
		if (headPart) {
			RE::NiAVObject* headNode = a_this->GetObjectByName(headPart->formEditorID);
			if (headNode) {
				RE::BSGeometry* headGeometry = headNode->AsGeometry();
				if (headGeometry) {
					SkinSingleGeometry__Hook(a_this, a_skeleton, headGeometry, a_unk);
				}
			}

			//
			for (auto it : headPart->extraParts) {
				ProcessHeadPart(a_this, it, a_skeleton, a_unk);
			}
		}
	}

	void BSFaceGenNiNodeHooks::SkinAllGeometryCalls(RE::BSFaceGenNiNode* const a_this, RE::NiNode* a_skeleton, bool a_unk)
	{
		bool needRegularCall = true;
		// userData (engine-supplied, can be null) must exist before we look up head parts. Check it first
		// because it's a cheap pointer read, while skeletonNeedsParts() walks the skeleton tree.
		auto* userData = a_skeleton->GetUserData();
		if (userData && hdt::ActorManager::instance()->skeletonNeedsParts(a_skeleton)) {
			RE::TESForm* form = RE::TESForm::LookupByID(userData->formID);
			RE::Actor* actor = skyrim_cast<RE::Actor*>(form);
			if (actor) {
				RE::TESNPC* actorBase = skyrim_cast<RE::TESNPC*>(actor->data.objectReference);
				uint32_t numHeadParts = 0;
				RE::BGSHeadPart** Headparts = nullptr;

				if (actorBase->HasOverlays()) {
					numHeadParts = actorBase->GetNumBaseOverlays();
					Headparts = actorBase->GetBaseOverlays();
				} else {
					numHeadParts = actorBase->numHeadParts;
					Headparts = actorBase->headParts;
				}

				if (Headparts) {
					for (uint32_t i = 0; i < numHeadParts; i++) {
						if (Headparts[i]) {
							ProcessHeadPart(a_this, Headparts[i], a_skeleton, a_unk);
						}
					}
				}

				if (userData->formID == 0x14) {
					needRegularCall = false;
				}
			}
		}

		if (needRegularCall) {
			SkinAllGeometry(a_this, a_skeleton, a_unk);
		}
	}

	void BSFaceGenNiNodeHooks::SkinSingleGeometry__Hook(RE::BSFaceGenNiNode* const a_this, RE::NiNode* a_skeleton, RE::BSGeometry* a_triShape, [[maybe_unused]] bool a_unk)
	{
		// a_skeleton is supplied by the engine and can be null
		if (!a_skeleton)
			return;

		//
		const char* name = "";
		uint32_t formId = 0x0;

		//
		if (a_skeleton->GetUserData() && a_skeleton->GetUserData()->GetBaseObject()) {
			auto bname = skyrim_cast<RE::TESFullName*>(a_skeleton->GetUserData()->GetBaseObject());
			if (bname) {
				name = bname->GetFullName();
			}

			auto bnpc = skyrim_cast<RE::TESNPC*>(a_skeleton->GetUserData()->GetBaseObject());
			if (bnpc && bnpc->faceNPC) {
				formId = bnpc->faceNPC->formID;
			}
		}

		//
		logger::debug("SkinSingleGeometry {} {} - {}, {}, (formid {:08x} base form {:08x} head template form {:08x})", a_skeleton->name.c_str(), a_skeleton->GetChildren().size(), a_triShape->name.c_str(), name, a_skeleton->GetUserData() ? a_skeleton->GetUserData()->formID : 0x0, a_skeleton->GetUserData() ? a_skeleton->GetUserData()->GetBaseObject()->formID : 0x0, formId);

		//
		Events::SkinSingleHeadGeometryEvent e;
		e.headNode = a_this;
		e.skeleton = a_skeleton;
		e.geometry = a_triShape;

		//
		Events::Sources::SkinSingleHeadGeometryEventSource::GetSingleton()->SendEvent(&e);
	}

	void BSFaceGenNiNodeHooks::SkinAllGeometry__Hook(RE::BSFaceGenNiNode* const a_this, RE::NiNode* a_skeleton, bool a_unk)
	{
		//
		const char* name = "";
		uint32_t formId = 0x0;

		//
		if (a_skeleton->GetUserData() && a_skeleton->GetUserData()->data.objectReference) {
			auto bname = skyrim_cast<RE::TESFullName*>(a_skeleton->GetUserData()->data.objectReference);
			if (bname) {
				name = bname->GetFullName();
			}

			auto bnpc = skyrim_cast<RE::TESNPC*>(a_skeleton->GetUserData()->data.objectReference);
			if (bnpc && bnpc->faceNPC) {
				formId = bnpc->faceNPC->formID;
			}
		}

		//
		logger::debug("SkinAllGeometry {} {}, {}, (formid {:08x} base form {:08x} head template form {:08x})", a_skeleton->name.c_str(), a_skeleton->GetChildren().size(), name, a_skeleton->GetUserData() ? a_skeleton->GetUserData()->formID : 0x0, a_skeleton->GetUserData() ? a_skeleton->GetUserData()->GetBaseObject()->formID : 0x0, formId);

		//
		Events::SkinAllHeadGeometryEvent e;
		e.skeleton = a_skeleton;
		e.headNode = a_this;

		Events::Sources::SkinAllHeadGeometryEventSource::GetSingleton()->SendEvent(&e);

		//
		if (REL::Module::IsAE()) {
			SkinAllGeometryCalls(a_this, a_skeleton, a_unk);
		} else {
			SkinAllGeometry(a_this, a_skeleton, a_unk);
		}

		//
		e.hasSkinned = true;

		//
		Events::Sources::SkinAllHeadGeometryEventSource::GetSingleton()->SendEvent(&e);
	}

	void BSFaceGenNiNodeHooks::SkinAllGeometry(RE::BSFaceGenNiNode* const a_this, RE::NiNode* a_skeleton, bool a_unk)
	{
		if (a_skeleton) {
			const auto& children = a_this->GetChildren();
			if (children.size() > 0) {
				for (std::uint16_t i = 0; i < children.size(); i++) {
					auto child = children[i];
					if (child) {
						auto triShape = child->AsTriShape();
						if (triShape) {
							SkinSingleGeometry__Hook(a_this, a_skeleton, triShape, a_unk);
						}
					}
				}
			}
		}
	}

	// Clamps a skinned shape's bone count to 8 at the engine's geometry-skinning read site.
	// The game reads skinData->boneCount there to size a fixed hardware skinning buffer, so a
	// larger count overruns it. We branch that site through a trampoline holding a hand-assembled
	// byte patch that loads the count and forces it down to 8 when it exceeds 8, then jumps back.
	// Only the count register differs by version (esi on SE/VR, ebp on AE), changing a few opcode bytes.
	void BSFaceGenNiNodeHooks::ApplyBoneLimitFix()
	{
		// VR resolves the SE id (24330) through the VR address library rather than a hardcoded
		// offset: a missing id then fails loudly at load instead of silently jumping to a bad
		// address. The per-runtime VariantOffset is the byte offset of the patched instruction.
		REL::Relocation<uintptr_t> GeometrySkinningBoneFix{ RELOCATION_ID(24330, 24836), REL::VariantOffset(0x58, 0x75, 0x58) };

		// Pre-assembled replacement for the former Xbyak::CodeGenerator (27 bytes):
		//   mov  reg, [rax+0x58]   ; read skinData->boneCount
		//   cmp  reg, 8            ; compare against the limit
		//   jle  +5                ; already <= 8: skip the clamp below
		//   mov  reg, 8            ; clamp to 8
		//   jmp  qword ptr [rip+0] ; absolute jump back to the patched site
		//   <8-byte return address>
#pragma pack(push, 1)
		struct Patch
		{
			std::uint8_t mov_load[3];   // mov reg32, [rax+0x58]
			std::uint8_t cmp_8[3];      // cmp reg32, 8
			std::uint8_t jle_5[2];      // jle +5 (skip the clamp below)
			std::uint8_t mov_clamp[5];  // mov reg32, 8
			std::uint8_t jmp_rip[6];    // jmp qword ptr [rip+0]
			std::uintptr_t ret_addr;    // absolute return address
		};
#pragma pack(pop)
		static_assert(sizeof(Patch) == 27, "Patch struct size mismatch");

		// SE and VR use esi, AE uses ebp as the bone-count register; only a few bytes differ.
		// IsAE() is false for VR, so VR takes the esi branch (same as the original xbyak code).
		const bool isAE = REL::Module::IsAE();

		Patch patch{};
		// mov reg, [rax+0x58]: opcode 0x8B, ModRM selects the destination register.
		patch.mov_load[0] = 0x8B;
		patch.mov_load[1] = isAE ? std::uint8_t{ 0x68 } : std::uint8_t{ 0x70 };  // 0x68=ebp(AE), 0x70=esi(SE/VR)
		patch.mov_load[2] = 0x58;
		// cmp reg, 8: opcode 0x83 /7, ModRM selects the register.
		patch.cmp_8[0] = 0x83;
		patch.cmp_8[1] = isAE ? std::uint8_t{ 0xFD } : std::uint8_t{ 0xFE };  // 0xFD=ebp(AE), 0xFE=esi(SE/VR)
		patch.cmp_8[2] = 0x08;
		// jle +5: skip past the 5-byte mov clamp.
		patch.jle_5[0] = 0x7E;
		patch.jle_5[1] = 0x05;
		// mov reg, 8: opcode 0xB8+rd selects the register.
		patch.mov_clamp[0] = isAE ? std::uint8_t{ 0xBD } : std::uint8_t{ 0xBE };  // 0xBD=ebp(AE), 0xBE=esi(SE/VR)
		patch.mov_clamp[1] = 0x08;
		patch.mov_clamp[2] = 0x00;
		patch.mov_clamp[3] = 0x00;
		patch.mov_clamp[4] = 0x00;
		// jmp qword ptr [rip+0]: FF 25 00000000, with the target address stored immediately after.
		patch.jmp_rip[0] = 0xFF;
		patch.jmp_rip[1] = 0x25;
		patch.jmp_rip[2] = 0x00;
		patch.jmp_rip[3] = 0x00;
		patch.jmp_rip[4] = 0x00;
		patch.jmp_rip[5] = 0x00;
		patch.ret_addr = GeometrySkinningBoneFix.address() + 7;

		auto& localTrampoline = SKSE::GetTrampoline();
		SKSE::AllocTrampoline(14 + sizeof(patch));

		void* code = localTrampoline.allocate(sizeof(patch));
		std::memcpy(code, &patch, sizeof(patch));
		localTrampoline.write_branch<5>(GeometrySkinningBoneFix.address(), reinterpret_cast<std::uintptr_t>(code));
	}

	void MainHooks::Update(RE::Main* const a_this)
	{
		//
		_Update(a_this);

		const auto& runtimeData = a_this->GetRuntimeData();

		//
		if (runtimeData.quitGame) {
			Events::ShutdownEvent e;
			Events::Sources::ShutdownEventEventSource::GetSingleton()->SendEvent(&e);
		} else {
			Events::FrameEvent e;
			e.gamePaused = runtimeData.freezeTime;
			Events::Sources::FrameEventSource::GetSingleton()->SendEvent(&e);
		}
	}

	void MainHooks::Unk_sub(void* a_this)
	{
		_Unk_sub(a_this);

		//
		Events::FrameSyncEvent framesyncEvent;
		Events::Sources::FrameSyncEventSource::GetSingleton()->SendEvent(&framesyncEvent);
	}

	bool ActorEquipManagerHooks::func(
		RE::ActorEquipManager* const a_this,
		RE::Actor* a_actor,
		RE::TESBoundObject* a_object,
		RE::ExtraDataList* a_extraData,
		std::uint32_t a_count,
		const RE::BGSEquipSlot* a_slot,
		bool a_queueEquip,
		bool a_forceEquip,
		bool a_playSounds,
		bool a_applyNow,
		const RE::BGSEquipSlot* a_slotToReplace)
	{
		Events::ArmorDetachEvent event;
		event.actor = a_actor;
		event.hasDetached = false;

		//

		Events::Sources::ArmorDetachEventSource::GetSingleton()->SendEvent(&event);

		//
		auto ret = _func(a_this, a_actor, a_object, a_extraData, a_count, a_slot, a_queueEquip, a_forceEquip, a_playSounds, a_applyNow, a_slotToReplace);

		//
		event.hasDetached = true;

		//
		Events::Sources::ArmorDetachEventSource::GetSingleton()->SendEvent(&event);

		//
		return ret;
	}

	RE::NiAVObject* BipedAnimHooks::func(RE::BipedAnim* const a_this, RE::NiNode* armor, RE::BSFadeNode* skeleton, uint32_t a_unk1, void* a_unk2, void* a_unk3, void* a_unk4)
	{
		Events::ArmorAttachEvent armorAtachEvent;

		//
		armorAtachEvent.armorModel = armor;
		armorAtachEvent.skeleton = skeleton;
		armorAtachEvent.attachedNode = nullptr;
		armorAtachEvent.hasAttached = false;

		//
		Events::Sources::ArmorAttachEventSource::GetSingleton()->SendEvent(&armorAtachEvent);

		// tmp fix until we figure out why _func is failing on some stuff.
		std::unordered_map<std::string, std::vector<RE::NiPointer<RE::NiAVObject>>> backupBones;

		//
		if (armor) {
			for (auto& NodeName : BackupNodes) {
				std::vector<RE::NiPointer<RE::NiAVObject>> result;

				//
				RE::NiAVObject* object = armor->GetObjectByName(NodeName);
				RE::BSTriShape* triShape = object ? object->AsTriShape() : nullptr;
				if (triShape) {
					auto size = triShape->GetGeometryRuntimeData().skinInstance->skinData->GetBoneCount();
					for (uint32_t idx = 0; idx < size; idx++)  // all good here
					{
						auto bone = triShape->GetGeometryRuntimeData().skinInstance->bones[idx];
						result.emplace_back(hdt::make_nismart(bone));
					}
				}

				if (result.size() > 0) {
					backupBones.insert({ NodeName, result });
				}
			}
		}

		//
		RE::NiAVObject* ret = _func(a_this, armor, skeleton, a_unk1, a_unk2, a_unk3, a_unk4);

		//
		if (ret) {
			for (auto& NodeName : BackupNodes) {
				RE::NiAVObject* object = ret->GetObjectByName(NodeName);
				RE::BSTriShape* triShape = object ? object->AsTriShape() : nullptr;
				if (triShape) {
					auto size = triShape->GetGeometryRuntimeData().skinInstance->skinData->GetBoneCount();
					for (uint32_t idx = 0; idx < size; idx++) {
						auto bone = triShape->GetGeometryRuntimeData().skinInstance->bones[idx];
						if (bone == nullptr) {
							if (backupBones.contains(NodeName)) {
								bone = triShape->GetGeometryRuntimeData().skinInstance->bones[idx] = backupBones[NodeName][idx].get();
							}
						}
					}
				}
			}
		}

		//
		if (ret) {
			armorAtachEvent.attachedNode = ret;
			armorAtachEvent.hasAttached = true;

			//
			Events::Sources::ArmorAttachEventSource::GetSingleton()->SendEvent(&armorAtachEvent);
		}

		//
		return ret;
	}

	void BSFaceGenNiNodeHooks::SetBoneName_Hook(RE::BSFaceGenModelExtraData* a_fmd, std::uint32_t a_boneIdx, RE::BSFixedString* a_boneName)
	{
		// FMD.bones[] has exactly 8 slots (indices 0-7); any index >= 8 is out-of-bounds
		if (a_boneIdx < 8) {
			_SetBoneName(a_fmd, a_boneIdx, a_boneName);
		}
	}

	void BSFaceGenNiNodeHooks::HookSetBoneName()
	{
		static REL::Relocation<uintptr_t> addr{ REL::VariantID(26303, 26886, 0x3E44E0) };
		_SetBoneName = reinterpret_cast<SetBoneName_t*>(addr.address());
		DetourAttach((PVOID*)&_SetBoneName, (PVOID)SetBoneName_Hook);
	}

	void InstallHighPriority()
	{
		logger::trace("Installing high-priority hooks...");

		MainHooks::Hook();

		logger::trace("...success");
	}

	void InstallLowPriority()
	{
		logger::trace("Installing low-priority hooks...");

		BSFaceGenNiNodeHooks::Hook();

		DetourTransactionBegin();
		DetourUpdateThread(GetCurrentThread());
		ActorEquipManagerHooks::Hook();
		BSFaceGenNiNodeHooks::HookSetBoneName();
		// We use a detour on this instead of modifying the vtable to avoid breaking compatibility with other mods like Mu Joint Fix
		DetourAttach((PVOID*)&BSFaceGenNiNodeHooks::_SkinAllGeometry_Orig, (PVOID)BSFaceGenNiNodeHooks::SkinAllGeometry__Hook);

		DetourTransactionCommit();

		DetourTransactionBegin();
		DetourUpdateThread(GetCurrentThread());
		BipedAnimHooks::Hook();
		DetourTransactionCommit();

		logger::trace("...success");
	}
}
