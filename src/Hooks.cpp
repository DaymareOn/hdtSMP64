#include "ActorManager.h"

#include "Hooks.h"
#include "Events.h"

//
#include <xbyak/xbyak.h>

namespace Hooks
{
	void BSFaceGenNiNodeHooks::ProcessHeadPart(RE::BSFaceGenNiNode* const a_this, RE::BGSHeadPart* headPart, RE::NiNode* a_skeleton, bool a_unk)
	{
		//
		if (headPart) 
		{
			RE::NiAVObject* headNode = a_this->GetObjectByName(headPart->formEditorID);
			if (headNode) 
			{
				RE::BSGeometry* headGeometry = headNode->AsGeometry();
				if (headGeometry)
				{
					SkinSingleGeometry__Hook(a_this, a_skeleton, headGeometry, a_unk);
				}
			}

			//
			for (auto it : headPart->extraParts)
			{
				ProcessHeadPart(a_this, it, a_skeleton, a_unk);
			}
		}
	}

	void BSFaceGenNiNodeHooks::SkinAllGeometryCalls(RE::BSFaceGenNiNode* const a_this, RE::NiNode* a_skeleton, bool a_unk)
	{
		bool needRegularCall = true;
		if (hdt::ActorManager::instance()->skeletonNeedsParts(a_skeleton)) 
		{
			RE::TESForm* form = RE::TESForm::LookupByID(a_skeleton->GetUserData()->formID);
			RE::Actor* actor = skyrim_cast<RE::Actor*>(form);
			if (actor) 
			{
				RE::TESNPC* actorBase = skyrim_cast<RE::TESNPC*>(actor->data.objectReference);
				uint32_t numHeadParts = 0;
				RE::BGSHeadPart** Headparts = nullptr;

				if (actorBase->HasOverlays()) 
				{
					numHeadParts = actorBase->GetNumBaseOverlays();
					Headparts = actorBase->GetBaseOverlays();
				} 
				else 
				{
					numHeadParts = actorBase->numHeadParts;
					Headparts = actorBase->headParts;
				}

				if (Headparts) 
				{
					for (uint32_t i = 0; i < numHeadParts; i++) 
					{
						if (Headparts[i]) 
						{
							ProcessHeadPart(a_this, Headparts[i], a_skeleton, a_unk);
						}
					}
				}

				if (a_skeleton->GetUserData() && a_skeleton->GetUserData()->formID == 0x14) 
				{
					needRegularCall = false;
				}
			}
		}

		if (needRegularCall) 
		{
			SkinAllGeometry(a_this, a_skeleton, a_unk);
		}
	}

	void BSFaceGenNiNodeHooks::SkinSingleGeometry__Hook(RE::BSFaceGenNiNode* const a_this, RE::NiNode* a_skeleton, RE::BSGeometry* a_triShape, bool a_unk)
	{
		//
		const char* name = "";
		uint32_t formId = 0x0;

		//
		if (a_skeleton->GetUserData() && a_skeleton->GetUserData()->GetBaseObject()) 
		{
			auto bname = skyrim_cast<RE::TESFullName*>(a_skeleton->GetUserData()->GetBaseObject());
			if (bname) 
			{
				name = bname->GetFullName();
			}

			auto bnpc = skyrim_cast<RE::TESNPC*>(a_skeleton->GetUserData()->GetBaseObject());
			if (bnpc && bnpc->faceNPC) 
			{
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
		if (a_skeleton->GetUserData() && a_skeleton->GetUserData()->data.objectReference) 
		{
			auto bname = skyrim_cast<RE::TESFullName*>(a_skeleton->GetUserData()->data.objectReference);
			if (bname) 
			{
				name = bname->GetFullName();
			}

			auto bnpc = skyrim_cast<RE::TESNPC*>(a_skeleton->GetUserData()->data.objectReference);
			if (bnpc && bnpc->faceNPC) 
			{
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
		if (REL::Module::IsAE())
		{
			SkinAllGeometryCalls(a_this, a_skeleton, a_unk);
		}
		else
		{
			SkinAllGeometry(a_this, a_skeleton, a_unk);
		}
		
		//
		e.hasSkinned = true;

		//
		Events::Sources::SkinAllHeadGeometryEventSource::GetSingleton()->SendEvent(&e);
	}

	void BSFaceGenNiNodeHooks::SkinAllGeometry(RE::BSFaceGenNiNode* const a_this, RE::NiNode* a_skeleton, bool a_unk)
	{
		if (a_skeleton)
		{
			const auto& children = a_this->GetChildren();
			if (children.size() > 0)
			{
				for (int i = 0; i < children.size(); i++)
				{
					auto child = children[i];
					if (child)
					{
						auto triShape = child->AsTriShape();
						if (triShape)
						{
							SkinSingleGeometry__Hook(a_this, a_skeleton, triShape, a_unk);
						}
					}
				}
			}
		}
	}

	void BSFaceGenNiNodeHooks::ApplyBoneLimitFix()
	{
		REL::Relocation<uintptr_t> GeometrySkinningBoneFix { REL::VariantID(24330, 24836, 0x37ADD0), REL::VariantOffset(0x58, 0x75, 0x58) };

		struct BoneLimitFix : 
			public Xbyak::CodeGenerator
		{
			BoneLimitFix(uintptr_t a_returnAddr) : 
				Xbyak::CodeGenerator()
			{
				Xbyak::Label ret;

				auto clamp_bone_count = [&](Xbyak::Reg32 reg) {
					mov(reg, ptr[rax + 0x58]);  // skinData->boneCount
					cmp(reg, 8);                // compare with limit
					jle(ret);                   // jump if <= 8 (keep value)
					mov(reg, 8);                // clamp to 8 if > 8
				};

				Xbyak::Reg32 boneReg = !REL::Module::IsAE() ? esi : ebp;
				clamp_bone_count(boneReg);

				//
				L(ret);
				jmp(ptr[rip]);
				dq(a_returnAddr);
			}
		};

		//
		BoneLimitFix code(GeometrySkinningBoneFix.address() + 7);

		//
		auto& localTrampoline = SKSE::GetTrampoline();
		SKSE::AllocTrampoline(14 + code.getSize());
		
		//
		localTrampoline.write_branch<5>(GeometrySkinningBoneFix.address(), localTrampoline.allocate(code));
	}
	
	void MainHooks::Update(RE::Main* const a_this)
	{
		//
		_Update(a_this);

		// why doesn't this class have a GetRuntimeData() helper? the offsets are borked with VR support enabled.
		bool quitGame = REL::RelocateMember<bool>(a_this, 0x10, 0x8);

		//
		if (quitGame)
		{
			Events::ShutdownEvent e;
			Events::Sources::ShutdownEventEventSource::GetSingleton()->SendEvent(&e);
		} 
		else 
		{
			Events::FrameEvent e;
			e.gamePaused = a_this->freezeTime;
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

	bool ActorEquipManagerHooks::func
	(
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
		const RE::BGSEquipSlot* a_slotToReplace
	)
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
		if (armor)
		{
			for (auto& NodeName : BackupNodes) 
			{
				std::vector<RE::NiPointer<RE::NiAVObject>> result;

				//
				RE::NiAVObject* object = armor->GetObjectByName(NodeName);
				RE::BSTriShape* triShape = object ? object->AsTriShape() : nullptr;
				if (triShape) 
				{
					auto size = triShape->GetGeometryRuntimeData().skinInstance->skinData->bones;
					for (int idx = 0; idx < size; idx++)  // all good here
					{
						auto bone = triShape->GetGeometryRuntimeData().skinInstance->bones[idx];
						result.emplace_back(hdt::make_nismart(bone));
					}
				}

				if (result.size() > 0) 
				{
					backupBones.insert({ NodeName, result });
				}
			}
		}

		//
		RE::NiAVObject* ret = _func(a_this, armor, skeleton, a_unk1, a_unk2, a_unk3, a_unk4);

		//
		if (ret)
		{
			for (auto& NodeName : BackupNodes) 
			{
				RE::NiAVObject* object = ret->GetObjectByName(NodeName);
				RE::BSTriShape* triShape = object ? object->AsTriShape() : nullptr;
				if (triShape) 
				{
					auto size = triShape->GetGeometryRuntimeData().skinInstance->skinData->bones;
					for (int idx = 0; idx < size; idx++) 
					{
						auto bone = triShape->GetGeometryRuntimeData().skinInstance->bones[idx];
						if (bone == nullptr) 
						{
							if (backupBones.contains(NodeName)) 
							{
								bone = triShape->GetGeometryRuntimeData().skinInstance->bones[idx] = backupBones[NodeName][idx].get();
							}
						}
					}
				}
			}
		}

		//
		if (ret) 
		{
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
		static REL::Relocation<uintptr_t> addr{ REL::RelocationID(26303, 26886) };
		_SetBoneName = reinterpret_cast<SetBoneName_t*>(addr.address());
		DetourAttach((PVOID*)&_SetBoneName, (PVOID)SetBoneName_Hook);
	}

	void Install()
	{
		logger::trace("Hooking...");

		// generic hooks
		BSFaceGenNiNodeHooks::Hook();
		MainHooks::Hook();
		
		//
		DetourTransactionBegin();
		DetourUpdateThread(GetCurrentThread());
		ActorEquipManagerHooks::Hook();
		BSFaceGenNiNodeHooks::HookSetBoneName();
		DetourTransactionCommit();

		//
		DetourTransactionBegin();
		DetourUpdateThread(GetCurrentThread());
		BipedAnimHooks::Hook();
		DetourTransactionCommit();

		//
		logger::trace("...success");
	}
}
