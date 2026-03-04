#include <detours.h>

#include "skse64/GameData.h"
#include "skse64/GameForms.h"
#include "skse64/GameReferences.h"
#include "skse64/NiObjects.h"
#include "skse64/NiGeometry.h"
#include "skse64/NiExtraData.h"

#include "Hooks.h"
#include "HookEvents.h"
#include "Offsets.h"
#include "skse64/NiNodes.h"
#include "skse64/GameRTTI.h"
#include "skse64_common/SafeWrite.h"
#include <xbyak/xbyak.h>
#include "skse64_common/BranchTrampoline.h"
#include "ActorManager.h"

namespace hdt
{
	class BSFaceGenNiNodeEx : BSFaceGenNiNode
	{
		MEMBER_FN_PREFIX(BSFaceGenNiNodeEx);

	public:
		DEFINE_MEMBER_FN_HOOK(SkinAllGeometry, void, offset::BSFaceGenNiNode_SkinAllGeometry, NiNode* a_skeleton, BSGeometry* a_geometry, char a_unk);
		DEFINE_MEMBER_FN_HOOK(SkinSingleGeometry, void, offset::BSFaceGenNiNode_SkinSingleGeometry, NiNode* a_skeleton, BSGeometry* a_geometry, BSTriShape* a_trishape);

		void ProcessHeadPart(BGSHeadPart* headPart, NiNode* a_skeleton)
		{
			if (headPart)
			{
				NiAVObject* headNode = this->GetObjectByName(&headPart->partName.data);
				if (headNode)
				{
					BSGeometry* headGeo = headNode->GetAsBSGeometry();
					if (headGeo)
						SkinSingleGeometry(a_skeleton, headGeo, nullptr);
				}
				BGSHeadPart* extraPart = NULL;
				for (UInt32 p = 0; p < headPart->extraParts.count; p++)
				{
					if (headPart->extraParts.GetNthItem(p, extraPart))
						ProcessHeadPart(extraPart, a_skeleton);
				}
			}
		}

		void SkinAllGeometryCalls(NiNode* a_skeleton, BSGeometry* a_geometry, char a_unk)
		{
			bool needRegularCall = true;
			if (ActorManager::instance()->skeletonNeedsParts(a_skeleton))
			{
				TESForm* form = LookupFormByID(a_skeleton->m_owner->formID);
				Actor* actor = DYNAMIC_CAST(form, TESForm, Actor);
				if (actor)
				{
					TESNPC* actorBase = DYNAMIC_CAST(actor->baseForm, TESForm, TESNPC);
					UInt32 numHeadParts = 0;
					BGSHeadPart** Headparts = nullptr;
					if (CALL_MEMBER_FN(actorBase, HasOverlays)()) {
						numHeadParts = GetNumActorBaseOverlays(actorBase);
						Headparts = GetActorBaseOverlays(actorBase);
					}
					else {
						numHeadParts = actorBase->numHeadParts;
						Headparts = actorBase->headparts;
					}
					if (Headparts)
					{
						for (UInt32 i = 0; i < numHeadParts; i++) {
							if (Headparts[i]) {
								ProcessHeadPart(Headparts[i], a_skeleton);
							}
						}
					}
					if (a_skeleton->m_owner && a_skeleton->m_owner->formID == 0x14)
						needRegularCall = false;
				}
			}
			if (needRegularCall)
				CALL_MEMBER_FN(this, SkinAllGeometry)(a_skeleton, a_geometry, a_unk);
		}

		void SkinSingleGeometry(NiNode* a_skeleton, BSGeometry* a_geometry, BSTriShape* a_trishape)
		{
			const char* name = "";
			uint32_t formId = 0x0;

			if (a_skeleton->m_owner && a_skeleton->m_owner->baseForm)
			{
				auto bname = DYNAMIC_CAST(a_skeleton->m_owner->baseForm, TESForm, TESFullName);
				if (bname)
					name = bname->GetName();
				auto bnpc = DYNAMIC_CAST(a_skeleton->m_owner->baseForm, TESForm, TESNPC);
				if (bnpc && bnpc->nextTemplate)
					formId = bnpc->nextTemplate->formID;
			}
			_MESSAGE("SkinSingleGeometry %s %d - %s, %s, (formid %08x base form %08x head template form %08x)",
				a_skeleton->m_name, a_skeleton->m_children.m_size, a_geometry->m_name, name,
				a_skeleton->m_owner ? a_skeleton->m_owner->formID : 0x0,
				a_skeleton->m_owner ? a_skeleton->m_owner->baseForm->formID : 0x0, formId);

			SkinSingleHeadGeometryEvent e;
			e.skeleton = a_skeleton;
			e.geometry = a_geometry;
			e.headNode = this;
			g_skinSingleHeadGeometryEventDispatcher.dispatch(e);
		}

		void SkinAllGeometry(NiNode* a_skeleton, BSGeometry* a_geometry, char a_unk)
		{
			const char* name = "";
			uint32_t formId = 0x0;
			if (a_skeleton->m_owner && a_skeleton->m_owner->baseForm)
			{
				auto bname = DYNAMIC_CAST(a_skeleton->m_owner->baseForm, TESForm, TESFullName);
				if (bname)
					name = bname->GetName();
				auto bnpc = DYNAMIC_CAST(a_skeleton->m_owner->baseForm, TESForm, TESNPC);
				if (bnpc && bnpc->nextTemplate)
					formId = bnpc->nextTemplate->formID;
			}
			_MESSAGE("SkinAllGeometry %s %d, %s, (formid %08x base form %08x head template form %08x)",
				a_skeleton->m_name, a_skeleton->m_children.m_size, name,
				a_skeleton->m_owner ? a_skeleton->m_owner->formID : 0x0,
				a_skeleton->m_owner ? a_skeleton->m_owner->baseForm->formID : 0x0, formId);

			SkinAllHeadGeometryEvent e;
			e.skeleton = a_skeleton;
			e.headNode = this;
			g_skinAllHeadGeometryEventDispatcher.dispatch(e);

#ifdef ANNIVERSARY_EDITION
			SkinAllGeometryCalls(a_skeleton, a_geometry, a_unk);
#else
			CALL_MEMBER_FN(this, SkinAllGeometry)(a_skeleton, a_geometry, a_unk);
#endif

			e.hasSkinned = true;
			g_skinAllHeadGeometryEventDispatcher.dispatch(e);
		}
	};

	RelocAddr<uintptr_t> BoneLimit(offset::BSFaceGenModelExtraData_BoneLimit);

	void hookFaceGen()
	{
		DetourAttach((void**)BSFaceGenNiNodeEx::_SkinSingleGeometry_GetPtrAddr(),
			(void*)GetFnAddr(&BSFaceGenNiNodeEx::SkinSingleGeometry));
		DetourAttach((void**)BSFaceGenNiNodeEx::_SkinAllGeometry_GetPtrAddr(),
			(void*)GetFnAddr(&BSFaceGenNiNodeEx::SkinAllGeometry));

		RelocAddr<uintptr_t> addr(offset::BSFaceGenNiNode_SkinSingleGeometry_bug);
		SafeWrite8(addr.GetUIntPtr(), 0x7);

		struct BSFaceGenExtraModelData_BoneCount_Code : Xbyak::CodeGenerator
		{
			BSFaceGenExtraModelData_BoneCount_Code(void* buf) : CodeGenerator(4096, buf)
			{
				Xbyak::Label j_Out;

#ifndef ANNIVERSARY_EDITION
				mov(esi, ptr[rax + 0x58]);
				cmp(esi, 9);
				jl(j_Out);
				mov(esi, 8);
#else
				mov(ebp, ptr[rax + 0x58]);
				cmp(ebp, 9);
				jl(j_Out);
				mov(ebp, 8);
#endif // !ANNIVERSARY_EDITION

				L(j_Out);
				jmp(ptr[rip]);
				dq(BoneLimit.GetUIntPtr() + 0x7);
			}
		};

		void* codeBuf = g_localTrampoline.StartAlloc();
		BSFaceGenExtraModelData_BoneCount_Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(BoneLimit.GetUIntPtr(), uintptr_t(code.getCode()));

	}

	void unhookFaceGen()
	{
		DetourDetach((void**)BSFaceGenNiNodeEx::_SkinSingleGeometry_GetPtrAddr(),
		             (void*)GetFnAddr(&BSFaceGenNiNodeEx::SkinSingleGeometry));
		DetourDetach((void**)BSFaceGenNiNodeEx::_SkinAllGeometry_GetPtrAddr(),
		             (void*)GetFnAddr(&BSFaceGenNiNodeEx::SkinAllGeometry));
	}

	struct Unk001CB0E0
	{
		MEMBER_FN_PREFIX(Unk001CB0E0);

		DEFINE_MEMBER_FN_HOOK(unk001CB0E0, NiAVObject*, offset::ArmorAttachFunction, NiNode* armor, NiNode* skeleton,
		                      void* unk3, char unk4, char unk5, void* unk6);

		NiAVObject* unk001CB0E0(NiNode* armor, NiNode* skeleton, void* unk3, char unk4, char unk5, void* unk6)
		{
			ArmorAttachEvent event;
			event.armorModel = armor;
			event.skeleton = skeleton;
			g_armorAttachEventDispatcher.dispatch(event);

			auto ret = CALL_MEMBER_FN(this, unk001CB0E0)(armor, skeleton, unk3, unk4, unk5, unk6);

			if (ret) {
			event.attachedNode = ret;
			event.hasAttached = true;
			g_armorAttachEventDispatcher.dispatch(event);
			}
			return ret;
		}
	};

	struct UnequipItem
	{
		MEMBER_FN_PREFIX(UnequipItem);

		DEFINE_MEMBER_FN_HOOK(unequipItem, bool, offset::ItemUnequipFunction, Actor* actor, TESForm* item, BaseExtraList* extraData, SInt32 count, BGSEquipSlot* equipSlot, bool unkFlag1, bool preventEquip, bool unkFlag2, bool unkFlag3, void* unk);

		bool unequipItem(Actor* actor, TESForm* item, BaseExtraList* extraData, SInt32 count, BGSEquipSlot* equipSlot, bool unkFlag1, bool preventEquip, bool unkFlag2, bool unkFlag3, void* unk)
		{
			ArmorDetachEvent event;
			event.actor = actor;
			g_armorDetachEventDispatcher.dispatch(event);

			auto ret = CALL_MEMBER_FN(this, unequipItem)(actor, item, extraData, count, equipSlot, unkFlag1, preventEquip, unkFlag2, unkFlag3, unk);

			event.hasDetached = true;
			g_armorDetachEventDispatcher.dispatch(event);
			return ret;
		}
	};

	void hookAttachArmor()
	{
		DetourAttach((void**)Unk001CB0E0::_unk001CB0E0_GetPtrAddr(), (void*)GetFnAddr(&Unk001CB0E0::unk001CB0E0));
	}

	void unhookAttachArmor()
	{
		DetourDetach((void**)Unk001CB0E0::_unk001CB0E0_GetPtrAddr(), (void*)GetFnAddr(&Unk001CB0E0::unk001CB0E0));
	}

	void hookDetachArmor()
	{
		DetourAttach((void**)UnequipItem::_unequipItem_GetPtrAddr(), (void*)GetFnAddr(&UnequipItem::unequipItem));
	}

	void unhookDetachArmor()
	{
		DetourDetach((void**)UnequipItem::_unequipItem_GetPtrAddr(), (void*)GetFnAddr(&UnequipItem::unequipItem));
	}

	struct UnkEngine
	{
		// This is the main loop
		//https://github.com/powerof3/CommonLibSSE/blob/master/include/RE/M/Main.h
		MEMBER_FN_PREFIX(UnkEngine);

		DEFINE_MEMBER_FN_HOOK(onFrame, void, offset::GameLoopFunction);

		void onFrame();

		// members
#ifndef SKYRIMVR
		char unk[0x10];
		bool                         quitGame;                     // 010
		bool                         resetGame;                    // 011
		bool                         fullReset;                    // 012
		bool                         gameActive;                   // 013
		bool                         onIdle;                       // 014
		bool                         reloadContent;                // 015
		bool                         freezeTime;                   // 016
		bool                         freezeNextFrame;              // 017
	};
	static_assert(offsetof(UnkEngine, quitGame) == 0x10);
	static_assert(offsetof(UnkEngine, freezeTime) == 0x16);
#else
		char unk[0x8];
		bool                         quitGame;                     // 008
		bool                         resetGame;                    // 009
		bool                         fullReset;                    // 00a
		bool                         gameActive;                   // 00b
		bool                         onIdle;                       // 00c
		bool                         reloadContent;                // 00d
		bool                         freezeTime;                   // 00e
		bool                         freezeNextFrame;              // 00f
	};
	static_assert(offsetof(UnkEngine, quitGame) == 0x08);
	static_assert(offsetof(UnkEngine, freezeTime) == 0x0e);
#endif

	struct SyncFrame
	{
		MEMBER_FN_PREFIX(SyncFrame);
		DEFINE_MEMBER_FN_HOOK(onFrameSync, void, offset::FrameSyncPoint, INT64 param_1);

		void onFrameSync(INT64 param_1);
	};

	void UnkEngine::onFrame()
	{
		CALL_MEMBER_FN(this, onFrame)();

		if (quitGame)
		{
			g_shutdownEventDispatcher.dispatch(ShutdownEvent());
		}
		else
		{
			FrameEvent e;
			e.gamePaused = this->freezeTime;
			g_frameEventDispatcher.dispatch(e);
		}
	}

	void SyncFrame::onFrameSync(INT64 param_1)
	{
		CALL_MEMBER_FN(this, onFrameSync)(param_1);

		g_frameSyncEventDispatcher.dispatch(FrameSyncEvent());
	}

	void hookEngine()
	{
		DetourAttach((void**)UnkEngine::_onFrame_GetPtrAddr(), (void*)GetFnAddr(&UnkEngine::onFrame));
	}

	void unhookEngine()
	{
		DetourDetach((void**)UnkEngine::_onFrame_GetPtrAddr(), (void*)GetFnAddr(&UnkEngine::onFrame));
	}

	void hookSyncFrame()
	{
		DetourAttach((void**)SyncFrame::_onFrameSync_GetPtrAddr(), (void*)GetFnAddr(&SyncFrame::onFrameSync));
	}

	void unhookSyncFrame()
	{
		DetourDetach((void**)SyncFrame::_onFrameSync_GetPtrAddr(), (void*)GetFnAddr(&SyncFrame::onFrameSync));
	}

	void hookAll()
	{
		DetourRestoreAfterWith();
		DetourTransactionBegin();
		hookEngine();
		hookAttachArmor();
		hookDetachArmor();
		hookFaceGen();
		hookSyncFrame();
		DetourTransactionCommit();
	}

	void unhookAll()
	{
		DetourRestoreAfterWith();
		DetourTransactionBegin();
		unhookEngine();
		unhookAttachArmor();
		unhookDetachArmor();
		unhookFaceGen();
		unhookSyncFrame();
		DetourTransactionCommit();
	}
}
