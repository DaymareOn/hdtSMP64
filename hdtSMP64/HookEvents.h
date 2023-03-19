#pragma once

#include "skse64/NiObjects.h"

#include "EventDispatcherImpl.h"
#include "skse64/NiNodes.h"

namespace hdt
{
	struct SkinAllHeadGeometryEvent
	{
		NiNode* skeleton = nullptr;
		BSFaceGenNiNode* headNode = nullptr;
		bool hasSkinned = false;
	};

	struct SkinSingleHeadGeometryEvent
	{
		NiNode* skeleton = nullptr;
		BSFaceGenNiNode* headNode = nullptr;
		BSGeometry* geometry = nullptr;
	};

	struct ArmorAttachEvent
	{
		NiNode* armorModel = nullptr;
		NiNode* skeleton = nullptr;
		NiAVObject* attachedNode = nullptr;
		bool hasAttached = false;
	};

	struct ArmorDetachEvent
	{
		Actor* actor = nullptr;
		TESForm* item = nullptr;
		BaseExtraList* extraData = nullptr;
		SInt32 count = 0;
		BGSEquipSlot* equipSlot = nullptr;
		bool unkFlag1 = false;
		bool preventEquip = false;
		bool unkFlag2 = false;
		bool unkFlag3 = false;
		void* unk = nullptr;
		bool hasDetached = false;
		bool result = false; // Possibly if the armor has managed to detach itself (cursed armor can't?)
	};

	struct FrameEvent
	{
		bool gamePaused;
	};

	struct ShutdownEvent
	{
	};

	extern EventDispatcherImpl<FrameEvent> g_frameEventDispatcher;
	extern EventDispatcherImpl<ShutdownEvent> g_shutdownEventDispatcher;
	extern EventDispatcherImpl<ArmorAttachEvent> g_armorAttachEventDispatcher;
	extern EventDispatcherImpl<ArmorDetachEvent> g_armorDetachEventDispatcher;
	extern EventDispatcherImpl<SkinAllHeadGeometryEvent> g_skinAllHeadGeometryEventDispatcher;
	extern EventDispatcherImpl<SkinSingleHeadGeometryEvent> g_skinSingleHeadGeometryEventDispatcher;
}
