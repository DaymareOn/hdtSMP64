#pragma once

#include "skse64/NiObjects.h"
#include "skse64/GameReferences.h"

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
		bool hasDetached = false;
	};

	struct FrameEvent
	{
		bool gamePaused;
	};

	struct FrameSyncEvent
	{
	};

	struct ShutdownEvent
	{
	};

	extern EventDispatcherImpl<FrameEvent> g_frameEventDispatcher;
	extern EventDispatcherImpl<FrameSyncEvent> g_frameSyncEventDispatcher;
	extern EventDispatcherImpl<ShutdownEvent> g_shutdownEventDispatcher;
	extern EventDispatcherImpl<ArmorAttachEvent> g_armorAttachEventDispatcher;
	extern EventDispatcherImpl<ArmorDetachEvent> g_armorDetachEventDispatcher;
	extern EventDispatcherImpl<SkinAllHeadGeometryEvent> g_skinAllHeadGeometryEventDispatcher;
	extern EventDispatcherImpl<SkinSingleHeadGeometryEvent> g_skinSingleHeadGeometryEventDispatcher;
}
