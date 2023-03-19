#include "HookEvents.h"
#include "EventDispatcherImpl.h"

namespace hdt
{
	EventDispatcherImpl<FrameEvent> g_frameEventDispatcher;
	EventDispatcherImpl<ShutdownEvent> g_shutdownEventDispatcher;
	EventDispatcherImpl<ArmorAttachEvent> g_armorAttachEventDispatcher;
	EventDispatcherImpl<ArmorDetachEvent> g_armorDetachEventDispatcher;
	EventDispatcherImpl<SkinAllHeadGeometryEvent> g_skinAllHeadGeometryEventDispatcher;
	EventDispatcherImpl<SkinSingleHeadGeometryEvent> g_skinSingleHeadGeometryEventDispatcher;
}
