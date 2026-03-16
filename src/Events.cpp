#include "Events.h"
#include "ActorManager.h"
#include "hdtSkyrimPhysicsWorld.h"

namespace Events
{
	namespace Sources
	{
		FrameEventSource* FrameEventSource::GetSingleton()
		{
			static FrameEventSource singleton;
			return std::addressof(singleton);
		}

		FrameSyncEventSource* FrameSyncEventSource::GetSingleton()
		{
			static FrameSyncEventSource singleton;
			return std::addressof(singleton);
		}

		ShutdownEventEventSource* ShutdownEventEventSource::GetSingleton()
		{
			static ShutdownEventEventSource singleton;
			return std::addressof(singleton);
		}

		SkinAllHeadGeometryEventSource* SkinAllHeadGeometryEventSource::GetSingleton()
		{
			static SkinAllHeadGeometryEventSource singleton;
			return std::addressof(singleton);
		}

		SkinSingleHeadGeometryEventSource* SkinSingleHeadGeometryEventSource::GetSingleton()
		{
			static SkinSingleHeadGeometryEventSource singleton;
			return std::addressof(singleton);
		}

		ArmorAttachEventSource* ArmorAttachEventSource::GetSingleton()
		{
			static ArmorAttachEventSource singleton;
			return std::addressof(singleton);
		}

		ArmorDetachEventSource* ArmorDetachEventSource::GetSingleton()
		{
			static ArmorDetachEventSource singleton;
			return std::addressof(singleton);
		}
	}

	namespace Sinks
	{
		FreezeEventHandler* FreezeEventHandler::GetSingleton()
		{
			static FreezeEventHandler singleton;
			return std::addressof(singleton);
		}

		void FreezeEventHandler::Register()
		{
			RE::UI::GetSingleton()->AddEventSink(GetSingleton());
		}

		void FreezeEventHandler::Unregister()
		{
			RE::UI::GetSingleton()->RemoveEventSink(GetSingleton());
		}

		RE::BSEventNotifyControl FreezeEventHandler::ProcessEvent(const RE::MenuOpenCloseEvent* a_event, [[maybe_unused]] RE::BSTEventSource<RE::MenuOpenCloseEvent>* a_eventSource)
		{
			if (a_event && a_event->opening && (a_event->menuName == "Loading Menu" || a_event->menuName == "RaceSex Menu")) {
				logger::debug("{} detected, scheduling physics reset on world un-suspend.", a_event->menuName);
				hdt::SkyrimPhysicsWorld::get()->suspend(true);
			}

			if (a_event && !a_event->opening && (a_event->menuName == "RaceSex Menu")) {
				logger::debug("Racemenu closed, reloading meshes.");
				hdt::ActorManager::instance()->ProcessEvent(a_event, a_eventSource);
			}

			return RE::BSEventNotifyControl::kContinue;
		}
	}

	void Register()
	{
		Sinks::FreezeEventHandler::Register();
	}

	void Unregister()
	{
		Sinks::FreezeEventHandler::Unregister();
	}
}
