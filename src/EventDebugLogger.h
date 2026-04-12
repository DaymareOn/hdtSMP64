#pragma once

#include "Events.h"

namespace hdt
{
	class EventDebugLogger :
		public RE::BSTEventSink<Events::ArmorAttachEvent>,
		public RE::BSTEventSink<RE::TESCellAttachDetachEvent>,
		public RE::BSTEventSink<RE::TESMoveAttachDetachEvent>
	{
	protected:
		RE::BSEventNotifyControl ProcessEvent(const RE::TESCellAttachDetachEvent* evn, RE::BSTEventSource<RE::TESCellAttachDetachEvent>* dispatcher) override;
		RE::BSEventNotifyControl ProcessEvent(const RE::TESMoveAttachDetachEvent* evn, RE::BSTEventSource<RE::TESMoveAttachDetachEvent>* dispatcher) override;
		RE::BSEventNotifyControl ProcessEvent(const Events::ArmorAttachEvent*, RE::BSTEventSource<Events::ArmorAttachEvent>*) override;
	};
}
