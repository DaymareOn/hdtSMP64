#include "EventDebugLogger.h"

namespace hdt
{
	RE::BSEventNotifyControl EventDebugLogger::ProcessEvent(const RE::TESCellAttachDetachEvent* evn, RE::BSTEventSource<RE::TESCellAttachDetachEvent>*)
	{
		if (evn && evn->reference && evn->reference->formType == RE::Character::FORMTYPE) {
			logger::debug(
				"Received TESCellAttachDetachEvent(formID {:08X}, name {}, attached={}).",
				evn->reference->formID,
				evn->reference->GetBaseObject()->GetFormEditorID(),
				evn->attached ? "true" : "false");
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl EventDebugLogger::ProcessEvent(const RE::TESMoveAttachDetachEvent* evn, RE::BSTEventSource<RE::TESMoveAttachDetachEvent>*)
	{
		if (evn && evn->movedRef && evn->movedRef->formType == RE::Character::FORMTYPE) {
			logger::debug(
				"Received TESMoveAttachDetachEvent(formID {:08X}, name {}, attached={}).",
				evn->movedRef->formID, evn->movedRef->GetBaseObject()->GetFormEditorID(), evn->isCellAttached ? "true" : "false");
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl EventDebugLogger::ProcessEvent(const Events::ArmorAttachEvent* e, RE::BSTEventSource<Events::ArmorAttachEvent>*)
	{
		logger::debug(
			"Received ArmorAttachEvent(armorModel={} ({:016X}), skeleton={} ({:016X}), attachedNode={} ({:016X}), hasAttached={}).",
			e->armorModel ? e->armorModel->name : "null",
			(uintptr_t)e->armorModel,
			e->skeleton ? e->skeleton->name : "null",
			(uintptr_t)e->skeleton,
			e->attachedNode ? e->attachedNode->name : "null",
			(uintptr_t)e->attachedNode,
			static_cast<uintptr_t>(e->hasAttached) ? "true" : "false");

		return RE::BSEventNotifyControl::kContinue;
	}
}
