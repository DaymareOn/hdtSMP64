#pragma once

#include <functional>
#include <string>
#include <stdexcept>

#include "ActorManager.h"
#include "config.h"
#include "EventDebugLogger.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "Hooks.h"
#include "HookEvents.h"
#ifndef SKYRIMVR
#include "skse64/skse64/PluginAPI.h"
#else
#include "sksevr/skse64/PluginAPI.h"
#endif
#include "dhdtOverrideManager.h"
#include "dhdtPapyrusFunctions.h"

constexpr auto PAPYRUS_CLASS_NAME = "DynamicHDT";

constexpr auto OVERRIDE_SAVE_PATH = "Data/SKSE/Plugins/hdtOverrideSaves/";

namespace hdt {
	namespace util {
		UInt32 splitArmorAddonFormID(std::string nodeName);

		std::string UInt32toString(UInt32 formID);
	}
}
