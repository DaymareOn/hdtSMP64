#include "config.h"

#include "GlobalConfig.h"
#include "Hooks.h"
#include "Validator/hdtAssetValidator.h"
#include "hdtSkyrimPhysicsWorld.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>

namespace hdt
{
	int g_logLevel;
	std::string g_locale;
	float g_outputFontScale = 1.0f;
	float g_overlayFontScale = 1.0f;

	namespace
	{
		// configs.json ships with FSMP and holds the defaults; an FSMP update may replace it. userConfigs.json
		// is written by the menu and never shipped, so the user's settings survive updates. It is layered on
		// top of configs.json at load, so a setting the user never touched keeps the shipped default.
		constexpr auto CONFIG_PATH = "data/skse/plugins/hdtSkinnedMeshConfigs/configs.json";
		constexpr auto USER_CONFIG_PATH = "data/skse/plugins/hdtSkinnedMeshConfigs/userConfigs.json";

		std::string readFileBytes(const std::string& path)
		{
			std::ifstream in(path, std::ios::binary);
			if (!in)
				return {};
			std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
			// Tolerate a UTF-8 BOM a text editor may have prepended; rapidjson::Parse does not skip it.
			if (bytes.size() >= 3 && static_cast<unsigned char>(bytes[0]) == 0xEF &&
				static_cast<unsigned char>(bytes[1]) == 0xBB && static_cast<unsigned char>(bytes[2]) == 0xBF)
				bytes.erase(0, 3);
			return bytes;
		}
	}

	// Copy a parsed/clamped GlobalConfig into the live physics singletons + globals. This is the single
	// place that knows how each field maps onto the engine (e.g. the inverted log level, and min-fps also
	// driving the fixed timestep). backupNodeByName is overwritten wholesale so a reload never duplicates.
	void applyConfig(const GlobalConfig& c)
	{
		g_logLevel = 5 - std::clamp(c.logLevel, 0, 5);
		spdlog::set_level(static_cast<spdlog::level::level_enum>(g_logLevel));
		spdlog::flush_on(static_cast<spdlog::level::level_enum>(g_logLevel));

		Hooks::BipedAnimHooks::BackupNodes = c.backupNodeByName;

		auto* a = ActorManager::instance();
		a->m_disableSMPHairWhenWigEquipped = c.disableSMPHairWhenWigEquipped;
		a->m_minCullingDistance = c.minCullingDistance;
		a->m_maxActiveSkeletons = c.maximumActiveSkeletons;
		a->m_autoAdjustMaxSkeletons = c.autoAdjustMaxSkeletons;
		a->m_disable1stPersonViewPhysics = c.disable1stPersonViewPhysics;
		a->m_skipDeadActors = c.skipDeadActors;
		a->m_minScreenSizePercent = c.minScreenSizePercent;

		auto* w = SkyrimPhysicsWorld::get();
		w->m_clampRotations = c.clampRotations;
		w->m_rotationSpeedLimit = c.rotationSpeedLimit;
		w->m_unclampedResets = c.unclampedResets;
		w->m_unclampedResetAngle = c.unclampedResetAngle;
		w->m_budgetMs = c.budgetMs;
		w->m_useRealTime = c.useRealTime;
		w->m_sampleSize = c.sampleSize;
		w->min_fps = c.minFps;
		w->m_timeTick = 1.0f / static_cast<float>(c.minFps);
		w->m_maxSubSteps = c.maxSubSteps;
		w->getSolverInfo().m_numIterations = c.numIterations;
		w->getSolverInfo().m_erp = c.erp;
		w->m_enableWind = c.windEnabled;
		w->m_windStrength = c.windStrength;
		w->m_distanceForNoWind = c.distanceForNoWind;
		w->m_distanceForMaxWind = c.distanceForMaxWind;

		g_validationConfig.modsDir = c.modsDir;

		g_locale = c.locale;  // the Localization loader and the menu's Language dropdown both read this
		g_outputFontScale = c.outputFontScale;
		g_overlayFontScale = c.overlayFontScale;
	}

	void loadConfig()
	{
		// Defaults from the shipped configs.json, then the user's userConfigs.json layered on top.
		GlobalConfig cfg = parseConfigJson(readFileBytes(CONFIG_PATH));
		cfg = parseConfigJson(readFileBytes(USER_CONFIG_PATH), cfg);
		applyConfig(cfg);
	}

	GlobalConfig readConfig()
	{
		GlobalConfig c;

		c.logLevel = std::clamp(5 - g_logLevel, 0, 5);
		c.backupNodeByName = Hooks::BipedAnimHooks::BackupNodes;

		auto* a = ActorManager::instance();
		c.disableSMPHairWhenWigEquipped = a->m_disableSMPHairWhenWigEquipped;
		c.minCullingDistance = a->m_minCullingDistance;
		c.maximumActiveSkeletons = a->m_maxActiveSkeletons;
		c.autoAdjustMaxSkeletons = a->m_autoAdjustMaxSkeletons;
		c.disable1stPersonViewPhysics = a->m_disable1stPersonViewPhysics;
		c.skipDeadActors = a->m_skipDeadActors;
		c.minScreenSizePercent = a->m_minScreenSizePercent;

		auto* w = SkyrimPhysicsWorld::get();
		c.clampRotations = w->m_clampRotations;
		c.rotationSpeedLimit = w->m_rotationSpeedLimit;
		c.unclampedResets = w->m_unclampedResets;
		c.unclampedResetAngle = w->m_unclampedResetAngle;
		c.budgetMs = w->m_budgetMs;
		c.useRealTime = w->m_useRealTime;
		c.sampleSize = w->m_sampleSize;
		c.minFps = w->min_fps;
		c.maxSubSteps = w->m_maxSubSteps;
		c.numIterations = w->getSolverInfo().m_numIterations;
		c.erp = w->getSolverInfo().m_erp;
		c.windEnabled = w->m_enableWind;
		c.windStrength = w->m_windStrength;
		c.distanceForNoWind = w->m_distanceForNoWind;
		c.distanceForMaxWind = w->m_distanceForMaxWind;

		c.modsDir = g_validationConfig.modsDir;
		c.locale = g_locale;
		c.outputFontScale = g_outputFontScale;
		c.overlayFontScale = g_overlayFontScale;

		return c;
	}

	const GlobalConfig& shippedDefaults()
	{
		// Parse configs.json alone (no userConfigs.json overlay): these are the "reset to default" targets.
		// Cached in a function-local static so the file is read once, lazily, on first use by the menu.
		static const GlobalConfig defs = parseConfigJson(readFileBytes(CONFIG_PATH));
		return defs;
	}

	void saveUserSettings()
	{
		// Write to userConfigs.json (never configs.json), so an FSMP update replacing configs.json can't wipe
		// the user's settings. We write a clean serialization of only the fields we model, so a setting that
		// a later FSMP version removes leaves no stale key behind. (Variant-only keys like enableCuda live in
		// the shipped configs.json, which we never rewrite, so they are unaffected.)
		const std::string merged = serializeConfigJson(readConfig());

		// Atomic write: emit to a sibling temp file, then rename over the target so a crash mid-write can
		// never leave a truncated userConfigs.json that would fail to parse on next launch.
		std::error_code ec;
		const std::filesystem::path target(USER_CONFIG_PATH);
		const std::filesystem::path tmp = std::filesystem::path(target).concat(".tmp");
		{
			std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
			if (!out) {
				logger::warn("saveUserSettings: could not open {} for writing", tmp.string());
				return;
			}
			out.write(merged.data(), static_cast<std::streamsize>(merged.size()));
		}
		std::filesystem::rename(tmp, target, ec);
		if (ec) {
			// Rename can fail if the destination is locked; fall back to a direct overwrite.
			std::ofstream out(target, std::ios::binary | std::ios::trunc);
			if (out)
				out.write(merged.data(), static_cast<std::streamsize>(merged.size()));
			std::filesystem::remove(tmp, ec);
		}
	}

	void applyConfigReset()
	{
		loadConfig();
		logConfig();

		const RE::MenuOpenCloseEvent e{ "", false };
		ActorManager::instance()->ProcessEvent(&e, nullptr);
		SkyrimPhysicsWorld::get()->resetSystems();
	}

	void logConfig()
	{
		auto* w = SkyrimPhysicsWorld::get();
		auto* a = ActorManager::instance();

#define LOG(name, val) logger::debug("config: " name " = {}", val)
		LOG("solver.numIterations", w->getSolverInfo().m_numIterations);
		LOG("solver.erp", w->getSolverInfo().m_erp);
		LOG("solver.min-fps", w->min_fps);
		LOG("solver.maxSubSteps", w->m_maxSubSteps);

		LOG("wind.windStrength", w->m_windStrength);
		LOG("wind.enabled", w->m_enableWind);
		LOG("wind.distanceForNoWind", w->m_distanceForNoWind);
		LOG("wind.distanceForMaxWind", w->m_distanceForMaxWind);

		LOG("smp.logLevel", 5 - g_logLevel);

		for (auto& item : Hooks::BipedAnimHooks::BackupNodes)
			logger::debug("config: smp.backupNodeByName += {}", item);

		LOG("smp.disableSMPHairWhenWigEquipped", a->m_disableSMPHairWhenWigEquipped);
		LOG("smp.clampRotations", w->m_clampRotations);
		LOG("smp.rotationSpeedLimit", w->m_rotationSpeedLimit);
		LOG("smp.unclampedResets", w->m_unclampedResets);
		LOG("smp.unclampedResetAngle", w->m_unclampedResetAngle);
		LOG("smp.budgetMS", w->m_budgetMs);
		LOG("smp.useRealTime", w->m_useRealTime);
		LOG("smp.minCullingDistance", a->m_minCullingDistance);
		LOG("smp.maximumActiveSkeletons", a->m_maxActiveSkeletons);
		LOG("smp.autoAdjustMaxSkeletons", a->m_autoAdjustMaxSkeletons);
		LOG("smp.sampleSize", w->m_sampleSize);
		LOG("smp.disable1stPersonViewPhysics", a->m_disable1stPersonViewPhysics);
		LOG("smp.skipDeadActors", a->m_skipDeadActors);
		LOG("smp.minScreenSizePercent", a->m_minScreenSizePercent);
#undef LOG
	}
}
