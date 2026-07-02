#pragma once

#include <string>

namespace hdt
{
	struct GlobalConfig;

	extern int g_logLevel;

	// The menu-language override the config was loaded with (a locale code like "fr_fr"; empty = follow
	// Skyrim's own language). applyConfig writes it here and the Localization loader reads it, so both the
	// string table and the menu's Language dropdown agree on the current choice. See GlobalConfig::locale.
	extern std::string g_locale;

	// The menu's font scales (Output panels + log viewer, and the gameplay overlay), adjusted by the menu's
	// A-/A+ buttons and persisted through userConfigs.json like every other setting. See GlobalConfig.
	extern float g_outputFontScale;
	extern float g_overlayFontScale;

	// Copy a parsed/clamped GlobalConfig into the live physics singletons + globals. Used by loadConfig and
	// by the menu's Presets page (apply a preset's values live before persisting them).
	void applyConfig(const GlobalConfig& config);

	// Read data/skse/plugins/hdtSkinnedMeshConfigs/configs.json from disk, parse it (fail-closed --- a
	// missing or corrupt file leaves stock defaults), and copy every value into the physics singletons.
	void loadConfig();

	// Dump the live config values to the log at debug level.
	void logConfig();

	// Snapshot the live engine settings back into a GlobalConfig (the inverse of applyConfig). The menu
	// uses this to render the current state and to know what to persist.
	GlobalConfig readConfig();

	// The shipped defaults: configs.json parsed on its own, with no userConfigs.json layer. The menu's
	// per-setting "reset to default" restores each field to the value here (the value FSMP ships with, not
	// the struct's built-in fallback). Parsed once and cached --- configs.json never changes while running.
	const GlobalConfig& shippedDefaults();

	// Persist the live settings to configs.json. Raw-merges over the existing file so keys this build does
	// not model (e.g. enableCuda) survive; writes atomically (temp file + rename).
	void saveUserSettings();

	// The canonical "apply settings" sequence shared by `smp reset` and the in-game menu: reload config,
	// log it, clear tracked skeletons, and reset all physics systems.
	void applyConfigReset();
}
