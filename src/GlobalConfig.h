#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace hdt
{
	// GlobalConfig is the in-memory, game-free representation of FSMP's global tunables --- everything the
	// user can set in configs.json. It is a plain struct with no dependency on SKSE, Bullet, or the physics
	// singletons, so the parse/serialize/clamp logic can be unit-tested on its own (see tests/config).
	// config.cpp owns the thin glue that copies these values into SkyrimPhysicsWorld / ActorManager /
	// g_logLevel / g_validationConfig and back.
	//
	// The JSON is grouped into smp/solver/wind/validation sections whose keys are the tag names (e.g.
	// "min-fps", "mods-dir", wind."enabled"). At load these are filled in layers (see config.cpp): the shipped
	// configs.json supplies the defaults, then the user's userConfigs.json is applied on top, so a setting the
	// user never changed keeps the shipped default (and survives an FSMP update, which only replaces
	// configs.json). The struct's own field defaults are just the last resort if configs.json itself is gone.
	struct GlobalConfig
	{
		// --- <smp> ---
		// logLevel is the FILE value 0..5 (0 = Fatal ... 5 = Debug). config.cpp inverts it to spdlog's
		// internal scale on apply; we keep the user-facing value here so the menu slider maps directly.
		int logLevel = 0;
		bool disableSMPHairWhenWigEquipped = false;
		bool hideSMPHairWhenInvisible = true;
		bool clampRotations = true;
		float rotationSpeedLimit = 10.0f;
		bool unclampedResets = true;
		float unclampedResetAngle = 120.0f;
		bool useRealTime = false;
		float minCullingDistance = 500.0f;
		bool autoAdjustMaxSkeletons = true;
		int maximumActiveSkeletons = 20;
		float budgetMs = 3.5f;
		int sampleSize = 5;
		bool disable1stPersonViewPhysics = false;
		bool skipDeadActors = false;
		float minScreenSizePercent = 0.0f;
		// backupNodeByName has no menu control; it is preserved purely so a round-trip never drops it.
		std::vector<std::string> backupNodeByName;

		// --- <solver> ---
		int numIterations = 10;
		float erp = 0.2f;
		int minFps = 60;  // JSON/XML tag "min-fps"
		int maxSubSteps = 4;

		// --- <wind> ---
		bool windEnabled = true;  // JSON/XML tag wind."enabled"
		float windStrength = 2.0f;
		float distanceForNoWind = 50.0f;
		float distanceForMaxWind = 3000.0f;

		// --- <validation> ---
		std::string modsDir;

		// --- top level ---
		// Menu-language override, a locale code like "fr_fr" (empty = follow Skyrim's own language). It lives
		// at the JSON root, not in a section, because it configures the UI rather than the physics. The menu's
		// Language dropdown writes it here so the choice persists in userConfigs.json and survives updates.
		std::string locale;

		// Font scales for the menu's text views (the A-/A+ buttons): one for the Output panels + log viewer,
		// one for the gameplay overlay. UI settings like locale, so they live at the JSON root too.
		static constexpr float minFontScale = 0.6f;
		static constexpr float maxFontScale = 2.0f;
		float outputFontScale = 1.0f;
		float overlayFontScale = 1.0f;

		// Value-equality across every field. Used by the Presets page to highlight which preset (if any)
		// the live config currently matches.
		bool operator==(const GlobalConfig&) const = default;
	};

	// Parse a JSON byte buffer into a GlobalConfig, layered over `base`: a key present in the buffer overrides
	// base, a key absent keeps base's value. Passing the default base (all built-in defaults) parses one file
	// against the defaults; passing an already-parsed config layers a second file on top (userConfigs.json
	// over configs.json). TRUST BOUNDARY: the input is a user-editable file, so this never throws --- a
	// malformed document, a wrong-typed field, or an out-of-range number falls back to base for that field,
	// and numbers are clamped to the ranges the engine and the menu enforce.
	GlobalConfig parseConfigJson(std::string_view bytes, GlobalConfig base = {});

	// Serialize a GlobalConfig back to pretty-printed JSON, mirroring the section layout (smp/solver/wind/
	// validation) and the section tag names. serialize -> parse is an identity for any clamped config.
	std::string serializeConfigJson(const GlobalConfig& config);
}
