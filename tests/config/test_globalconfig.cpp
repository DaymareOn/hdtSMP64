// Behaviour tests for hdt::parseConfigJson / serializeConfigJson. The parser sits on a trust boundary
// (a user-editable file), so the cases below focus on the failure modes that must never crash or corrupt
// the simulation: malformed input, wrong types, out-of-range numbers, and missing keys all fall back to
// defaults / clamps. Round-trip identity guarantees the menu can save and reload without drift.
#include <doctest/doctest.h>

#include "GlobalConfig.h"

using hdt::GlobalConfig;
using hdt::parseConfigJson;
using hdt::serializeConfigJson;

TEST_CASE("defaults: empty/invalid input yields the default config")
{
	const GlobalConfig def;

	CHECK(parseConfigJson("") == def);
	CHECK(parseConfigJson("not json at all {{{") == def);
	CHECK(parseConfigJson("[]") == def);  // valid JSON, wrong root type
	CHECK(parseConfigJson("\"a string\"") == def);
	CHECK(parseConfigJson("{}") == def);  // object, no sections
}

TEST_CASE("round-trip: serialize then parse is identity for a clamped config")
{
	GlobalConfig c;  // all defaults are already in-range
	c.logLevel = 5;
	c.disableSMPHairWhenWigEquipped = true;
	c.useRealTime = true;
	c.maximumActiveSkeletons = 42;
	c.budgetMs = 7.5f;
	c.erp = 0.33f;
	c.windEnabled = false;
	c.distanceForMaxWind = 1234.0f;
	c.modsDir = "C:/Mods/staging";
	c.backupNodeByName = { "Virtual Hands", "Virtual Body" };
	c.outputFontScale = 1.4f;
	c.overlayFontScale = 0.8f;

	const GlobalConfig round = parseConfigJson(serializeConfigJson(c));
	CHECK(round == c);
}

TEST_CASE("missing keys keep their defaults")
{
	// Only logLevel is supplied; every other field must stay at its default.
	const GlobalConfig c = parseConfigJson(R"({"smp":{"logLevel":2}})");
	const GlobalConfig def;

	CHECK(c.logLevel == 2);
	CHECK(c.numIterations == def.numIterations);
	CHECK(c.windStrength == doctest::Approx(def.windStrength));
}

TEST_CASE("out-of-range numbers are clamped, not rejected")
{
	const GlobalConfig c = parseConfigJson(R"({
		"outputFontScale": 99.0, "overlayFontScale": 0.01,
		"smp": { "logLevel": 99, "budgetMs": 999.0, "sampleSize": 0, "minScreenSizePercent": -5 },
		"solver": { "numIterations": 1, "erp": 5.0, "min-fps": 5000, "maxSubSteps": 0 },
		"wind": { "windStrength": -10.0, "distanceForMaxWind": 99999.0 }
	})");

	CHECK(c.logLevel == 5);                                                    // [0,5]
	CHECK(c.budgetMs == doctest::Approx(20.0f));                               // [0.1,20]
	CHECK(c.sampleSize == 1);                                                  // [1,50]
	CHECK(c.minScreenSizePercent == doctest::Approx(0.0f));                    // [0,100]
	CHECK(c.numIterations == 4);                                               // [4,128]
	CHECK(c.erp == doctest::Approx(1.0f));                                     // [0.01,1.0]
	CHECK(c.minFps == 300);                                                    // [1,300]
	CHECK(c.maxSubSteps == 1);                                                 // [1,60]
	CHECK(c.windStrength == doctest::Approx(0.0f));                            // [0,1000]
	CHECK(c.distanceForMaxWind == doctest::Approx(10000.0f));                  // [0,10000]
	CHECK(c.outputFontScale == doctest::Approx(GlobalConfig::maxFontScale));   // [0.6,2.0]
	CHECK(c.overlayFontScale == doctest::Approx(GlobalConfig::minFontScale));  // [0.6,2.0]
}

TEST_CASE("wrong-typed fields fall back to defaults")
{
	const GlobalConfig def;
	// numbers given as strings, bool given as number, string given as bool, array given as object.
	const GlobalConfig c = parseConfigJson(R"({
		"smp": { "logLevel": "high", "clampRotations": 1, "backupNodeByName": {} },
		"validation": { "mods-dir": true }
	})");

	CHECK(c.logLevel == def.logLevel);
	CHECK(c.clampRotations == def.clampRotations);
	CHECK(c.backupNodeByName.empty());
	CHECK(c.modsDir.empty());
}

TEST_CASE("string arrays drop non-string elements")
{
	const GlobalConfig c = parseConfigJson(R"({"smp":{"backupNodeByName":["A", 3, "B", null, "C"]}})");
	CHECK(c.backupNodeByName == std::vector<std::string>{ "A", "B", "C" });
}

TEST_CASE("unknown keys are ignored without affecting known ones")
{
	const GlobalConfig c = parseConfigJson(R"({
		"smp": { "logLevel": 4, "enableCuda": true, "someFutureField": 123 },
		"totallyUnknownSection": { "x": 1 }
	})");
	CHECK(c.logLevel == 4);
}

TEST_CASE("layering: a user layer overrides base for present keys, keeps base otherwise")
{
	// base stands in for the shipped configs.json defaults.
	GlobalConfig base;
	base.logLevel = 3;
	base.windStrength = 7.0f;
	base.modsDir = "C:/base";

	// The user layer (userConfigs.json) sets only logLevel and mods-dir.
	const GlobalConfig layered =
		parseConfigJson(R"({"smp":{"logLevel":1},"validation":{"mods-dir":"C:/user"}})", base);

	CHECK(layered.logLevel == 1);                          // overridden by the user layer
	CHECK(layered.modsDir == "C:/user");                   // overridden by the user layer
	CHECK(layered.windStrength == doctest::Approx(7.0f));  // untouched -> keeps the base (shipped) value
}

TEST_CASE("layering: an empty or invalid user layer leaves base unchanged")
{
	GlobalConfig base;
	base.numIterations = 42;
	base.useRealTime = true;
	CHECK(parseConfigJson("", base) == base);
	CHECK(parseConfigJson("not json", base) == base);
}

TEST_CASE("equality distinguishes differing configs (Presets highlight)")
{
	GlobalConfig a;
	GlobalConfig b;
	CHECK(a == b);
	b.windStrength = a.windStrength + 1.0f;
	CHECK_FALSE(a == b);
}
