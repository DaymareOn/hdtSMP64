#include "GlobalConfig.h"

#include <algorithm>
#include <cmath>
#include <type_traits>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

// This translation unit is deliberately free of SKSE / Bullet / spdlog so the doctest suite in
// tests/config can compile it directly without the game's force-included PCH.

namespace hdt
{
	namespace
	{
		template <typename T>
		T clampv(T v, T lo, T hi)
		{
			return v < lo ? lo : (v > hi ? hi : v);
		}

		// Look up a key in a section object; null when absent. Collapses the FindMember / MemberEnd dance so
		// each reader below is just a type check plus an extract.
		const rapidjson::Value* find(const rapidjson::Value& obj, const char* key)
		{
			auto it = obj.FindMember(key);
			return it == obj.MemberEnd() ? nullptr : &it->value;
		}

		// Read obj[key] as T, or def when the key is absent or the JSON type doesn't match. One template for
		// every field type: bool, integers and floats (an int is rounded from the JSON number), strings, and
		// string arrays. It always fails closed. Numeric ranges are per-field, so callers wrap the result in
		// clampv() to keep a hand-edited file from pushing a value the simulation would choke on.
		template <typename T>
		T read(const rapidjson::Value& obj, const char* key, const T& def)
		{
			const auto* v = find(obj, key);
			if (!v)
				return def;

			if constexpr (std::is_same_v<T, bool>)
				return v->IsBool() ? v->GetBool() : def;
			else if constexpr (std::is_same_v<T, std::string>)
				return v->IsString() ? std::string(v->GetString(), v->GetStringLength()) : def;
			else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
				if (!v->IsArray())
					return def;
				std::vector<std::string> out;
				for (const auto& e : v->GetArray())
					if (e.IsString())
						out.emplace_back(e.GetString(), e.GetStringLength());
				return out;
			} else {
				if (!v->IsNumber())
					return def;
				if constexpr (std::is_integral_v<T>)
					return static_cast<T>(std::lround(v->GetDouble()));
				else
					return static_cast<T>(v->GetDouble());
			}
		}

		// A missing section is not an error: every field then keeps its default. We hand back an empty object
		// so the per-field readers run uniformly whether or not the section was present.
		const rapidjson::Value& sectionOrEmpty(const rapidjson::Value& root, const char* key,
			const rapidjson::Value& empty)
		{
			const auto* v = find(root, key);
			return v && v->IsObject() ? *v : empty;
		}
	}

	GlobalConfig parseConfigJson(std::string_view bytes, GlobalConfig base)
	{
		GlobalConfig c = base;

		if (bytes.empty())
			return c;  // nothing to parse; data() may be null -> avoid handing rapidjson a null pointer

		rapidjson::Document doc;
		doc.Parse(bytes.data(), bytes.size());
		if (doc.HasParseError() || !doc.IsObject())
			return c;  // unreadable file -> all defaults (fail-closed)

		rapidjson::Value empty(rapidjson::kObjectType);
		const auto& smp = sectionOrEmpty(doc, "smp", empty);
		const auto& solver = sectionOrEmpty(doc, "solver", empty);
		const auto& wind = sectionOrEmpty(doc, "wind", empty);
		const auto& validation = sectionOrEmpty(doc, "validation", empty);

		// --- smp --- (clampv keeps hand-edited numbers inside the engine's accepted range)
		c.logLevel = clampv(read(smp, "logLevel", c.logLevel), 0, 5);
		c.enableNPCFaceParts = read(smp, "enableNPCFaceParts", c.enableNPCFaceParts);
		c.disableSMPHairWhenWigEquipped = read(smp, "disableSMPHairWhenWigEquipped", c.disableSMPHairWhenWigEquipped);
		c.clampRotations = read(smp, "clampRotations", c.clampRotations);
		c.rotationSpeedLimit = clampv(read(smp, "rotationSpeedLimit", c.rotationSpeedLimit), 0.0f, 100.0f);
		c.unclampedResets = read(smp, "unclampedResets", c.unclampedResets);
		c.unclampedResetAngle = clampv(read(smp, "unclampedResetAngle", c.unclampedResetAngle), 0.0f, 360.0f);
		c.useRealTime = read(smp, "useRealTime", c.useRealTime);
		c.minCullingDistance = clampv(read(smp, "minCullingDistance", c.minCullingDistance), 0.0f, 10000.0f);
		c.autoAdjustMaxSkeletons = read(smp, "autoAdjustMaxSkeletons", c.autoAdjustMaxSkeletons);
		c.maximumActiveSkeletons = clampv(read(smp, "maximumActiveSkeletons", c.maximumActiveSkeletons), 0, 200);
		c.budgetMs = clampv(read(smp, "budgetMs", c.budgetMs), 0.1f, 20.0f);
		c.sampleSize = clampv(read(smp, "sampleSize", c.sampleSize), 1, 50);
		c.disable1stPersonViewPhysics = read(smp, "disable1stPersonViewPhysics", c.disable1stPersonViewPhysics);
		c.skipDeadActors = read(smp, "skipDeadActors", c.skipDeadActors);
		c.minScreenSizePercent = clampv(read(smp, "minScreenSizePercent", c.minScreenSizePercent), 0.0f, 100.0f);
		c.backupNodeByName = read(smp, "backupNodeByName", c.backupNodeByName);

		// --- solver ---
		c.numIterations = clampv(read(solver, "numIterations", c.numIterations), 4, 128);
		c.erp = clampv(read(solver, "erp", c.erp), 0.01f, 1.0f);
		c.minFps = clampv(read(solver, "min-fps", c.minFps), 1, 300);
		c.maxSubSteps = clampv(read(solver, "maxSubSteps", c.maxSubSteps), 1, 60);

		// --- wind ---
		c.windEnabled = read(wind, "enabled", c.windEnabled);
		c.windStrength = clampv(read(wind, "windStrength", c.windStrength), 0.0f, 1000.0f);
		c.distanceForNoWind = clampv(read(wind, "distanceForNoWind", c.distanceForNoWind), 0.0f, 10000.0f);
		c.distanceForMaxWind = clampv(read(wind, "distanceForMaxWind", c.distanceForMaxWind), 0.0f, 10000.0f);

		// --- validation ---
		c.modsDir = read(validation, "mods-dir", c.modsDir);

		// --- top level --- (UI settings sit at the root rather than in a section)
		c.locale = read(doc, "locale", c.locale);
		c.outputFontScale = clampv(read(doc, "outputFontScale", c.outputFontScale),
			GlobalConfig::minFontScale, GlobalConfig::maxFontScale);
		c.overlayFontScale = clampv(read(doc, "overlayFontScale", c.overlayFontScale),
			GlobalConfig::minFontScale, GlobalConfig::maxFontScale);

		return c;
	}

	std::string serializeConfigJson(const GlobalConfig& c)
	{
		rapidjson::StringBuffer buf;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> w(buf);

		w.StartObject();

		// Only emit locale when the user set one, so a clean userConfigs.json stays "auto" by omission and the
		// round-trip (serialize -> parse) is still identity (an absent locale parses back to the default "").
		if (!c.locale.empty()) {
			w.Key("locale");
			w.String(c.locale.c_str(), static_cast<rapidjson::SizeType>(c.locale.size()));
		}
		w.Key("outputFontScale");
		w.Double(c.outputFontScale);
		w.Key("overlayFontScale");
		w.Double(c.overlayFontScale);

		w.Key("smp");
		w.StartObject();
		w.Key("logLevel");
		w.Int(c.logLevel);
		w.Key("enableNPCFaceParts");
		w.Bool(c.enableNPCFaceParts);
		w.Key("disableSMPHairWhenWigEquipped");
		w.Bool(c.disableSMPHairWhenWigEquipped);
		w.Key("clampRotations");
		w.Bool(c.clampRotations);
		w.Key("rotationSpeedLimit");
		w.Double(c.rotationSpeedLimit);
		w.Key("unclampedResets");
		w.Bool(c.unclampedResets);
		w.Key("unclampedResetAngle");
		w.Double(c.unclampedResetAngle);
		w.Key("useRealTime");
		w.Bool(c.useRealTime);
		w.Key("minCullingDistance");
		w.Double(c.minCullingDistance);
		w.Key("autoAdjustMaxSkeletons");
		w.Bool(c.autoAdjustMaxSkeletons);
		w.Key("maximumActiveSkeletons");
		w.Int(c.maximumActiveSkeletons);
		w.Key("budgetMs");
		w.Double(c.budgetMs);
		w.Key("sampleSize");
		w.Int(c.sampleSize);
		w.Key("disable1stPersonViewPhysics");
		w.Bool(c.disable1stPersonViewPhysics);
		w.Key("skipDeadActors");
		w.Bool(c.skipDeadActors);
		w.Key("minScreenSizePercent");
		w.Double(c.minScreenSizePercent);
		w.Key("backupNodeByName");
		w.StartArray();
		for (const auto& n : c.backupNodeByName)
			w.String(n.c_str(), static_cast<rapidjson::SizeType>(n.size()));
		w.EndArray();
		w.EndObject();

		w.Key("solver");
		w.StartObject();
		w.Key("numIterations");
		w.Int(c.numIterations);
		w.Key("erp");
		w.Double(c.erp);
		w.Key("min-fps");
		w.Int(c.minFps);
		w.Key("maxSubSteps");
		w.Int(c.maxSubSteps);
		w.EndObject();

		w.Key("wind");
		w.StartObject();
		w.Key("enabled");
		w.Bool(c.windEnabled);
		w.Key("windStrength");
		w.Double(c.windStrength);
		w.Key("distanceForNoWind");
		w.Double(c.distanceForNoWind);
		w.Key("distanceForMaxWind");
		w.Double(c.distanceForMaxWind);
		w.EndObject();

		w.Key("validation");
		w.StartObject();
		w.Key("mods-dir");
		w.String(c.modsDir.c_str(), static_cast<rapidjson::SizeType>(c.modsDir.size()));
		w.EndObject();

		w.EndObject();

		return { buf.GetString(), buf.GetSize() };
	}
}
