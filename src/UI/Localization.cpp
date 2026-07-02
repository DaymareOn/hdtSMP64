#include "UI/Localization.h"

#include "config.h"  // hdt::g_locale --- the locale override, set by loadConfig before the menu registers

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <rapidjson/document.h>

namespace hdt::loc
{
	namespace
	{
		// English-source-string -> translated-string. Empty when the locale is English or no file was found,
		// in which case tr() returns the source string unchanged.
		std::unordered_map<std::string, std::string> g_strings;

		// The locale file load() resolved to ("" = none matched, English source strings in use).
		std::string g_currentLocale;

		constexpr auto LOC_DIR = "data/skse/plugins/hdtSkinnedMeshConfigs/localization";

		std::string toLower(std::string s)
		{
			std::transform(s.begin(), s.end(), s.begin(),
				[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			return s;
		}

		std::string readFileBytes(const std::filesystem::path& path)
		{
			std::ifstream in(path, std::ios::binary);
			if (!in)
				return {};
			std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
			// Strip a UTF-8 BOM (see config.cpp) so an editor-saved file still parses.
			if (bytes.size() >= 3 && static_cast<unsigned char>(bytes[0]) == 0xEF &&
				static_cast<unsigned char>(bytes[1]) == 0xBB && static_cast<unsigned char>(bytes[2]) == 0xBF)
				bytes.erase(0, 3);
			return bytes;
		}

		bool has(const std::vector<std::string>& v, const std::string& s)
		{
			return std::find(v.begin(), v.end(), s) != v.end();
		}

		// Choose the best available locale file for a wanted code: an exact match ("fr_fr"), else the base
		// language of a regional code ("fr_fr" -> "fr"), else any regional variant of a base code
		// ("fr" -> "fr_ca"). Empty string means "no file, use English".
		std::string bestMatch(const std::string& want, const std::vector<std::string>& avail)
		{
			if (want.empty())
				return {};
			if (has(avail, want))
				return want;
			const auto us = want.find('_');
			if (us != std::string::npos && has(avail, want.substr(0, us)))
				return want.substr(0, us);
			for (const auto& l : avail)
				if (l.rfind(want + "_", 0) == 0)  // l starts with want + "_"
					return l;
			return {};
		}

		// Skyrim's sLanguage is a language NAME ("ENGLISH", "FRENCH"); normalise it to an ISO 639-1 code so
		// we can match it against the locale files. This tiny map is a fact of Skyrim's fixed language set,
		// not a list of the locales we support (those come from availableLocales()).
		std::string gameLanguageCode()
		{
			auto* setting = RE::GetINISetting("sLanguage:General");
			const std::string name = (setting && setting->GetType() == RE::Setting::Type::kString && setting->GetString()) ? toLower(setting->GetString()) : "";
			static const std::unordered_map<std::string, std::string> iso = {
				{ "english", "en" }, { "french", "fr" }, { "german", "de" }, { "italian", "it" },
				{ "spanish", "es" }, { "polish", "pl" }, { "russian", "ru" }, { "japanese", "ja" },
				{ "chinese", "zh" }, { "czech", "cs" }, { "portuguese", "pt" }
			};
			const auto it = iso.find(name);
			return it != iso.end() ? it->second : "en";
		}
	}

	std::vector<std::string> availableLocales()
	{
		// Discovered by listing the folder, not hardcoded: every <locale>.json there is one available locale
		// (lower-cased stem, e.g. "en", "fr_fr"). The Language dropdown lists exactly what is installed.
		std::vector<std::string> out;
		std::error_code ec;
		for (const auto& e : std::filesystem::directory_iterator(LOC_DIR, ec))
			if (e.path().extension() == ".json")
				out.push_back(toLower(e.path().stem().string()));
		return out;
	}

	const std::string& currentLocale()
	{
		return g_currentLocale;
	}

	void load()
	{
		g_strings.clear();
		g_currentLocale.clear();

		const std::vector<std::string> avail = availableLocales();

		// The config's "locale" override (g_locale, set by loadConfig / the dropdown) wins; otherwise
		// auto-detect from the game language. Either way we resolve to a file that actually exists (with
		// base/regional fallback); "" means no match -> use the English source strings.
		const std::string want = toLower(hdt::g_locale);
		const std::string chosen = !want.empty() ? bestMatch(want, avail) : bestMatch(gameLanguageCode(), avail);

		const std::string bytes = chosen.empty() ? std::string{} : readFileBytes(std::string(LOC_DIR) + "/" + chosen + ".json");
		if (bytes.empty()) {
			logger::info("FSMP menu: no localization file matched; using English");
			return;  // tr() falls back to the English source strings
		}

		rapidjson::Document doc;
		doc.Parse(bytes.data(), bytes.size());
		if (doc.HasParseError() || !doc.IsObject()) {
			logger::warn("FSMP menu: localization file '{}' is not a valid JSON object; using English", chosen);
			return;
		}

		// Iterate members directly (not GetObject(), which <windows.h> redefines to GetObjectA via macro).
		for (auto it = doc.MemberBegin(); it != doc.MemberEnd(); ++it)
			if (it->name.IsString() && it->value.IsString())
				g_strings.emplace(std::string(it->name.GetString(), it->name.GetStringLength()),
					std::string(it->value.GetString(), it->value.GetStringLength()));

		g_currentLocale = chosen;
		logger::info("FSMP menu: loaded {} strings for locale '{}'", g_strings.size(), chosen);
	}

	const char* tr(const char* english)
	{
		const auto it = g_strings.find(english);
		return it != g_strings.end() ? it->second.c_str() : english;
	}
}
