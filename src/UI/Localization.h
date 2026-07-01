#pragma once

#include <string>
#include <vector>

namespace hdt::loc
{
	// Load the configuration menu's localized strings. The locale is the config's optional "locale" override
	// (hdt::g_locale, a code like "fr_fr"); if empty, it is auto-detected from Skyrim's own sLanguage setting.
	// It then loads localization/<locale>.json (with base/regional fallback). A missing file or a missing key
	// falls back to the English source string, so the menu always works even with no localization installed.
	// Safe to call again at runtime (the Language dropdown does) to switch languages live.
	void load();

	// Return the localized text for an English source string, or the source string itself when there is no
	// translation for it. The returned pointer stays valid until the next load(), which is enough for ImGui
	// (the table is loaded once at startup and never mutated during rendering).
	const char* tr(const char* english);

	// The locale codes we actually ship, discovered by listing the localization/ folder (e.g. "en", "fr_fr").
	// The in-game Language dropdown lists these; nothing is hardcoded.
	std::vector<std::string> availableLocales();

	// The locale file currently loaded ("" when none matched and English source strings are in use). The
	// dropdown uses it to show which entry is active when the override is "auto".
	const std::string& currentLocale();
}
