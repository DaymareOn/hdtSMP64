#include "VariantSelector.h"

#include <cctype>
#include <fstream>
#include <string>
#include <string_view>

namespace loader
{
	namespace
	{
		std::string_view trim(std::string_view s) noexcept
		{
			const auto isSpace = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
			while (!s.empty() && isSpace(s.front())) {
				s.remove_prefix(1);
			}
			while (!s.empty() && isSpace(s.back())) {
				s.remove_suffix(1);
			}
			return s;
		}

		std::string toLowerAscii(std::string_view s)
		{
			std::string out;
			out.reserve(s.size());
			for (const char c : s) {
				out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
			}
			return out;
		}
	}

	ForceResult readForcedVariant(const std::filesystem::path& iniFile) noexcept
	{
		ForceResult result{};

		std::ifstream ifs(iniFile);
		if (!ifs) {
			return result;  // no INI -> auto, and not an explicit choice
		}

		std::string line;
		while (std::getline(ifs, line)) {
			std::string_view view = trim(line);
			if (view.empty() || view.front() == ';' || view.front() == '#' || view.front() == '[') {
				continue;  // blank, comment, or [section] header
			}
			const auto eq = view.find('=');
			if (eq == std::string_view::npos) {
				continue;
			}
			const std::string key = toLowerAscii(trim(view.substr(0, eq)));
			if (key != "forcevariant") {
				continue;
			}

			const std::string value = toLowerAscii(trim(view.substr(eq + 1)));
			if (value == "auto") {
				// Explicit "auto" means: ignore past crashes and let detection decide.
				result.variant.reset();
				result.explicitChoice = true;
			} else {
				Variant parsed{};
				if (variantFromToken(value, parsed)) {
					result.variant = parsed;
					result.explicitChoice = true;
				}
				// An unrecognized value falls through as a non-explicit "auto" so a typo cannot pin
				// the user to a broken state.
			}
			// Last meaningful ForceVariant wins; keep scanning in case of duplicates.
		}

		return result;
	}

	Selection buildSelection(Variant cpuMax, Variant ceiling, std::optional<Variant> forced)
	{
		Variant start;
		if (forced.has_value()) {
			// Honour the pin, but never above what the silicon supports.
			start = (*forced < cpuMax) ? *forced : cpuMax;
		} else {
			// Auto: bounded by both the hardware and the crash ceiling.
			start = (cpuMax < ceiling) ? cpuMax : ceiling;
		}

		Selection selection;
		selection.start = start;
		for (const Variant v : kVariantsHighToLow) {
			if (!(start < v)) {  // v <= start
				selection.candidates.push_back(v);
			}
		}
		return selection;
	}
}
