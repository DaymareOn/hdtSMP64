#pragma once

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
	// Trim ASCII whitespace from both ends of a string.
	inline std::string TrimAsciiWhitespace(const std::string& s)
	{
		auto start = s.find_first_not_of(" \t\r\n");
		if (start == std::string::npos)
			return "";
		auto end = s.find_last_not_of(" \t\r\n");
		return s.substr(start, end - start + 1);
	}

	// Strip a namespace prefix token (prefix + ':') from an XPath expression when
	// it appears on XPath boundaries, avoiding false positives inside literals.
	inline std::string StripNamespacePrefix(const std::string& expr, const std::string& prefix)
	{
		if (prefix.empty()) {
			return expr;
		}

		const std::string token = prefix + ":";
		std::string result;
		result.reserve(expr.size());
		for (size_t i = 0; i < expr.size();) {
			if (i + token.size() <= expr.size() && expr.compare(i, token.size(), token) == 0) {
				bool atBoundary = (i == 0);
				if (!atBoundary && !result.empty()) {
					char prev = result.back();
					atBoundary = (prev == '/' || prev == '[' || prev == '(' ||
								  prev == '|' || prev == ' ' || prev == '\t' ||
								  prev == ':');
				}
				if (atBoundary) {
					i += token.size();
					continue;
				}
			}
			result += expr[i++];
		}
		return result;
	}

	// Build a sorted, comma-separated string from a set of strings.
	inline std::string JoinSortedSet(const std::unordered_set<std::string>& values)
	{
		std::vector<std::string> sorted(values.begin(), values.end());
		std::sort(sorted.begin(), sorted.end());
		std::string result;
		for (const auto& value : sorted) {
			if (!result.empty())
				result += ", ";
			result += value;
		}
		return result;
	}

}  // namespace hdt
