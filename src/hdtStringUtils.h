#pragma once

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

// General-purpose string and path helpers shared across the whole plugin (runtime
// XML parsing and the Validator alike). Domain-specific helpers (XSD/XPath/NIF/
// validation messages) live in Validator/Utils/hdtStringUtils.h instead.
namespace hdt
{
	// Replace every non-overlapping occurrence of `from` in `s` with `to` in place.
	inline void ReplaceAllInPlace(std::string& s, const std::string& from, const std::string& to)
	{
		if (from.empty())
			return;
		size_t pos = 0;
		while ((pos = s.find(from, pos)) != std::string::npos) {
			s.replace(pos, from.size(), to);
			pos += to.size();
		}
	}

	// Convert a std::filesystem::path to a UTF-8 std::string.
	// generic_u8string() returns char8_t data; reinterpret_cast is required to
	// produce a plain std::string without an explicit loop or locale dependency.
	inline std::string PathToUtf8(const std::filesystem::path& fp)
	{
		auto u8 = fp.generic_u8string();
		return { reinterpret_cast<const char*>(u8.data()), u8.size() };
	}

	// Trim ASCII whitespace from both ends of a string.
	inline std::string TrimAsciiWhitespace(const std::string& s)
	{
		auto start = s.find_first_not_of(" \t\r\n");
		if (start == std::string::npos)
			return "";
		auto end = s.find_last_not_of(" \t\r\n");
		return s.substr(start, end - start + 1);
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

	// Normalize a filesystem-like path string for case-insensitive comparisons.
	// Converts backslashes to forward slashes and lowercases all characters.
	inline std::string NormalizePathForComparison(std::string path)
	{
		std::transform(path.begin(), path.end(), path.begin(), [](unsigned char c) {
			return c == '\\' ? '/' : static_cast<char>(std::tolower(c));
		});
		return path;
	}
}  // namespace hdt
