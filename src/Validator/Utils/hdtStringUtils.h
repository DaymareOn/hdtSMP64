#pragma once

// Domain-specific string helpers for the Validator (XSD/XPath, asset paths, NIF,
// validation messages). General-purpose string/path helpers live one level up in
// src/hdtStringUtils.h, included below so existing consumers keep seeing them.
#include "../../hdtStringUtils.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>
#include <utility>

namespace hdt
{
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

	// Strip the leading "data/" prefix using case-insensitive matching.
	// Backslashes are normalized to forward slashes first.
	inline std::string stripDataPrefix(std::string path)
	{
		std::replace(path.begin(), path.end(), '\\', '/');

		if (path.size() >= 5) {
			bool hasDataPrefix =
				std::tolower(static_cast<unsigned char>(path[0])) == 'd' &&
				std::tolower(static_cast<unsigned char>(path[1])) == 'a' &&
				std::tolower(static_cast<unsigned char>(path[2])) == 't' &&
				std::tolower(static_cast<unsigned char>(path[3])) == 'a' &&
				path[4] == '/';
			if (hasDataPrefix)
				return path.substr(5);
		}

		return path;
	}

	// Resolve a raw XML path to a filesystem location, trying the path as-is first, then with a
	// "data/" prefix. Returns the resolved path and whether it exists on disk.
	inline std::pair<std::string, bool> ResolveXMLPath(const std::string& rawPath)
	{
		if (rawPath.empty())
			return { {}, false };

		std::string xmlPath = rawPath;
		std::replace(xmlPath.begin(), xmlPath.end(), '\\', '/');

		std::error_code ec;
		std::filesystem::path xmlFsPath = xmlPath;
		if (!std::filesystem::exists(xmlFsPath, ec))
			xmlFsPath = "data/" + xmlPath;

		return { PathToUtf8(xmlFsPath), std::filesystem::exists(xmlFsPath, ec) };
	}

	// Strip the namespace prefix (everything up to and including the first ':') from an XML element name.
	inline std::string StripXmlNamespacePrefix(std::string s)
	{
		auto pos = s.find(':');
		if (pos != std::string::npos)
			s.erase(0, pos + 1);
		return s;
	}

	// Extract the base stem from a normalised NIF path, stripping _0/_1 weight-variant suffixes.
	// "foo_0.nif" -> "foo",  "foo_1.nif" -> "foo",  "foo.nif" -> "foo".
	inline std::string GetNifPairBaseStem(const std::string& normPath)
	{
		if (normPath.size() > 6 &&
			(normPath.substr(normPath.size() - 6) == "_0.nif" ||
				normPath.substr(normPath.size() - 6) == "_1.nif"))
			return normPath.substr(0, normPath.size() - 6);
		return normPath.size() > 4 ? normPath.substr(0, normPath.size() - 4) : normPath;
	}

	// Parse a validation message of the form "value '<v>' is out of range..." and
	// return the boundary ("0" or "1") nearest to the extracted value.
	inline std::string ExtractOutOfRangeClampTarget(const std::string& message)
	{
		const std::string markerStart = "value '";
		const std::string markerEnd = "' is out of range";
		auto start = message.find(markerStart);
		if (start == std::string::npos)
			return "1";
		start += markerStart.size();
		auto end = message.find(markerEnd, start);
		if (end == std::string::npos || end <= start)
			return "1";
		try {
			const float value = std::stof(message.substr(start, end - start));
			return value <= 0.0f ? "0" : "1";
		} catch (...) {
			return "1";
		}
	}

}  // namespace hdt
