#pragma once

#include "NetImmerseUtils.h"          // readAllFile2
#include "hdtXmlPatternExpander.h"

#include <string>
#include <utility>
#include <vector>

namespace hdt
{
	/// A physics XML document ready for a validator to parse: the file's text with any <pattern>
	/// macros already expanded, plus the diagnostics and source-map from that expansion.
	struct PhysicsXmlSource
	{
		std::string xml;  ///< expanded document text; empty if the file was missing/empty
		bool ok = true;   ///< false if pattern expansion failed (xml then holds the raw input)
		std::vector<PatternDiag> diags;
		PatternSourceMap sourceMap;
	};

	/// Reads a loose physics XML file and expands its <pattern> macros, so every validator parses the
	/// same fully-expanded document the runtime loader builds. This is the single read+expand seam the
	/// four validator entry points share (the runtime loader reads via the BSA VFS and calls
	/// expandPatterns directly).
	///
	/// A missing/empty file yields an empty xml -- callers keep their existing file-not-found handling.
	/// A malformed pattern yields ok=false plus diagnostics; by convention only the XSD validator turns
	/// those into report entries, while the other validators bail quietly, exactly as they already do
	/// for malformed XML.
	inline PhysicsXmlSource readAndExpandPhysicsXml(const std::string& path)
	{
		PhysicsXmlSource s;
		const std::string raw = readAllFile2(path.c_str());
		if (raw.empty())
			return s;
		PatternExpansion ex = expandPatterns(raw);
		s.xml = std::move(ex.xml);
		s.ok = ex.ok;
		s.diags = std::move(ex.diags);
		s.sourceMap = std::move(ex.sourceMap);
		return s;
	}
}
