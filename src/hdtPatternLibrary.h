#pragma once

#include "hdtXmlPatternExpander.h"

#include <vector>

namespace hdt
{
	/// Loads and caches the shared pattern definitions from the global patterns/ folder
	/// (data/skse/plugins/hdtSkinnedMeshConfigs/patterns/*.xml), in deterministic filename order. Built
	/// once on first call and shared thereafter. Pass the result to expandPatterns via
	/// PatternOptions::libraries so the runtime loader and every validator resolve the same cross-mod
	/// patterns. Authors drop a file of <pattern-default>s here (namespaced with author=) to publish
	/// patterns other mods can use.
	const std::vector<PatternLibrary>& getGlobalPatternLibraries();
}
