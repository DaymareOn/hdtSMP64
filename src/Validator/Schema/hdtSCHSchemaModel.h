#pragma once

#include "../Validators/hdtSCHValidator.h"

#include <string>
#include <vector>

namespace hdt
{
	// A single compiled Schematron rule: an absolute XPath expression
	// (namespace-stripped and document-rooted), plus assertion message and role.
	struct CompiledRule
	{
		std::string xpathExpr;
		std::string message;
		SCHRole role = SCHRole::Warning;
	};

	struct CompiledSchema
	{
		std::vector<CompiledRule> rules;
		bool loaded = false;
	};

}  // namespace hdt
