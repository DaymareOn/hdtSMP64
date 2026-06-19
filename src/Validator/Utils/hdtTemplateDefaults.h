#pragma once

#include <pugixml.hpp>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdt
{
	// Returns true if the tag is a default node (bone-default, generic-constraint-default, etc.)
	bool isDefaultNodeName(const std::string& localName);
	struct TemplateRedundantChildInfo
	{
		std::string location;
		std::string tagName;
		int line = 0;
		bool shadowedByLaterFrameTag = false;
		std::string shadowingTagName;
	};

	// Analyze one physics XML document and return detailed redundant child info.
	// When sourceBytes is provided, line numbers are computed from node offsets.
	std::vector<TemplateRedundantChildInfo> CollectTemplateRedundantChildrenInfo(
		const pugi::xml_document& doc,
		const std::string* sourceBytes = nullptr);

	// Analyze one physics XML document using runtime-like template semantics and
	// return locations of child tags that are redundant relative to effective defaults.
	std::unordered_set<std::string> CollectTemplateRedundantChildLocations(const pugi::xml_document& doc);

	// Remove child tags that are redundant relative to effective defaults.
	// Returns true when at least one child element was removed.
	bool RemoveTemplateRedundantChildren(pugi::xml_document& doc);

	// Remove top-level *-default nodes that are never referenced by any later
	// template/template-inheritance use in the same file.
	bool RemoveUnusedDefaultNodes(pugi::xml_document& doc);

	// Returns duplicate named default templates mapped to an earlier equivalent
	// named template using runtime-like effective template semantics.
	std::unordered_map<std::string, std::string> CollectEquivalentDefaultTemplateAliases(
		const pugi::xml_document& doc);

	// One top-level <bone> declaration found to be redundant: its complete effective
	// settings match the bone the engine would auto-create for an undeclared node.
	struct RedundantBoneInfo
	{
		std::string location;  // positional path, e.g. /system[1]/bone[5]
		std::string boneName;  // value of the bone's name attribute (for the message)
		int line = 0;
	};

	// Find top-level <bone> declarations that only restate the auto-created default
	// bone (the unnamed bone-default) and are therefore removable with no behavioural
	// change. Reuses the effective-default machinery; see the .cpp for the conservative
	// comparison. When sourceBytes is provided, line numbers are computed from offsets.
	std::vector<RedundantBoneInfo> CollectRedundantBoneDeclarations(
		const pugi::xml_document& doc,
		const std::string* sourceBytes = nullptr);

}  // namespace hdt
