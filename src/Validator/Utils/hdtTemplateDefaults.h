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

	// One top-level <bone> declaration the engine never uses: by the time the parser
	// reaches it, an earlier element in the same file has already claimed the same
	// (case-folded) bone name, so readOrUpdateBone skips it ("Bone X already exists,
	// skipped") — or, when the name resolves to no node, repeats the identical failed
	// lookup. Removing such a declaration cannot change behaviour.
	struct InertBoneInfo
	{
		std::string location;  // positional path, e.g. /system[1]/bone[5]
		std::string boneName;  // value of the bone's name attribute (for the message)
		int line = 0;
	};

	// Find top-level <bone> declarations that are inert — never acted on by the engine,
	// so removable with zero behaviour change — because an earlier same-file element
	// claims the same bone name first: an earlier <bone>, a constraint
	// bodyA/bodyB endpoint, or a can-/no-collide-with-bone shape reference — the engine
	// creates a bone at the first of these it reads, and every later <bone> of that name
	// is skipped. The walk follows document order (recursing into constraint-group) and
	// folds case like BSFixedString; renaming cannot break the equivalence because both
	// occurrences go through the same rename. Only certainty is reported: creators the
	// engine resolves through data outside this file (mesh skinning) are ignored, so the
	// walk under-reports rather than ever flagging a live declaration.
	// When sourceBytes is provided, line numbers are computed from offsets.
	std::vector<InertBoneInfo> CollectInertBoneDeclarations(
		const pugi::xml_document& doc,
		const std::string* sourceBytes = nullptr);

	// The XML-side half of the "skin-redundant <bone>" check (issue #406): top-level <bone>
	// declarations that are removable AS FAR AS THE XML ALONE CAN PROVE — a mesh skinned to the
	// node would make the engine auto-create an identical bone, so the declaration only restates
	// it. Two conditions, both position-aware:
	//   (1) the bone's effective FieldMap equals the unnamed bone-default in force at its own
	//       document position (so what it declares is exactly what would be auto-created), and
	//   (2) no unnamed <bone-default> appears AFTER it — otherwise the default the engine would
	//       recreate the bone with (at the skinning shape's later position) could differ, so
	//       removal would not be neutral.
	// This returns candidates ONLY; it deliberately says nothing about whether any mesh actually
	// skins to the node. That is a per-consumer fact — the caller must confirm EVERY NIF that
	// references this XML is skinned to `boneName` (after renaming) before reporting removability,
	// which is what keeps the warning a mod-level truth rather than a per-item accident.
	// When sourceBytes is provided, line numbers are computed from offsets.
	std::vector<InertBoneInfo> CollectSkinRedundantBoneCandidates(
		const pugi::xml_document& doc,
		const std::string* sourceBytes = nullptr);

	// The ecosystem half of the #406 check: given the XML-intrinsic candidates for one physics
	// file and, for each NIF that references that file, the set of node names that NIF is skinned
	// to (case-folded, e.g. from ExtractSkinBoundBoneNames), keep only the candidates that are
	// removable community-wide — those whose bone name (case-folded) is skinned by EVERY consumer.
	// The reasoning: a mesh skinned to node X makes the engine auto-create an identical default
	// bone X, so <bone X> is pure redundancy there; but if even one consumer does NOT skin to X,
	// its <bone X> may be that bone's only creator, so removal would change behaviour. Requiring
	// ALL consumers is what makes the warning a mod-level truth. With zero consumers nothing is
	// proven and the result is empty. A consumer whose skin bones could not be read contributes an
	// empty set, which drops every candidate — the check under-reports rather than over-reports.
	std::vector<InertBoneInfo> FilterSkinRedundantBonesByConsumers(
		const std::vector<InertBoneInfo>& candidates,
		const std::vector<std::vector<std::string>>& consumerSkinBones);

}  // namespace hdt
