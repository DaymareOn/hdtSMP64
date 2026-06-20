#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace RE
{
	class NiNode;
}

namespace hdt
{
	// A single physics-XML node reference that does not resolve to any node in the
	// skeleton the XML is applied to. `usedAsBone`/`constraintRefs` record how the XML
	// reaches the node so the report can state the concrete effect of its absence.
	struct MissingBoneRef
	{
		std::string referencedName;  // name exactly as written in the XML (before renameMap)
		std::string resolvedName;    // name after renameMap — what SMP looks up; == referencedName when unmapped
		bool usedAsBone = false;     // referenced by at least one <bone name="…"> definition
		int constraintRefs = 0;      // count of constraint endpoints (bodyA/bodyB) referencing it
	};

	// Returns the physics-XML node references that do NOT resolve to any node in
	// `skeletonRoot` — the NPC node SMP resolves bones against at load time, which already
	// contains the equipped item's merged + renamed nodes.
	//
	// Algorithm: (1) read+parse the XML; (2) collect the skeleton's node-name set via
	// CollectNamedSkeletonNodes; (3) walk every element, gathering each <bone>'s `name`
	// and each generic-/stiffspring-/conetwist-constraint's `bodyA`/`bodyB`; (4) push each
	// reference through `renameMap` (mirroring SkyrimSystemCreator::getRenamedBone) and
	// keep the ones whose resolved name is not in the set, merged per resolved name and
	// sorted by it.
	//
	// A returned entry means SMP would also fail the lookup and therefore silently skip
	// the bone / drop the constraint. The "-default" template element variants are ignored
	// because their `name`/`bodyA`/`bodyB` carry template class names, not node references.
	// Returns empty when the XML is missing or unparsable: reporting bad XML belongs to the
	// schema validator, which runs over the same equipped XMLs. `renameMap` may be empty.
	std::vector<MissingBoneRef> FindMissingPhysicsXmlBoneRefs(
		RE::NiNode* skeletonRoot,
		const std::string& xmlPath,
		const std::unordered_map<std::string, std::string>& renameMap);
}
