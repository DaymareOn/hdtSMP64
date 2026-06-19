#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace RE
{
	class NiNode;
}

namespace hdt
{
	struct NIFScanResult
	{
		bool hasPhysicsData = false;
		std::string physicsXmlPath;
		std::vector<std::string> allPhysicsXmlPaths;  // all found, for duplicate detection
		bool hasGeometry = false;
		bool hasSkinning = false;
		bool hasOrphanedPhysicsMarker = false;  // marker string present but no NiStringExtraData block references it
		std::vector<std::string> errors;
	};

	// Extract physics XML references from a NIF binary file on disk.
	// Parses NIF header structures to locate "HDT Skinned Mesh Physics Object"
	// NiStringExtraData links and records scanner/parsing failures in NIFScanResult::errors.
	NIFScanResult ExtractPhysicsXmlRefsFromNIFs(const std::string& nifPath);

	struct NIFStructuralResult
	{
		bool isValid = true;
		bool hasSkinningData = false;
		uint32_t boneCount = 0;
		std::vector<std::string> boneNames;
		std::vector<std::string> errors;
		std::vector<std::string> warnings;
	};

	// Validate NIF structural requirements for FSMP physics using a loaded NiNode*.
	// Can be called at runtime when the NIF has been loaded by the game.
	NIFStructuralResult ValidateNIFStructure(RE::NiNode* root, const std::string& nifPath);

	// Recursively appends every non-empty NiNode name in the subtree rooted at `root`
	// to `outNames`. Used both as a diagnostic bone inventory and as the lookup set the
	// bone-reference validator resolves physics-XML node references against.
	void CollectNamedSkeletonNodes(RE::NiNode* root, std::vector<std::string>& outNames);

}  // namespace hdt
