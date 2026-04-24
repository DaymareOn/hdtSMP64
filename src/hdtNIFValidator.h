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
		bool hasGeometry = false;
		bool hasSkinning = false;
		uint32_t boneCount = 0;
		std::vector<std::string> errors;
	};

	// Scan a NIF binary file on disk for "HDT Skinned Mesh Physics Object" extra data.
	// Parses the NIF string table and block list to locate NiStringExtraData entries.
	// Returns the XML physics path referenced by the NIF, if found.
	NIFScanResult ScanNIFBinary(const std::string& nifPath);

	struct NIFStructuralResult
	{
		bool isValid = true;
		bool hasSkinningData = false;
		uint32_t boneCount = 0;
		uint32_t triangleCount = 0;
		std::vector<std::string> boneNames;
		std::vector<std::string> errors;
		std::vector<std::string> warnings;
	};

	// Validate NIF structural requirements for FSMP physics using a loaded NiNode*.
	// Can be called at runtime when the NIF has been loaded by the game.
	NIFStructuralResult ValidateNIFStructure(RE::NiNode* root, const std::string& nifPath);

}  // namespace hdt
