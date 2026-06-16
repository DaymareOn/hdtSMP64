#pragma once

#include <string>
#include <vector>

namespace hdt
{
	// ── Config ────────────────────────────────────────────────────────────────

	struct ValidationConfig
	{
		std::string modsDir;  // mods folder (MO2 mods/ or Vortex staging) scanned natively, bypassing the VFS
	};

	extern ValidationConfig g_validationConfig;

	// ── Shared asset type ─────────────────────────────────────────────────────

	struct PhysicsAsset
	{
		std::string nifPath;
		std::string xmlPath;
		std::vector<std::string> relatedTRIPaths;
		std::vector<std::string> allPhysicsXmlPaths;  // all "HDT Skinned Mesh Physics Object" blocks
		bool nifExists = false;
		bool xmlExists = false;
		bool hasOrphanedPhysicsMarker = false;  // marker string present but no NiStringExtraData block
	};

	// ── Validation ────────────────────────────────────────────────────────────

	enum class ValidationReportMode
	{
		Full,
		ErrorsOnly
	};

	struct AssetValidationResult
	{
		bool hasErrors = false;
		bool hasWarnings = false;
		int skinMeshIssuesFound = 0;
		int filesystemNifFilesDiscovered = 0;
		int equippedNifsDiscovered = 0;
		int nifScanViolationCount = 0;
		int totalNIFsScanned = 0;
		int totalXMLsFound = 0;
		int xmlPassCount = 0;
		int xmlErrorCount = 0;
		int xmlWarningCount = 0;
		double elapsedSeconds = 0.0;
		std::vector<std::string> errors;
		std::vector<std::string> warnings;
		std::vector<PhysicsAsset> assets;
	};

	// Run validation from the console command path.
	// When equippedOnly is true, validates only currently equipped items on tracked
	// skeletons (PC and instantiated NPCs).
	// Always writes the report file regardless of config.
	// Populates outReportPath with the absolute path to the written report (empty on failure).
	// Returns the validation result for the selected scope.
	AssetValidationResult ValidatePhysicsAssets(
		std::string& outReportPath,
		bool equippedOnly = false,
		ValidationReportMode reportMode = ValidationReportMode::Full);

}  // namespace hdt
