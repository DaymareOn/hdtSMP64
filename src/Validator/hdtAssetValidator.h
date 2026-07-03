#pragma once

#include <string>
#include <vector>

namespace hdt
{
	// ── Config ────────────────────────────────────────────────────────────────

	struct ValidationConfig
	{
		std::string modsDir;    // mods folder (MO2 mods/ or Vortex staging) scanned natively, bypassing the VFS
		std::string outputDir;  // derived modsDir/FSMP-out — used by 'smp fix xml'
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

	// ── XML improvement ───────────────────────────────────────────────────────

	struct XMLImproveResult
	{
		int totalXMLsFound = 0;
		int xmlImprovedCount = 0;
		std::vector<std::string> errors;
	};

	// Scan physics XML sources and write improved copies for files where unknown
	// or misplaced elements can be removed.
	// When equippedOnly is true, scans only XML files referenced by currently
	// equipped physics assets.
	// When errorsOnly is true, only files whose source has schema errors (not
	// merely warnings) are rewritten.
	XMLImproveResult ImprovePhysicsXMLs(
		const std::string& outputDir,
		bool equippedOnly = false,
		bool copyOriginal = false,
		bool errorsOnly = false);

}  // namespace hdt
