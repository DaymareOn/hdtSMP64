#pragma once

#include <string>
#include <vector>

namespace hdt
{
	// ── Config ────────────────────────────────────────────────────────────────

	struct ValidationConfig
	{
		int warnTriangleCount = 10000;
		std::string modsDir;    // mods folder (MO2 mods/ or Vortex staging); FSMP-out is created inside it
		std::string outputDir;  // derived: modsDir/FSMP-out — set by config loader, used everywhere
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
	XMLImproveResult ImprovePhysicsXMLs(
		const std::string& outputDir,
		bool equippedOnly = false,
		bool copyOriginal = false,
		bool stateless = false);

	// ── NIF improvement ───────────────────────────────────────────────────────

	struct NIFImproveResult
	{
		int totalNIFsFound = 0;
		int totalTRIFilesFound = 0;
		int nifImprovedCount = 0;
		int orphanedSkinInstancesRemoved = 0;
		int skinMeshIssuesFixed = 0;
		std::vector<std::string> errors;
	};

	// Scan NIFs and write structurally repaired copies (bogus node removal, orphaned skin
	// instances, partition triangle-copy mismatches). Never runs decimation.
	// When equippedOnly is true, scans only NIFs associated with currently equipped
	// physics assets.
	NIFImproveResult ImprovePhysicsNIFs(
		const std::string& outputDir,
		bool equippedOnly = false,
		bool copyOriginal = false);

	struct NIFTrimResult
	{
		int totalNIFsFound = 0;
		int totalTRIFilesFound = 0;
		int nifTrimmedCount = 0;
		int decimationCandidatesDiscovered = 0;
		int decimationCandidatesAttempted = 0;
		int decimationCandidatesApplied = 0;
		int decimationCandidatesSkippedNoChange = 0;
		int decimationCandidatesSkippedUnsafe = 0;
		std::vector<std::string> decimationSkipReasonHistogram;
		std::vector<std::string> errors;
	};

	/// Reduce collision mesh polygon counts for all physics NIFs and write the results.
	/// Runs decimation only — no structural repairs. Use FixTrimPhysicsNIFs for both.
	/// When equippedOnly is true, scans only NIFs associated with currently equipped
	/// physics assets.
	NIFTrimResult TrimPhysicsNIFs(
		const std::string& outputDir,
		bool equippedOnly = false,
		bool copyOriginal = false);

	struct NIFFixTrimResult
	{
		int totalNIFsFound = 0;
		int totalTRIFilesFound = 0;
		int nifProcessedCount = 0;
		int orphanedSkinInstancesRemoved = 0;
		int skinMeshIssuesFixed = 0;
		int decimationCandidatesDiscovered = 0;
		int decimationCandidatesAttempted = 0;
		int decimationCandidatesApplied = 0;
		int decimationCandidatesSkippedNoChange = 0;
		int decimationCandidatesSkippedUnsafe = 0;
		std::vector<std::string> decimationSkipReasonHistogram;
		std::vector<std::string> errors;
	};

	/// Structural repairs (bogus nodes, orphaned skins, mesh issues, missing XML refs)
	/// followed by collision mesh decimation in a single pass.
	/// When equippedOnly is true, scans only NIFs associated with currently equipped
	/// physics assets.
	NIFFixTrimResult FixTrimPhysicsNIFs(
		const std::string& outputDir,
		bool equippedOnly = false,
		bool copyOriginal = false);

}  // namespace hdt
