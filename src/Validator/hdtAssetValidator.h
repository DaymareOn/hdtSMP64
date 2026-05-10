#pragma once

#include <string>
#include <vector>

namespace hdt
{
	struct ValidationConfig
	{
		bool enabled = true;
		int warnTriangleCount = 10000;
		std::string outputDir;  // if set, improved XML copies are written here
		bool improveNIFs = false;
		bool decimateCollisionMeshesOffline = false;
		float decimationTargetVertexRatio = 0.99f;
		int decimationTargetVertexCount = 0;
		float decimationQemCostThreshold = 1.0f;  // scale-invariant: threshold is multiplied by diag² before comparison with QEM cost (0 = disabled)
		float decimationShortEdgeRatio = 0.01f;
		float decimationSkinWeightPenalty = 0.0f;
		float decimationMaxSkinWeightDrift = 0.0f;
		float decimationMaxVolumeLossPercent = 1.0f;
		float decimationMaxLocalVolumeChangePercent = 1.0f;
		float decimationMaxNormalDeviationDegrees = 25.0f;
		int decimationMaxPointRemovals = 256;   // legacy/no-op: retained for config compatibility
		int decimationMaxEdgeCollapses = 4096;  // legacy/no-op: retained for config compatibility
		bool decimationPreserveBoundary = true;
		bool decimationPreserveFeatures = true;
		bool parallelNIFImprovement = true;
	};

	extern ValidationConfig g_validationConfig;

	struct PhysicsAsset
	{
		std::string nifPath;
		std::string xmlPath;
		std::vector<std::string> relatedTRIPaths;
		std::vector<std::string> allPhysicsXmlPaths;  // all "HDT Skinned Mesh Physics Object" blocks
		bool nifExists = false;
		bool xmlExists = false;
	};

	struct AssetValidationResult
	{
		bool hasErrors = false;
		bool hasWarnings = false;
		int filesystemNifFilesDiscovered = 0;
		int equippedNifsDiscovered = 0;
		int nifScanViolationCount = 0;
		int totalNIFsScanned = 0;
		int totalXMLsFound = 0;
		int xmlPassCount = 0;
		int xmlErrorCount = 0;
		int xmlWarningCount = 0;
		int xmlImprovedCount = 0;  // number of improved XML files written
		int nifImprovedCount = 0;  // number of improved NIF files written
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
	AssetValidationResult ValidatePhysicsAssets(std::string& outReportPath, bool equippedOnly = false);

	struct NIFImproveResult
	{
		int totalNIFsFound = 0;
		int totalTRIFilesFound = 0;
		int nifImprovedCount = 0;
		int decimationCandidatesDiscovered = 0;
		int decimationCandidatesAttempted = 0;
		int decimationCandidatesApplied = 0;
		int decimationCandidatesSkippedNoChange = 0;
		int decimationCandidatesSkippedUnsafe = 0;
		std::vector<std::string> decimationSkipReasonHistogram;
		std::vector<std::string> errors;
	};

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
	XMLImproveResult ImprovePhysicsXMLs(const std::string& outputDir, bool equippedOnly = false);

	// Scan NIFs and write improved copies for files where bogus nodes can be removed.
	// When equippedOnly is true, scans only NIFs associated with currently equipped
	// physics assets.
	NIFImproveResult ImprovePhysicsNIFs(const std::string& outputDir, bool equippedOnly = false);

}  // namespace hdt
