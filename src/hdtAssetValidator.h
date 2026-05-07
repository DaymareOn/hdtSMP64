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
	};

	extern ValidationConfig g_validationConfig;

	struct PhysicsAsset
	{
		std::string nifPath;
		std::string xmlPath;
		std::vector<std::string> allPhysicsXmlPaths;  // all "HDT Skinned Mesh Physics Object" blocks
		bool nifExists = false;
		bool xmlExists = false;
	};

	struct AssetValidationResult
	{
		bool hasErrors = false;
		bool hasWarnings = false;
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

	// Run the full validation pipeline at startup.
	// Returns true if no errors were found (warnings are non-blocking).
	bool ValidateAllPhysicsAssets();

	// Run the full validation pipeline on demand (e.g. from the console command).
	// Always writes the report file regardless of config.
	// Populates outReportPath with the absolute path to the written report (empty on failure).
	// Returns the full validation result.
	AssetValidationResult ValidateAllPhysicsAssetsOnDemand(std::string& outReportPath);

	// Run validation only for currently equipped items on tracked skeletons (PC and instantiated NPCs).
	// Always writes the report file regardless of config.
	// Populates outReportPath with the absolute path to the written report (empty on failure).
	// Returns the validation result for equipped gear only.
	AssetValidationResult ValidateEquippedPhysicsAssetsOnDemand(std::string& outReportPath);

	struct NIFImproveResult
	{
		int totalNIFsFound = 0;
		int nifImprovedCount = 0;
		std::vector<std::string> errors;
	};

	struct XMLImproveResult
	{
		int totalXMLsFound = 0;
		int xmlImprovedCount = 0;
		std::vector<std::string> errors;
	};

	// Scan all discovered physics XML sources and write improved copies for files
	// where unknown or misplaced elements can be removed.
	XMLImproveResult ImprovePhysicsXMLsOnDemand(const std::string& outputDir);

	// Scan only currently equipped physics assets and write improved XML copies
	// for their referenced XML files.
	XMLImproveResult ImproveEquippedPhysicsXMLsOnDemand(const std::string& outputDir);

	// Scan data/meshes for physics-enabled NIFs and write improved copies for
	// files where bogus nodes can be removed.
	NIFImproveResult ImprovePhysicsNIFsOnDemand(const std::string& outputDir);

}  // namespace hdt
