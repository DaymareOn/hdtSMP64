#pragma once

#include <string>
#include <vector>

namespace hdt
{
	struct ValidationConfig
	{
		bool enabled = true;
		bool strictMode = false;
		bool reportFileEnabled = true;
		bool scanDataFolder = false;  // NIF scanning at plugin-load time is unsafe (BSAs not yet mounted)
		int warnTriangleCount = 10000;
	};

	extern ValidationConfig g_validationConfig;

	struct PhysicsAsset
	{
		std::string nifPath;
		std::string xmlPath;
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
		std::vector<std::string> errors;
		std::vector<std::string> warnings;
		std::vector<PhysicsAsset> assets;
	};

	// Run the full validation pipeline at startup.
	// Returns true if no errors were found (warnings are non-blocking).
	bool ValidateAllPhysicsAssets();

}  // namespace hdt
