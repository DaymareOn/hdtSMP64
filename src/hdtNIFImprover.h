#pragma once

#include <string>

namespace hdt
{
	struct NIFDecimationOptions
	{
		bool enableCollisionMeshDecimation = false;
		float targetVertexRatio = 0.99f;
		int targetVertexCount = 0;
		float qemCostThreshold = 1.0f;
		float shortEdgeRatio = 0.01f;
		float maxVolumeLossPercent = 1.0f;
		float maxLocalVolumeChangePercent = 1.0f;
		float maxNormalDeviationDegrees = 25.0f;
		int maxPointRemovals = 256;
		int maxEdgeCollapses = 4096;
		bool preserveBoundary = true;
		bool preserveFeatures = true;
	};

	// Removes obviously bogus NiNode blocks from NIF binaries and writes improved
	// files to:
	//   <outputDir>/<path-relative-to-data/>
	// only when at least one block was removed.
	//
	// Returns true  — a cleaned file was written.
	// Returns false — no changes, unsupported format, or I/O error.
	bool GenerateImprovedNIF(
		const std::string& srcNIFPath,
		const std::string& outputDir,
		const NIFDecimationOptions& options = {});
}
