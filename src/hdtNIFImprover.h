#pragma once

#include <string>
#include <utility>
#include <vector>

namespace hdt
{
		struct NIFImproverDiagnostics
		{
			int decimationCandidatesDiscovered = 0;
			int decimationCandidatesAttempted = 0;
			int decimationCandidatesApplied = 0;
			int decimationCandidatesSkippedNoChange = 0;
			int decimationCandidatesSkippedUnsafe = 0;
			std::vector<std::pair<std::string, int>> decimationSkipReasons;
		};

	struct NIFDecimationOptions
	{
		bool enableCollisionMeshDecimation = false;
		float targetVertexRatio = 0.99f;  // fraction of vertices to retain (0.99 => ~1% reduction)
		int targetVertexCount = 0;
		float qemCostThreshold = 1.0f;
		float shortEdgeRatio = 0.01f;
		float skinWeightPenalty = 0.0f;
		float maxSkinWeightDrift = 0.0f;
		float maxVolumeLossPercent = 1.0f;
		float maxLocalVolumeChangePercent = 1.0f;
		float maxNormalDeviationDegrees = 25.0f;
		int maxPointRemovals = 256;   // legacy/no-op: retained for config compatibility
		int maxEdgeCollapses = 4096;  // legacy/no-op: retained for config compatibility
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
			const NIFDecimationOptions& options = {},
			NIFImproverDiagnostics* outDiagnostics = nullptr);

	// Copies a NIF file unchanged to:
	//   <outputDir>/<path-relative-to-data/>
	// Used to keep _0/_1 NIF pairs consistent in the output directory when
	// one member of the pair was improved but the other needed no changes.
	//
	// Returns true  — the file was successfully copied.
	// Returns false — I/O error.
	bool CopyNIFToOutput(
		const std::string& srcNIFPath,
		const std::string& outputDir);
}
