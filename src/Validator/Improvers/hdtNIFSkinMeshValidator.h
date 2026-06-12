#pragma once

#include <optional>
#include <string>
#include <vector>

namespace hdt
{
	struct ParsedNif;

	struct NifSkinMeshIssue
	{
		int triShapeBlockIndex = -1;
		std::string shapeType;
		std::string reasonCode;  // mirrors decimateCandidateFailClosed reason strings
	};

	/// Runs steps 4–11 from decimateCandidateFailClosed on every skinned mesh candidate
	/// in read-only mode. Returns one issue per failing check per candidate.
	/// Stops at the first fatal inconsistency per candidate (step 6 = count mismatch);
	/// continues across all others so that all issues are surfaced in one pass.
	/// When the binary parser rejects a shape or partition layout, the file is re-read
	/// through nifly (at most once, and only if something was actually rejected): a
	/// shape that nifly reads with sane geometry is accepted silently instead of
	/// warned as unsupported.
	std::vector<NifSkinMeshIssue> detectNIFSkinMeshIssues(const ParsedNif& parsed, const std::string& nifPath);

	/// Per-block query used by the repair pass: returns the byte offsets needed to
	/// overwrite trianglesCopy with triangles in-place, but only when the two arrays
	/// are actually different.  Returns nullopt when the block does not parse as a
	/// supported SSE NiSkinPartition or when the arrays already match.
	struct PartitionMismatchInfo
	{
		size_t trianglesOffset;      // byte offset of the triangles array in the block
		size_t trianglesCopyOffset;  // byte offset of the trianglesCopy array in the block
		uint16_t numTriangles;
	};
	std::optional<PartitionMismatchInfo> checkPartitionTriangleMismatch(
		const std::vector<uint8_t>& block, uint32_t bsVersion);
}
