#include "hdtNIFSkinMeshRepair.h"

#include "hdtNIFSkinMeshValidator.h"
#include "hdtNIFBinaryIO.h"
#include "../Utils/hdtNIFBinaryUtils.h"

#include <cstring>
#include <optional>

namespace hdt
{
	namespace
	{
		std::optional<std::string> typeAt(const ParsedNif& parsed, int32_t idx)
		{
			if (idx < 0 || idx >= static_cast<int32_t>(parsed.blockTypeIndex.size()))
				return std::nullopt;
			uint16_t tIdx = parsed.blockTypeIndex[static_cast<size_t>(idx)];
			if (tIdx >= parsed.blockTypes.size()) return std::nullopt;
			return parsed.blockTypes[tIdx];
		}

	}  // namespace

	int repairNIFSkinMeshIssues(ParsedNif& parsed)
	{
		int fixed = 0;

		for (int32_t i = 0; i < static_cast<int32_t>(parsed.blocks.size()); ++i) {
			auto tOpt = typeAt(parsed, i);
			if (!tOpt || *tOpt != nif::kTypeNiSkinPartition) continue;

			auto& block = parsed.blocks[static_cast<size_t>(i)];
			auto  infoOpt = checkPartitionTriangleMismatch(block, parsed.bsVersion);
			if (!infoOpt) continue;

			const size_t byteCount = static_cast<size_t>(infoOpt->numTriangles) * nif::kTriangleByteSize;

			// Both regions must fit in the block. The trailing-bytes check in the tolerant
			// parser guarantees trianglesCopyOffset + byteCount == block.size(), and
			// trianglesOffset + byteCount <= block.size() by virtue of a successful parse.
			if (infoOpt->trianglesCopyOffset + byteCount != block.size()) continue;
			if (infoOpt->trianglesOffset     + byteCount  > block.size()) continue;

			std::memcpy(
				block.data() + infoOpt->trianglesCopyOffset,
				block.data() + infoOpt->trianglesOffset,
				byteCount);
			++fixed;
		}

		return fixed;
	}
}
