#include "hdtNIFBogusNodeImprover.h"

#include "hdtNIFBinaryIO.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
	// The "remove bogus nodes" algorithm (removing orphan NiNode blocks with
	// no children and no inbound references) is inspired by the spRemoveBogusNodes
	// spell in NifSkope (fo76utils/nifskope), copyright (c) 2005-2014, NIF File
	// Format Library and Tools, under the NifSkope 3-clause BSD license.
	// This is an independent clean-room reimplementation; no NifSkope source
	// code is copied or derived here.

	static bool isImportantNodeName(const std::string& name, uint32_t bsVersion)
	{
		if (name == "ProjectileNode")
			return true;
		if (bsVersion < 83 && (name == "ShellCasingNode" || name == "##SightingNode"))
			return true;
		if (bsVersion >= 130 && name == "WorkshopConnectPoints")
			return true;
		return false;
	}

	bool removeBogusTailNodes(ParsedNif& parsed)
	{
		bool changed = false;

		for (;;) {
			if (parsed.blocks.empty())
				break;

			const int32_t numBlocks = static_cast<int32_t>(parsed.blocks.size());
			const int32_t tailIdx = numBlocks - 1;
			if (tailIdx < 0 || tailIdx >= static_cast<int32_t>(parsed.blockTypeIndex.size()))
				break;

			uint16_t tIdx = parsed.blockTypeIndex[static_cast<size_t>(tailIdx)];
			if (tIdx >= parsed.blockTypes.size())
				break;
			const std::string& typeName = parsed.blockTypes[tIdx];

			bool isSpecialNodeClass = hasTypeName(typeName, { "BSBlastNode", "BSDamageStage", "BSDebrisNode", "BSValueNode" });
			if (typeName != "NiNode" && !isSpecialNodeClass)
				break;

			if (isSpecialNodeClass)
				break;

			std::vector<std::unordered_set<int32_t>> outbound(parsed.blocks.size());
			for (size_t i = 0; i < parsed.blocks.size(); ++i)
				outbound[i] = collectPotentialRefs(parsed.blocks[i], numBlocks);

			std::string nodeName;
			const auto& tailBlock = parsed.blocks.back();
			if (tailBlock.size() >= 4) {
				int32_t nameIdx = -1;
				std::memcpy(&nameIdx, tailBlock.data(), 4);
				if (nameIdx >= 0 && nameIdx < static_cast<int32_t>(parsed.strings.size()))
					nodeName = parsed.strings[static_cast<size_t>(nameIdx)];
			}

			if (isImportantNodeName(nodeName, parsed.bsVersion))
				break;

			if (!outbound.back().empty())
				break;

			bool hasInbound = false;
			for (size_t i = 0; i + 1 < outbound.size(); ++i) {
				if (outbound[i].count(tailIdx)) {
					hasInbound = true;
					break;
				}
			}
			if (hasInbound)
				break;

			parsed.blocks.pop_back();
			parsed.blockTypeIndex.pop_back();
			changed = true;
		}

		return changed;
	}
}