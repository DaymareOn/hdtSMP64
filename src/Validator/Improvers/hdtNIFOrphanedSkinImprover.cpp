#include "hdtNIFOrphanedSkinImprover.h"

#include "../Schema/hdtNifSchema.h"
#include "hdtNIFBinaryIO.h"

#include <algorithm>
#include <cstring>
#include <optional>
#include <unordered_set>
#include <vector>

namespace hdt
{
	namespace
	{
		/// Build the list of block indices each block points to via Ref fields.
		std::vector<std::vector<int32_t>> buildRefOutbound(const ParsedNif& parsed)
		{
			const NifSchema& schema = globalNifSchema();
			const int32_t numBlocks = static_cast<int32_t>(parsed.blocks.size());
			std::vector<std::vector<int32_t>> out(numBlocks);

			for (int32_t i = 0; i < numBlocks; ++i) {
				auto tOpt = blockTypeOf(parsed, i);
				if (!tOpt)
					continue;
				const auto& block = parsed.blocks[static_cast<size_t>(i)];
				auto offsets = walkBlockRefs(schema, *tOpt, block.data(), block.size(),
					numBlocks, LinkFilter::RefOnly);
				if (!offsets)
					continue;
				for (size_t off : *offsets) {
					if (off + 4 > block.size())
						continue;
					int32_t v = -1;
					std::memcpy(&v, block.data() + off, 4);
					if (v >= 0 && v < numBlocks)
						out[i].push_back(v);
				}
			}
			return out;
		}

		/// Returns the set of NiSkinInstance block indices that have no NiSkinPartition ref.
		std::unordered_set<int32_t> findOrphanedSkinInstances(
			const ParsedNif& parsed,
			const std::vector<std::vector<int32_t>>& outbound)
		{
			const int32_t numBlocks = static_cast<int32_t>(parsed.blocks.size());
			std::unordered_set<int32_t> result;

			for (int32_t i = 0; i < numBlocks; ++i) {
				auto tOpt = blockTypeOf(parsed, i);
				if (!tOpt)
					continue;
				if (*tOpt != "BSTriShape" && *tOpt != "BSDynamicTriShape")
					continue;

				for (int32_t skinRef : outbound[i]) {
					auto skinTypeOpt = blockTypeOf(parsed, skinRef);
					if (!skinTypeOpt)
						continue;
					if (*skinTypeOpt != "NiSkinInstance" && *skinTypeOpt != "BSDismemberSkinInstance")
						continue;

					bool hasPartition = false;
					for (int32_t child : outbound[static_cast<size_t>(skinRef)]) {
						auto childTypeOpt = blockTypeOf(parsed, child);
						if (childTypeOpt && *childTypeOpt == "NiSkinPartition") {
							hasPartition = true;
							break;
						}
					}
					if (!hasPartition)
						result.insert(skinRef);
				}
			}
			return result;
		}

		/// Remap all surviving block refs (nulling refs to removed blocks, adjusting
		/// indices for the gap left by removal), then erase the removed entries.
		/// toRemoveSorted must be sorted ascending and free of duplicates.
		void remapAndRemove(ParsedNif& parsed, const std::vector<int32_t>& toRemoveSorted)
		{
			if (toRemoveSorted.empty())
				return;

			const int32_t totalBefore = static_cast<int32_t>(parsed.blocks.size());
			const std::unordered_set<int32_t> removeSet(toRemoveSorted.begin(), toRemoveSorted.end());

			auto adjustRef = [&](int32_t v) -> int32_t {
				if (v < 0 || v >= totalBefore)
					return v;
				if (removeSet.count(v))
					return -1;
				auto it = std::lower_bound(toRemoveSorted.begin(), toRemoveSorted.end(), v);
				return v - static_cast<int32_t>(it - toRemoveSorted.begin());
			};

			const NifSchema& schema = globalNifSchema();
			for (int32_t i = 0; i < totalBefore; ++i) {
				if (removeSet.count(i))
					continue;
				auto tOpt = blockTypeOf(parsed, i);
				if (!tOpt)
					continue;
				auto& block = parsed.blocks[static_cast<size_t>(i)];
				auto offsets = walkBlockRefs(schema, *tOpt, block.data(), block.size(),
					totalBefore, LinkFilter::All);
				if (!offsets)
					continue;
				for (size_t off : *offsets) {
					if (off + 4 > block.size())
						continue;
					int32_t v = -1;
					std::memcpy(&v, block.data() + off, 4);
					int32_t newV = adjustRef(v);
					if (newV != v)
						std::memcpy(block.data() + off, &newV, 4);
				}
			}

			for (auto& root : parsed.footerRoots)
				root = adjustRef(root);

			for (auto it = toRemoveSorted.rbegin(); it != toRemoveSorted.rend(); ++it) {
				parsed.blocks.erase(parsed.blocks.begin() + *it);
				parsed.blockTypeIndex.erase(parsed.blockTypeIndex.begin() + *it);
			}
		}

	}  // namespace

	int countOrphanedSkinInstances(const ParsedNif& parsed)
	{
		auto outbound = buildRefOutbound(parsed);
		return static_cast<int>(findOrphanedSkinInstances(parsed, outbound).size());
	}

	int removeOrphanedSkinInstances(ParsedNif& parsed)
	{
		auto outbound = buildRefOutbound(parsed);
		auto orphans = findOrphanedSkinInstances(parsed, outbound);
		if (orphans.empty())
			return 0;

		const int skinInstancesRemoved = static_cast<int>(orphans.size());

		// Also remove NiSkinData blocks — owned exclusively by the skin instance.
		std::unordered_set<int32_t> toRemoveSet = orphans;
		for (int32_t skinRef : orphans) {
			for (int32_t child : outbound[static_cast<size_t>(skinRef)]) {
				auto childTypeOpt = blockTypeOf(parsed, child);
				if (childTypeOpt && *childTypeOpt == "NiSkinData")
					toRemoveSet.insert(child);
			}
		}

		std::vector<int32_t> toRemoveSorted(toRemoveSet.begin(), toRemoveSet.end());
		std::sort(toRemoveSorted.begin(), toRemoveSorted.end());
		remapAndRemove(parsed, toRemoveSorted);
		return skinInstancesRemoved;
	}
}
