#pragma once

#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
	struct ParsedNif;

	// Removes orphan NiNode blocks (no children, no inbound references) from anywhere
	// in the block list, iterating until no more can be removed. Updates all block
	// references in the remaining blocks after each pass.
	// Nodes whose names appear in xmlProtectedNodeNames are never removed.
	// Returns true if any blocks were removed.
	bool removeBogusNiNodes(ParsedNif& parsed,
	                        const std::unordered_set<std::string>* xmlProtectedNodeNames = nullptr);

	// Remove the given blocks (indices must be sorted ascending, no duplicates) and
	// update all surviving block references. Array-element refs to removed blocks are
	// excised from their arrays with the element count decremented.
	void removeBlocksAndRemap(ParsedNif& parsed, const std::vector<int32_t>& toRemoveSorted);

	void resetBogusNodeTimings(); // call before the parallel loop
	void logBogusNodeTimings();   // call after
}
