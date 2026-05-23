#pragma once

#include <vector>

namespace hdt
{
	struct ParsedNif;

	// Removes orphan NiNode blocks (no children, no inbound references) from anywhere
	// in the block list, iterating until no more can be removed. Updates all block
	// references in the remaining blocks after each pass.
	// Returns true if any blocks were removed.
	bool removeBogusNiNodes(ParsedNif& parsed);

	// Remove the given blocks (indices must be sorted ascending, no duplicates) and
	// update all surviving block references. Array-element refs to removed blocks are
	// excised from their arrays with the element count decremented.
	void removeBlocksAndRemap(ParsedNif& parsed, const std::vector<int32_t>& toRemoveSorted);

	void resetBogusNodeTimings(); // call before the parallel loop
	void logBogusNodeTimings();   // call after
}
