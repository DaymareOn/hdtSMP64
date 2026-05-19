#pragma once

namespace hdt
{
	struct ParsedNif;

	// Removes orphan NiNode blocks (no children, no inbound references) from anywhere
	// in the block list, iterating until no more can be removed. Updates all block
	// references in the remaining blocks after each pass.
	// Returns true if any blocks were removed.
	bool removeBogusNiNodes(ParsedNif& parsed);
}
