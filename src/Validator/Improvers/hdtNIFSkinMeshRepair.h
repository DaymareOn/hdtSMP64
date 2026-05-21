#pragma once

namespace hdt
{
	struct ParsedNif;

	/// Repairs NiSkinPartition blocks where the triangles array and its redundant copy
	/// have diverged.  For each such block, overwrites the copy in-place with the primary
	/// array.  Returns the number of partition blocks repaired.
	int repairNIFSkinMeshIssues(ParsedNif& parsed);
}
