#pragma once

namespace hdt
{
	struct ParsedNif;

	/// Read-only detection: counts BSTriShape / BSDynamicTriShape blocks whose
	/// NiSkinInstance has no outbound NiSkinPartition ref.  Does not modify the NIF.
	int countOrphanedSkinInstances(const ParsedNif& parsed);

	/// Repair: removes the orphaned NiSkinInstance (and any NiSkinData children) and
	/// nulls the TriShape ref.  Returns the number of NiSkinInstance blocks removed.
	int removeOrphanedSkinInstances(ParsedNif& parsed);
}
