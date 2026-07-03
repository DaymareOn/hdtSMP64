#pragma once

#include "../Improvers/hdtNIFBinaryIO.h"

#include <string>
#include <vector>

namespace hdt
{
	namespace nif
	{
		// Extracts all XML string references from NiStringExtraData blocks named with the
		// physics marker ("HDT Skinned Mesh Physics Object") in a fully-parsed NIF.
		// Reads directly from parsed.blocks — no raw-byte offset arithmetic needed.
		std::vector<std::string> FindXmlPathsInNif(const ParsedNif& parsed);

		// Returns every skeleton-node name this NIF is skinned to: the union of the Bones
		// arrays of all NiSkinInstance / BSDismemberSkinInstance / BSSkin::Instance blocks.
		// Names are case-folded (ASCII lower) and de-duplicated so callers can compare and
		// intersect them directly.
		//
		// How it reads a skin instance's bones: the schema marks the Bones field as the block's
		// only array-valued Ref (Data / Skin Partition / Skeleton Root are scalar Refs), so we
		// take every array-element Ref, follow it to its block, and read that block's Name — the
		// first field (a StringIndex at byte offset 0) of every NiObjectNET-derived node. A bone
		// Ref that points to a non-node block, an unknown type, or an out-of-range string is
		// skipped, so the result UNDER-reports (never invents) a skin binding — which is what lets
		// the #406 caller treat "every consumer skins to X" as a zero-false-positive fact.
		std::vector<std::string> ExtractSkinBoundBoneNames(const ParsedNif& parsed);

	}  // namespace nif

}  // namespace hdt
