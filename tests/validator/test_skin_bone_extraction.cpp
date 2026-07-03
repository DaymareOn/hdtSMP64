// ExtractSkinBoundBoneNames tests (issue #406, offline-NIF half).
//
// Given a parsed NIF, ExtractSkinBoundBoneNames returns the case-folded, de-duplicated set of
// skeleton-node names every skin instance is skinned to (the union of the Bones arrays). These
// tests hand-build a ParsedNif so they exercise the exact offset math and the schema-driven
// Bones-vs-scalar-ref selection without needing a real .nif on disk. The zero-false-positive
// contract is what they guard: only true bone bindings are reported, and only from node blocks.

#include "Validator/Parser/hdtNIFBinaryParser.h"

#include <doctest/doctest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace
{
	void putI32(std::vector<uint8_t>& b, int32_t v)
	{
		uint8_t tmp[4];
		std::memcpy(tmp, &v, 4);
		b.insert(b.end(), tmp, tmp + 4);
	}

	// A skin-instance block laid out exactly as the SSE schema expects, so walkBlockRefDetails
	// parses it: three scalar refs (Data, Skin Partition, Skeleton Root), Num Bones, then the
	// Bones ref array. `skeletonRoot` is exposed so a test can point it at a node and prove that
	// scalar ref is NOT mistaken for a bone.
	std::vector<uint8_t> skinInstanceBlock(const std::vector<int32_t>& boneRefs, int32_t skeletonRoot = -1)
	{
		std::vector<uint8_t> b;
		putI32(b, -1);                                     // Data
		putI32(b, -1);                                     // Skin Partition
		putI32(b, skeletonRoot);                           // Skeleton Root
		putI32(b, static_cast<int32_t>(boneRefs.size()));  // Num Bones
		for (int32_t r : boneRefs)
			putI32(b, r);
		return b;
	}

	// A scene node only needs its Name (a StringIndex at block offset 0) for the extractor.
	std::vector<uint8_t> nodeBlock(uint32_t nameIdx)
	{
		std::vector<uint8_t> b;
		putI32(b, static_cast<int32_t>(nameIdx));
		return b;
	}

	bool has(const std::vector<std::string>& v, const std::string& s)
	{
		return std::find(v.begin(), v.end(), s) != v.end();
	}
}

TEST_CASE("bones of a NiSkinInstance are returned, case-folded")
{
	hdt::ParsedNif p;
	p.blockTypes = { "NiSkinInstance", "NiNode" };
	p.blockTypeIndex = { 0, 1, 1 };
	p.strings = { "NPC Spine", "NPC Head" };
	p.blocks = { skinInstanceBlock({ 1, 2 }), nodeBlock(0), nodeBlock(1) };

	auto names = hdt::nif::ExtractSkinBoundBoneNames(p);
	CHECK(names.size() == 2);
	CHECK(has(names, "npc spine"));
	CHECK(has(names, "npc head"));
}

TEST_CASE("BSDismemberSkinInstance bones are read the same way")
{
	hdt::ParsedNif p;
	p.blockTypes = { "BSDismemberSkinInstance", "NiNode" };
	p.blockTypeIndex = { 0, 1 };
	p.strings = { "NPC L Breast01" };
	// BSDismemberSkinInstance adds Num Partitions + Partitions AFTER Bones; the extractor reads
	// the Bones array from the NiSkinInstance base, so those trailing uints are irrelevant here.
	p.blocks = { skinInstanceBlock({ 1 }), nodeBlock(0) };

	auto names = hdt::nif::ExtractSkinBoundBoneNames(p);
	REQUIRE(names.size() == 1);
	CHECK(names[0] == "npc l breast01");
}

TEST_CASE("the scalar Skeleton Root ref is not treated as a bone")
{
	hdt::ParsedNif p;
	p.blockTypes = { "NiSkinInstance", "NiNode" };
	p.blockTypeIndex = { 0, 1, 1 };
	p.strings = { "NPC Spine", "NPC Root" };
	// Skeleton Root -> block 2 ("NPC Root"), Bones -> [block 1 ("NPC Spine")].
	p.blocks = { skinInstanceBlock({ 1 }, /*skeletonRoot=*/2), nodeBlock(0), nodeBlock(1) };

	auto names = hdt::nif::ExtractSkinBoundBoneNames(p);
	REQUIRE(names.size() == 1);
	CHECK(has(names, "npc spine"));
	CHECK(!has(names, "npc root"));  // scalar ref, not a bone
}

TEST_CASE("a bone ref to a non-node block is skipped")
{
	hdt::ParsedNif p;
	p.blockTypes = { "NiSkinInstance", "NiNode", "BSTriShape" };
	p.blockTypeIndex = { 0, 1, 2 };
	p.strings = { "NPC Spine", "SomeMesh" };
	// Bones -> [block 1 (NiNode "NPC Spine"), block 2 (BSTriShape — not a scene node)].
	p.blocks = { skinInstanceBlock({ 1, 2 }), nodeBlock(0), nodeBlock(1) };

	auto names = hdt::nif::ExtractSkinBoundBoneNames(p);
	REQUIRE(names.size() == 1);
	CHECK(has(names, "npc spine"));
	CHECK(!has(names, "somemesh"));  // BSTriShape does not inherit NiNode
}

TEST_CASE("an out-of-range bone ref is skipped without crashing")
{
	hdt::ParsedNif p;
	p.blockTypes = { "NiSkinInstance", "NiNode" };
	p.blockTypeIndex = { 0, 1 };
	p.strings = { "NPC Spine" };
	// Bones -> [block 1 (valid), block 99 (out of range)].
	p.blocks = { skinInstanceBlock({ 1, 99 }), nodeBlock(0) };

	auto names = hdt::nif::ExtractSkinBoundBoneNames(p);
	REQUIRE(names.size() == 1);
	CHECK(names[0] == "npc spine");
}

TEST_CASE("an empty bone name is ignored")
{
	hdt::ParsedNif p;
	p.blockTypes = { "NiSkinInstance", "NiNode" };
	p.blockTypeIndex = { 0, 1, 1 };
	p.strings = { "", "NPC Spine" };
	p.blocks = { skinInstanceBlock({ 1, 2 }), nodeBlock(0), nodeBlock(1) };

	auto names = hdt::nif::ExtractSkinBoundBoneNames(p);
	REQUIRE(names.size() == 1);
	CHECK(names[0] == "npc spine");
}

TEST_CASE("bones are unioned across skin instances and de-duplicated")
{
	hdt::ParsedNif p;
	p.blockTypes = { "NiSkinInstance", "NiNode" };
	// Two skin instances (blocks 0 and 1), three nodes (blocks 2,3,4).
	p.blockTypeIndex = { 0, 0, 1, 1, 1 };
	p.strings = { "NPC Spine", "NPC Head" };
	p.blocks = {
		skinInstanceBlock({ 2, 3 }),  // Spine, Head
		skinInstanceBlock({ 2, 4 }),  // Spine (dup), Head-again
		nodeBlock(0),                 // "NPC Spine"
		nodeBlock(1),                 // "NPC Head"
		nodeBlock(1),                 // "NPC Head" (same name, different node)
	};

	auto names = hdt::nif::ExtractSkinBoundBoneNames(p);
	CHECK(names.size() == 2);
	CHECK(has(names, "npc spine"));
	CHECK(has(names, "npc head"));
}

TEST_CASE("a NIF with no skin instance yields no bones")
{
	hdt::ParsedNif p;
	p.blockTypes = { "NiNode" };
	p.blockTypeIndex = { 0 };
	p.strings = { "NPC Spine" };
	p.blocks = { nodeBlock(0) };

	CHECK(hdt::nif::ExtractSkinBoundBoneNames(p).empty());
}
