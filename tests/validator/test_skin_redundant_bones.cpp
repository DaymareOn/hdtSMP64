// CollectSkinRedundantBoneCandidates tests (issue #406, XML-intrinsic half).
//
// The candidate collector returns top-level <bone> declarations that are removable AS FAR AS
// THE XML PROVES: effective FieldMap == the unnamed default at the bone's position, and no
// unnamed <bone-default> appears after it. Whether a mesh actually skins to the node is a
// per-consumer fact applied elsewhere; these tests pin only the XML-side conditions.

#include "Validator/Utils/hdtTemplateDefaults.h"

#include <doctest/doctest.h>
#include <pugixml.hpp>

#include <string>
#include <vector>

namespace
{
	std::vector<hdt::InertBoneInfo> candidates(const std::string& xml, bool withSource = false)
	{
		pugi::xml_document doc;
		if (!doc.load_buffer(xml.data(), xml.size()))
			return {};
		return hdt::CollectSkinRedundantBoneCandidates(doc, withSource ? &xml : nullptr);
	}

	bool hasBone(const std::vector<hdt::InertBoneInfo>& v, const std::string& name)
	{
		for (const auto& b : v)
			if (b.boneName == name)
				return true;
		return false;
	}
}

TEST_CASE("an empty <bone> equals the default and is a candidate")
{
	auto c = candidates("<system><bone name=\"A\"/></system>");
	REQUIRE(c.size() == 1);
	CHECK(c[0].boneName == "A");
	CHECK(c[0].location == "/system[1]/bone[1]");
}

TEST_CASE("a <bone> carrying a non-default value is not a candidate")
{
	// A non-default field (or any unmodelled child) makes the effective map differ.
	CHECK(candidates("<system><bone name=\"A\"><mass>99</mass></bone></system>").empty());
	CHECK(candidates("<system><bone name=\"A\"><some-tag/></bone></system>").empty());
}

TEST_CASE("an unnamed <bone-default> after a candidate clears it (position guard)")
{
	// A: before the unnamed bone-default -> the default the engine would recreate it with
	// could differ, so it must NOT be flagged. B: after it -> flagged.
	auto c = candidates(
		"<system>"
		"<bone name=\"A\"/>"
		"<bone-default/>"
		"<bone name=\"B\"/>"
		"</system>");
	CHECK(!hasBone(c, "A"));
	CHECK(hasBone(c, "B"));
	CHECK(c.size() == 1);
}

TEST_CASE("a NAMED bone-default does not clear candidates")
{
	// Named bone-defaults change boneTemplates["x"], not the unnamed default, so they are
	// irrelevant to the position guard.
	auto c = candidates(
		"<system>"
		"<bone name=\"A\"/>"
		"<bone-default name=\"x\"/>"
		"<bone name=\"B\"/>"
		"</system>");
	CHECK(hasBone(c, "A"));
	CHECK(hasBone(c, "B"));
	CHECK(c.size() == 2);
}

TEST_CASE("a <bone> equal to the updated unnamed default (declared after it) is a candidate")
{
	// The unnamed bone-default sets mass=1; a later <bone> restating mass=1 equals the default
	// in force at its position, with no further bone-default after it.
	auto c = candidates(
		"<system>"
		"<bone-default><mass>1</mass></bone-default>"
		"<bone name=\"A\"><mass>1</mass></bone>"
		"</system>");
	REQUIRE(c.size() == 1);
	CHECK(c[0].boneName == "A");
}

TEST_CASE("a <bone> that does NOT match the updated default is not a candidate")
{
	auto c = candidates(
		"<system>"
		"<bone-default><mass>1</mass></bone-default>"
		"<bone name=\"A\"><mass>2</mass></bone>"
		"</system>");
	CHECK(c.empty());
}

TEST_CASE("empty or missing name attributes are ignored")
{
	CHECK(candidates("<system><bone name=\"\"/><bone/></system>").empty());
}

TEST_CASE("a document without a <system> root yields nothing")
{
	CHECK(candidates("<notsystem><bone name=\"A\"/></notsystem>").empty());
}

TEST_CASE("multiple default bones are all candidates")
{
	auto c = candidates("<system><bone name=\"A\"/><bone name=\"B\"/></system>");
	CHECK(c.size() == 2);
	CHECK(hasBone(c, "A"));
	CHECK(hasBone(c, "B"));
}

TEST_CASE("line numbers are computed from source bytes")
{
	const std::string xml =
		"<system>\n"
		"<bone name=\"A\"/>\n"
		"</system>\n";
	auto c = candidates(xml, /*withSource=*/true);
	REQUIRE(c.size() == 1);
	CHECK(c[0].line == 2);
}

// ── Cross-consumer filter (FilterSkinRedundantBonesByConsumers) ──────────────────
//
// The ecosystem half: a candidate survives only when its (case-folded) bone name is skinned
// by EVERY NIF that references the XML. These pin the zero-false-positive guarantees.

namespace
{
	hdt::InertBoneInfo cand(const std::string& name)
	{
		hdt::InertBoneInfo b;
		b.boneName = name;
		b.location = "/system[1]/bone[1]";
		b.line = 1;
		return b;
	}

	std::vector<std::string> flaggedNames(
		const std::vector<hdt::InertBoneInfo>& candidates,
		const std::vector<std::vector<std::string>>& consumers)
	{
		std::vector<std::string> out;
		for (const auto& b : hdt::FilterSkinRedundantBonesByConsumers(candidates, consumers))
			out.push_back(b.boneName);
		return out;
	}
}

TEST_CASE("a bone skinned by every consumer is flagged")
{
	auto out = flaggedNames({ cand("NPC Spine") }, { { "npc spine" }, { "npc spine", "npc head" } });
	REQUIRE(out.size() == 1);
	CHECK(out[0] == "NPC Spine");
}

TEST_CASE("a bone missing from even one consumer is not flagged")
{
	auto out = flaggedNames({ cand("NPC Spine") }, { { "npc spine" }, { "npc head" } });
	CHECK(out.empty());
}

TEST_CASE("with no consumers nothing is proven, so nothing is flagged")
{
	CHECK(flaggedNames({ cand("NPC Spine") }, {}).empty());
}

TEST_CASE("a consumer whose skin bones could not be read (empty set) drops every candidate")
{
	// The empty set represents a NIF we failed to read skinning from: it must veto, never pass.
	auto out = flaggedNames({ cand("NPC Spine") }, { { "npc spine" }, {} });
	CHECK(out.empty());
}

TEST_CASE("consumer bone names match candidates case-insensitively")
{
	// Candidate keeps author casing; consumer sets are already folded. The compare folds the candidate.
	auto out = flaggedNames({ cand("NPC L Breast01") }, { { "npc l breast01" } });
	REQUIRE(out.size() == 1);
	CHECK(out[0] == "NPC L Breast01");
}

TEST_CASE("only the candidates common to all consumers survive")
{
	auto out = flaggedNames(
		{ cand("A"), cand("B"), cand("C") },
		{ { "a", "b", "c" }, { "a", "c" } });  // B missing from consumer 2
	REQUIRE(out.size() == 2);
	CHECK(out[0] == "A");
	CHECK(out[1] == "C");
}
