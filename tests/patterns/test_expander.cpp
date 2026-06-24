// Unit tests for the SMP XML pattern expander (hdtXmlPatternExpander). These exercise hdtXmlPatternExpander.cpp
// and pugixml only -- no Bullet / CommonLibSSE. Coverage maps 1:1 to the eng-review test diagram:
// short-circuit passthrough, parse failure, substitution, defaults, repeat (count/from/nesting/arithmetic),
// every fail-closed error path, the trust-boundary caps, and the cape/tissue golden expansions.

#include "hdtXmlPatternExpander.h"

#include <doctest/doctest.h>
#include <pugixml.hpp>

#include <cstring>
#include <string>

namespace
{
	void countRec(const pugi::xml_node& n, const char* tag, int& c)
	{
		for (pugi::xml_node ch : n.children())
		{
			if (ch.type() == pugi::node_element)
			{
				if (std::strcmp(ch.name(), tag) == 0)
					++c;
				countRec(ch, tag, c);
			}
		}
	}

	// Counts elements named `tag` anywhere in `xml`. Returns -1 if `xml` does not parse (a test failure).
	int countTag(const std::string& xml, const char* tag)
	{
		pugi::xml_document d;
		if (!d.load_buffer(xml.data(), xml.size()))
			return -1;
		int c = 0;
		countRec(d, tag, c);
		return c;
	}

	bool has(const std::string& hay, const std::string& needle)
	{
		return hay.find(needle) != std::string::npos;
	}
}

using hdt::expandPatterns;
using hdt::PatternLimits;

TEST_CASE("no-pattern input is returned byte-identical with no parsing")
{
	const std::string xml = "<system><bone name=\"a\"/></system>";
	const auto r = expandPatterns(xml);
	CHECK(r.ok);
	CHECK_FALSE(r.changed);
	CHECK(r.xml == xml);
}

TEST_CASE("malformed XML fails closed and leaves input unchanged")
{
	const std::string xml = "<system><pattern name=\"x\"></system>";  // <pattern> never closed
	const auto r = expandPatterns(xml);
	CHECK_FALSE(r.ok);
	CHECK(r.xml == xml);
	CHECK_FALSE(r.diags.empty());
}

TEST_CASE("parameters substitute into attributes and text; definition and use are removed")
{
	const std::string xml =
		"<system>"
		"<pattern-default name=\"p\"><param name=\"n\"/><body>"
		"<bone name=\"${n}\">${n}_tail</bone>"
		"</body></pattern-default>"
		"<pattern name=\"p\" n=\"NPC_X\"/>"
		"</system>";
	const auto r = expandPatterns(xml);
	REQUIRE(r.ok);
	CHECK(r.changed);
	CHECK(has(r.xml, "name=\"NPC_X\""));
	CHECK(has(r.xml, "NPC_X_tail"));
	CHECK_FALSE(has(r.xml, "pattern-default"));  // definition not emitted
	CHECK_FALSE(has(r.xml, "<pattern "));        // use replaced
	CHECK(countTag(r.xml, "bone") == 1);
}

TEST_CASE("a default value is used when the parameter is omitted")
{
	const std::string xml =
		"<system><pattern-default name=\"p\"><param name=\"rows\" default=\"4\"/>"
		"<body><bone name=\"b${rows}\"/></body></pattern-default>"
		"<pattern name=\"p\"/></system>";
	const auto r = expandPatterns(xml);
	REQUIRE(r.ok);
	CHECK(has(r.xml, "name=\"b4\""));
}

TEST_CASE("a missing required parameter is an error")
{
	const std::string xml =
		"<system><pattern-default name=\"p\"><param name=\"n\"/><body><bone name=\"${n}\"/></body></pattern-default>"
		"<pattern name=\"p\"/></system>";
	const auto r = expandPatterns(xml);
	CHECK_FALSE(r.ok);
	REQUIRE_FALSE(r.diags.empty());
	CHECK(has(r.diags[0].message, "required"));
}

TEST_CASE("an unknown ${variable} is an error")
{
	const std::string xml =
		"<system><pattern-default name=\"p\"><body><bone name=\"${ghost}\"/></body></pattern-default>"
		"<pattern name=\"p\"/></system>";
	const auto r = expandPatterns(xml);
	CHECK_FALSE(r.ok);
	REQUIRE_FALSE(r.diags.empty());
	CHECK(has(r.diags[0].message, "ghost"));
}

TEST_CASE("using an undefined pattern is an error")
{
	const auto r = expandPatterns("<system><pattern name=\"nope\"/></system>");
	CHECK_FALSE(r.ok);
	REQUIRE_FALSE(r.diags.empty());
	CHECK(has(r.diags[0].message, "undefined"));
}

TEST_CASE("a stray attribute on a pattern use is an error (typo protection)")
{
	const std::string xml =
		"<system><pattern-default name=\"p\"><param name=\"n\"/><body><bone name=\"${n}\"/></body></pattern-default>"
		"<pattern name=\"p\" n=\"x\" prfix=\"y\"/></system>";  // 'prfix' typo
	const auto r = expandPatterns(xml);
	CHECK_FALSE(r.ok);
	REQUIRE_FALSE(r.diags.empty());
	CHECK(has(r.diags[0].message, "prfix"));
}

TEST_CASE("a cape: <repeat> with count and from unrolls a 1-D chain")
{
	const std::string xml =
		"<system><pattern-default name=\"cape\"><param name=\"p\"/><body>"
		"<repeat var=\"i\" count=\"3\"><bone name=\"${p}_${i}\"/></repeat>"
		"<repeat var=\"i\" count=\"2\" from=\"1\">"
		"<generic-constraint bodyA=\"${p}_${i-1}\" bodyB=\"${p}_${i}\" template=\"link\"/>"
		"</repeat>"
		"</body></pattern-default>"
		"<pattern name=\"cape\" p=\"K\"/></system>";
	const auto r = expandPatterns(xml);
	REQUIRE(r.ok);
	CHECK(countTag(r.xml, "bone") == 3);                  // K_0, K_1, K_2
	CHECK(countTag(r.xml, "generic-constraint") == 2);   // 0-1, 1-2
	CHECK(has(r.xml, "name=\"K_2\""));
	CHECK(has(r.xml, "bodyA=\"K_0\""));
	CHECK(has(r.xml, "bodyB=\"K_2\""));
}

TEST_CASE("a non-integer <repeat count> is an error")
{
	const std::string xml =
		"<system><pattern-default name=\"p\"><body>"
		"<repeat var=\"i\" count=\"abc\"><bone name=\"b${i}\"/></repeat>"
		"</body></pattern-default><pattern name=\"p\"/></system>";
	const auto r = expandPatterns(xml);
	CHECK_FALSE(r.ok);
	REQUIRE_FALSE(r.diags.empty());
	CHECK(has(r.diags[0].message, "integer"));
}

TEST_CASE("a <repeat count> over the cap is an error, never a blow-up")
{
	const std::string xml =
		"<system><pattern-default name=\"p\"><body>"
		"<repeat var=\"i\" count=\"6\"><bone name=\"b${i}\"/></repeat>"
		"</body></pattern-default><pattern name=\"p\"/></system>";
	PatternLimits limits;
	limits.maxRepeatCount = 5;
	const auto r = expandPatterns(xml, limits);
	CHECK_FALSE(r.ok);
	REQUIRE_FALSE(r.diags.empty());
	CHECK(has(r.diags[0].message, "cap"));
}

TEST_CASE("a tissue: nested <repeat> + index arithmetic builds a 2-D grid with diagonal links")
{
	const std::string xml =
		"<system><pattern-default name=\"tissue\">"
		"<param name=\"p\"/><param name=\"rows\"/><param name=\"cols\"/><body>"
		"<repeat var=\"i\" count=\"${rows}\"><repeat var=\"j\" count=\"${cols}\">"
		"<bone name=\"${p}_${i}_${j}\"/>"
		"</repeat></repeat>"
		"<repeat var=\"i\" count=\"${rows-1}\"><repeat var=\"j\" count=\"${cols-1}\">"
		"<generic-constraint bodyA=\"${p}_${i}_${j}\" bodyB=\"${p}_${i+1}_${j+1}\" template=\"x\"/>"
		"</repeat></repeat>"
		"</body></pattern-default>"
		"<pattern name=\"tissue\" p=\"T\" rows=\"3\" cols=\"3\"/></system>";
	const auto r = expandPatterns(xml);
	REQUIRE(r.ok);
	CHECK(countTag(r.xml, "bone") == 9);                 // 3x3
	CHECK(countTag(r.xml, "generic-constraint") == 4);   // 2x2 diagonals
	CHECK(has(r.xml, "name=\"T_2_2\""));
	CHECK(has(r.xml, "bodyA=\"T_1_1\""));
	CHECK(has(r.xml, "bodyB=\"T_2_2\""));                // i=1,j=1 -> i+1,j+1
}

TEST_CASE("index arithmetic on a non-integer value is an error")
{
	const std::string xml =
		"<system><pattern-default name=\"p\"><param name=\"s\" default=\"abc\"/>"
		"<body><bone name=\"b${s+1}\"/></body></pattern-default>"
		"<pattern name=\"p\"/></system>";
	const auto r = expandPatterns(xml);
	CHECK_FALSE(r.ok);
	REQUIRE_FALSE(r.diags.empty());
	CHECK(has(r.diags[0].message, "integer"));
}

TEST_CASE("one pattern may use another (bounded nesting)")
{
	const std::string xml =
		"<system>"
		"<pattern-default name=\"inner\"><param name=\"x\"/><body><bone name=\"${x}\"/></body></pattern-default>"
		"<pattern-default name=\"outer\"><param name=\"y\"/><body><pattern name=\"inner\" x=\"${y}\"/></body></pattern-default>"
		"<pattern name=\"outer\" y=\"Z\"/></system>";
	const auto r = expandPatterns(xml);
	REQUIRE(r.ok);
	CHECK(has(r.xml, "name=\"Z\""));
	CHECK(countTag(r.xml, "bone") == 1);
}

TEST_CASE("runaway pattern recursion trips the depth cap instead of looping forever")
{
	const std::string xml =
		"<system><pattern-default name=\"loop\"><body><pattern name=\"loop\"/></body></pattern-default>"
		"<pattern name=\"loop\"/></system>";
	PatternLimits limits;
	limits.maxRecursionDepth = 3;
	const auto r = expandPatterns(xml, limits);
	CHECK_FALSE(r.ok);
	REQUIRE_FALSE(r.diags.empty());
	CHECK(has(r.diags[0].message, "nesting"));
}

TEST_CASE("exceeding the total element cap is an error")
{
	const std::string xml =
		"<system><pattern-default name=\"p\"><body>"
		"<repeat var=\"i\" count=\"5\"><bone name=\"b${i}\"/></repeat>"
		"</body></pattern-default><pattern name=\"p\"/></system>";
	PatternLimits limits;
	limits.maxExpandedElements = 2;
	const auto r = expandPatterns(xml, limits);
	CHECK_FALSE(r.ok);
	REQUIRE_FALSE(r.diags.empty());
	CHECK(has(r.diags[0].message, "element cap"));
}

TEST_CASE("a duplicate <pattern-default> name is an error")
{
	const std::string xml =
		"<system>"
		"<pattern-default name=\"p\"><body><bone name=\"a\"/></body></pattern-default>"
		"<pattern-default name=\"p\"><body><bone name=\"b\"/></body></pattern-default>"
		"<pattern name=\"p\"/></system>";
	const auto r = expandPatterns(xml);
	CHECK_FALSE(r.ok);
	REQUIRE_FALSE(r.diags.empty());
	CHECK(has(r.diags[0].message, "duplicate"));
}

TEST_CASE("a <pattern-default> with no <body> is an error")
{
	const std::string xml =
		"<system><pattern-default name=\"p\"><param name=\"n\"/></pattern-default>"
		"<pattern name=\"p\" n=\"x\"/></system>";
	const auto r = expandPatterns(xml);
	CHECK_FALSE(r.ok);
	REQUIRE_FALSE(r.diags.empty());
	CHECK(has(r.diags[0].message, "body"));
}

TEST_CASE("no markers leak into the output and a clean file has an empty source map")
{
	const auto r = expandPatterns("<system><bone name=\"a\"/></system>");
	CHECK(r.sourceMap.ranges.empty());
	CHECK_FALSE(has(r.xml, "_fsmp_pat"));
}

TEST_CASE("the source map attributes generated text to the pattern and its use line")
{
	const std::string xml =
		"<system>\n"                                  // line 1
		"  <pattern-default name=\"cape\">\n"          // line 2
		"    <param name=\"p\"/>\n"                    // line 3
		"    <body><bone name=\"${p}_0\"/></body>\n"   // line 4
		"  </pattern-default>\n"                       // line 5
		"  <pattern name=\"cape\" p=\"K\"/>\n"         // line 6
		"</system>\n";                                 // line 7
	const auto r = expandPatterns(xml);
	REQUIRE(r.ok);
	CHECK_FALSE(has(r.xml, "_fsmp_pat"));  // transient marker fully stripped
	REQUIRE(r.sourceMap.ranges.size() == 1);

	const std::size_t off = r.xml.find("K_0");
	REQUIRE(off != std::string::npos);
	const hdt::PatternRange* rng = r.sourceMap.find(off);
	REQUIRE(rng != nullptr);
	CHECK(rng->patternName == "cape");
	CHECK(rng->useLine == 6);
	// the range really covers the generated element's bytes in the clean output
	CHECK(r.xml.substr(rng->lo, rng->hi - rng->lo).find("K_0") != std::string::npos);
	// a hand-written offset (the <system> root) is not attributed to any pattern
	CHECK(r.sourceMap.find(0) == nullptr);
}

TEST_CASE("nested patterns attribute generated text to the innermost use")
{
	const std::string xml =
		"<system>\n"                                                                                            // 1
		"<pattern-default name=\"inner\"><param name=\"x\"/><body><bone name=\"${x}\"/></body></pattern-default>\n"  // 2
		"<pattern-default name=\"outer\"><param name=\"y\"/><body><pattern name=\"inner\" x=\"${y}\"/></body></pattern-default>\n"  // 3
		"<pattern name=\"outer\" y=\"Z\"/>\n"                                                                    // 4
		"</system>\n";
	const auto r = expandPatterns(xml);
	REQUIRE(r.ok);
	const std::size_t off = r.xml.find("\"Z\"");
	REQUIRE(off != std::string::npos);
	const hdt::PatternRange* rng = r.sourceMap.find(off);
	REQUIRE(rng != nullptr);
	CHECK(rng->patternName == "inner");  // innermost wins
	CHECK(rng->useLine == 3);            // the inner use sits inside outer's body on line 3
}
