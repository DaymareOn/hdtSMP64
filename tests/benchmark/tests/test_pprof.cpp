// Unit tests for the pprof export (pprofWriter.cpp). These exercise the pure encoder
// encodePprofProfile() only - no Bullet, no disk - so they are a cheap layer of the suite. A tiny
// purpose-built protobuf reader decodes the bytes back and asserts the call-tree -> Profile mapping:
//   - string_table[0] is the empty string (a pprof requirement),
//   - one sample per real (non-root) node; one function/location per unique scope name,
//   - per-sample value pair is [self-time ns, calls] and the location stack depth matches the node,
//   - self-time over all samples sums to the total inclusive time (self-time derivation is correct),
//   - duration_nanos equals that same total.

#include "pprofWriter.h"

#include <doctest/doctest.h>

#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

using hdt::replay::encodePprofProfile;
using hdt::replay::prof::ScopeNode;

namespace
{
	// pprof profile.proto field numbers the test inspects (kept local; the encoder's copies are private).
	constexpr std::uint32_t kProfileSampleType = 1;
	constexpr std::uint32_t kProfileSample = 2;
	constexpr std::uint32_t kProfileLocation = 4;
	constexpr std::uint32_t kProfileFunction = 5;
	constexpr std::uint32_t kProfileStringTable = 6;
	constexpr std::uint32_t kProfileDurationNanos = 10;
	constexpr std::uint32_t kSampleLocationId = 1;
	constexpr std::uint32_t kSampleValue = 2;
	constexpr std::uint32_t kWireVarint = 0;
	constexpr std::uint32_t kWireLen = 2;

	// Minimal protobuf reader over a byte span: just enough to walk varint and length-delimited fields.
	struct Reader
	{
		const std::uint8_t* p;
		std::size_t n;
		std::size_t i = 0;

		bool atEnd() const { return i >= n; }

		std::uint64_t varint()
		{
			std::uint64_t r = 0;
			int shift = 0;
			while (i < n) {
				const std::uint8_t b = p[i++];
				r |= static_cast<std::uint64_t>(b & 0x7F) << shift;
				if (!(b & 0x80))
					break;
				shift += 7;
			}
			return r;
		}

		// Reads a field tag and returns (field_number, wire_type).
		std::pair<std::uint32_t, std::uint32_t> tag()
		{
			const std::uint64_t t = varint();
			return { static_cast<std::uint32_t>(t >> 3), static_cast<std::uint32_t>(t & 0x7) };
		}

		// Reads a length-delimited payload and returns a Reader over it, advancing past it.
		Reader sub()
		{
			const std::uint64_t len = varint();
			Reader r{ p + i, static_cast<std::size_t>(len), 0 };
			i += static_cast<std::size_t>(len);
			return r;
		}
	};

	// Counts how many varints a packed-repeated blob contains.
	std::size_t countVarints(Reader r)
	{
		std::size_t c = 0;
		while (!r.atEnd()) {
			r.varint();
			++c;
		}
		return c;
	}

	// One decoded sample: location-stack depth and the two value columns.
	struct DecodedSample
	{
		std::size_t depth = 0;
		std::uint64_t selfNs = 0;
		std::uint64_t calls = 0;
	};

	// What the test pulls out of an encoded Profile.
	struct Decoded
	{
		std::vector<std::string> strings;
		std::vector<DecodedSample> samples;
		std::size_t sampleTypes = 0;
		std::size_t functions = 0;
		std::size_t locations = 0;
		std::uint64_t durationNanos = 0;
	};

	Decoded decode(const std::vector<std::uint8_t>& bytes)
	{
		Decoded out;
		Reader top{ bytes.data(), bytes.size(), 0 };
		while (!top.atEnd()) {
			const auto [field, wire] = top.tag();
			if (wire == kWireVarint) {
				const std::uint64_t v = top.varint();
				if (field == kProfileDurationNanos)
					out.durationNanos = v;
				continue;
			}
			REQUIRE(wire == kWireLen);
			Reader payload = top.sub();
			switch (field) {
			case kProfileSampleType:
				++out.sampleTypes;
				break;
			case kProfileFunction:
				++out.functions;
				break;
			case kProfileLocation:
				++out.locations;
				break;
			case kProfileStringTable:
				out.strings.emplace_back(reinterpret_cast<const char*>(payload.p), payload.n);
				break;
			case kProfileSample: {
				DecodedSample s;
				while (!payload.atEnd()) {
					const auto [sf, sw] = payload.tag();
					REQUIRE(sw == kWireLen);
					Reader blob = payload.sub();
					if (sf == kSampleLocationId) {
						s.depth = countVarints(blob);
					} else if (sf == kSampleValue) {
						s.selfNs = blob.varint();
						s.calls = blob.varint();
					}
				}
				out.samples.push_back(s);
				break;
			}
			default:
				break;
			}
		}
		return out;
	}

	// Builds a small tree: Root -> {step(100ns,1) -> solveConstraints(60,3), collision(30,2); read(20,1)}.
	// step's self-time is 100-(60+30)=10; the others are leaves so self == total.
	ScopeNode makeTree()
	{
		ScopeNode root{ "Root" };

		ScopeNode& step = root.child("step");
		step.totalNs = 100;
		step.calls = 1;

		ScopeNode& solve = step.child("solveConstraints");
		solve.totalNs = 60;
		solve.calls = 3;

		ScopeNode& coll = step.child("collision");
		coll.totalNs = 30;
		coll.calls = 2;

		ScopeNode& read = root.child("read");
		read.totalNs = 20;
		read.calls = 1;

		return root;
	}

	// True if `samples` contains exactly one entry matching (depth, selfNs, calls).
	bool hasSample(const std::vector<DecodedSample>& samples, std::size_t depth, std::uint64_t selfNs, std::uint64_t calls)
	{
		std::size_t matches = 0;
		for (const auto& s : samples)
			if (s.depth == depth && s.selfNs == selfNs && s.calls == calls)
				++matches;
		return matches == 1;
	}
}

TEST_CASE("pprof encoder maps the call tree to samples")
{
	const ScopeNode root = makeTree();
	const Decoded d = decode(encodePprofProfile(root));

	// string_table[0] must be "" (pprof requirement).
	REQUIRE(!d.strings.empty());
	CHECK(d.strings[0] == "");

	// Two value columns: cpu/nanoseconds and calls/count.
	CHECK(d.sampleTypes == 2);

	// One sample per real node; one function/location per unique scope name (4 distinct names here).
	CHECK(d.samples.size() == 4);
	CHECK(d.functions == 4);
	CHECK(d.locations == 4);

	// Per-node value pairs and stack depths (leaf-first stack length == node depth).
	CHECK(hasSample(d.samples, 1, 10, 1));  // step:            self 10, depth 1
	CHECK(hasSample(d.samples, 2, 60, 3));  // solveConstraints: self 60, depth 2
	CHECK(hasSample(d.samples, 2, 30, 2));  // collision:        self 30, depth 2
	CHECK(hasSample(d.samples, 1, 20, 1));  // read:             self 20, depth 1

	// Self-time over all samples == total inclusive time of the top-level scopes (100 + 20).
	std::uint64_t selfSum = 0;
	for (const auto& s : d.samples)
		selfSum += s.selfNs;
	CHECK(selfSum == 120);
	CHECK(d.durationNanos == 120);
}

TEST_CASE("pprof encoder on an empty tree is still a valid profile")
{
	const ScopeNode root{ "Root" };
	const Decoded d = decode(encodePprofProfile(root));

	REQUIRE(!d.strings.empty());
	CHECK(d.strings[0] == "");
	CHECK(d.sampleTypes == 2);
	CHECK(d.samples.empty());
	CHECK(d.functions == 0);
	CHECK(d.locations == 0);
	CHECK(d.durationNanos == 0);
}
