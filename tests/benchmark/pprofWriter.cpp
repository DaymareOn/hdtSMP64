#include "pprofWriter.h"

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
	using hdt::replay::prof::ScopeNode;
	using Bytes = std::vector<std::uint8_t>;

	// ── protobuf wire constants (project "Magic values" rule: named, never inline literals) ──────────
	// Wire types (low 3 bits of a field tag).
	constexpr std::uint32_t kWireVarint = 0;
	constexpr std::uint32_t kWireLen = 2;

	// Field numbers from pprof's profile.proto (only the ones we emit).
	constexpr std::uint32_t kProfileSampleType = 1;     // repeated ValueType
	constexpr std::uint32_t kProfileSample = 2;         // repeated Sample
	constexpr std::uint32_t kProfileLocation = 4;       // repeated Location
	constexpr std::uint32_t kProfileFunction = 5;       // repeated Function
	constexpr std::uint32_t kProfileStringTable = 6;    // repeated string
	constexpr std::uint32_t kProfileDurationNanos = 10; // int64
	constexpr std::uint32_t kValueTypeType = 1;         // int64 (string index)
	constexpr std::uint32_t kValueTypeUnit = 2;         // int64 (string index)
	constexpr std::uint32_t kSampleLocationId = 1;      // repeated uint64 (packed)
	constexpr std::uint32_t kSampleValue = 2;           // repeated int64 (packed)
	constexpr std::uint32_t kLocationId = 1;            // uint64
	constexpr std::uint32_t kLocationLine = 4;          // repeated Line
	constexpr std::uint32_t kLineFunctionId = 1;        // uint64
	constexpr std::uint32_t kFunctionId = 1;            // uint64
	constexpr std::uint32_t kFunctionName = 2;          // int64 (string index)
	constexpr std::uint32_t kFunctionSystemName = 3;    // int64 (string index)

	// ── low-level protobuf wire helpers ─────────────────────────────────────────────────────────────

	/// Appends a base-128 varint (LEB128): 7 payload bits per byte, high bit set on all but the last.
	void putVarint(Bytes& b, std::uint64_t v)
	{
		while (v >= 0x80) {
			b.push_back(static_cast<std::uint8_t>(v | 0x80));
			v >>= 7;
		}
		b.push_back(static_cast<std::uint8_t>(v));
	}

	/// Appends a field tag: (field_number << 3) | wire_type.
	void putTag(Bytes& b, std::uint32_t field, std::uint32_t wire)
	{
		putVarint(b, (static_cast<std::uint64_t>(field) << 3) | wire);
	}

	/// Appends a varint-typed field (tag + value).
	void putVarintField(Bytes& b, std::uint32_t field, std::uint64_t value)
	{
		putTag(b, field, kWireVarint);
		putVarint(b, value);
	}

	/// Appends a length-delimited field (tag + length + payload bytes) - used for sub-messages,
	/// strings, and packed repeated scalars.
	void putLenField(Bytes& b, std::uint32_t field, const Bytes& payload)
	{
		putTag(b, field, kWireLen);
		putVarint(b, payload.size());
		b.insert(b.end(), payload.begin(), payload.end());
	}

	/// Appends a length-delimited string field.
	void putStrField(Bytes& b, std::uint32_t field, const std::string& s)
	{
		putTag(b, field, kWireLen);
		putVarint(b, s.size());
		b.insert(b.end(), s.begin(), s.end());
	}

	// ── Profile builder ─────────────────────────────────────────────────────────────────────────────

	// Accumulates the string table, the per-name Function/Location records, and the per-node Samples,
	// then assembles them into the final Profile message. One Function and one Location are created per
	// unique scope name (locations carry no address, so name is the identity); the call structure lives
	// entirely in each Sample's leaf-first location stack.
	struct Builder
	{
		std::vector<std::string> strings;
		std::unordered_map<std::string, std::int64_t> stringIndex;
		std::unordered_map<std::string, std::uint64_t> locIdByName;
		Bytes functions;  // concatenated Function records (each as its own length-delimited field)
		Bytes locations;  // concatenated Location records
		Bytes samples;    // concatenated Sample records
		std::uint64_t nextId = 1;

		Builder()
		{
			intern("");  // pprof requires string_table[0] == ""
		}

		/// Returns the string-table index of `s`, appending it on first use.
		std::int64_t intern(const std::string& s)
		{
			auto it = stringIndex.find(s);
			if (it != stringIndex.end())
				return it->second;
			const auto idx = static_cast<std::int64_t>(strings.size());
			strings.push_back(s);
			stringIndex.emplace(s, idx);
			return idx;
		}

		/// Returns the location id for a scope name, creating the matching Function + Location records
		/// on first use. Function id and Location id share the integer (they live in separate id spaces,
		/// so reusing the value is unambiguous and keeps the mapping one-to-one).
		std::uint64_t locFor(const std::string& name)
		{
			auto it = locIdByName.find(name);
			if (it != locIdByName.end())
				return it->second;

			const std::uint64_t id = nextId++;
			locIdByName.emplace(name, id);
			const auto nameIdx = static_cast<std::uint64_t>(intern(name));

			Bytes fn;
			putVarintField(fn, kFunctionId, id);
			putVarintField(fn, kFunctionName, nameIdx);
			putVarintField(fn, kFunctionSystemName, nameIdx);
			putLenField(functions, kProfileFunction, fn);

			Bytes line;
			putVarintField(line, kLineFunctionId, id);
			Bytes loc;
			putVarintField(loc, kLocationId, id);
			putLenField(loc, kLocationLine, line);
			putLenField(locations, kProfileLocation, loc);

			return id;
		}

		/// Emits one Sample: a leaf-first stack of location ids plus the [self ns, calls] value pair.
		void addSample(const std::vector<std::uint64_t>& stackRootFirst, std::uint64_t selfNs, std::uint64_t calls)
		{
			Bytes locBlob;
			for (auto i = stackRootFirst.rbegin(); i != stackRootFirst.rend(); ++i)
				putVarint(locBlob, *i);  // leaf-first: reverse of the root->node path

			Bytes valBlob;
			putVarint(valBlob, selfNs);
			putVarint(valBlob, calls);

			Bytes sample;
			putLenField(sample, kSampleLocationId, locBlob);
			putLenField(sample, kSampleValue, valBlob);
			putLenField(samples, kProfileSample, sample);
		}

		/// One ValueType sub-message (its type/unit names interned into the string table).
		Bytes valueType(const std::string& type, const std::string& unit)
		{
			Bytes vt;
			putVarintField(vt, kValueTypeType, static_cast<std::uint64_t>(intern(type)));
			putVarintField(vt, kValueTypeUnit, static_cast<std::uint64_t>(intern(unit)));
			return vt;
		}
	};

	/// Sum of a node's direct children's inclusive times (used to derive self-time).
	std::uint64_t childTotal(const ScopeNode& node)
	{
		std::uint64_t total = 0;
		for (const auto& c : node.children)
			total += c->totalNs;
		return total;
	}

	/// Depth-first walk that emits one sample per real scope node. `stack` holds the root->node path of
	/// location ids; self-time is the node's inclusive time minus its children's (clamped at 0 in the
	/// degenerate case where rounding makes children exceed the parent).
	void walk(Builder& b, const ScopeNode& node, bool isRoot, std::vector<std::uint64_t>& stack)
	{
		if (!isRoot) {
			stack.push_back(b.locFor(node.name));
			const std::uint64_t kids = childTotal(node);
			const std::uint64_t selfNs = node.totalNs > kids ? node.totalNs - kids : 0;
			b.addSample(stack, selfNs, node.calls);
		}
		for (const auto& c : node.children)
			walk(b, *c, false, stack);
		if (!isRoot)
			stack.pop_back();
	}
}

namespace hdt::replay
{
	std::vector<std::uint8_t> encodePprofProfile(const prof::ScopeNode& root)
	{
		Builder b;
		std::vector<std::uint64_t> stack;
		walk(b, root, true, stack);

		Bytes out;
		// sample_type: column 0 = cpu/nanoseconds, column 1 = calls/count.
		putLenField(out, kProfileSampleType, b.valueType("cpu", "nanoseconds"));
		putLenField(out, kProfileSampleType, b.valueType("calls", "count"));
		// samples / locations / functions were pre-encoded as their own length-delimited fields.
		out.insert(out.end(), b.samples.begin(), b.samples.end());
		out.insert(out.end(), b.locations.begin(), b.locations.end());
		out.insert(out.end(), b.functions.begin(), b.functions.end());
		// string table, in index order.
		for (const auto& s : b.strings)
			putStrField(out, kProfileStringTable, s);
		// total profiled time (informational): sum of the top-level scopes' inclusive time.
		putVarintField(out, kProfileDurationNanos, childTotal(root));

		return out;
	}

	void writePprofProfile(const prof::ScopeNode& root, const std::string& path)
	{
		const Bytes bytes = encodePprofProfile(root);
		std::ofstream out(path, std::ios::binary | std::ios::trunc);
		if (!out)
			throw std::runtime_error("could not open pprof output file: " + path);
		out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
		if (!out)
			throw std::runtime_error("could not write pprof output file: " + path);
	}
}
