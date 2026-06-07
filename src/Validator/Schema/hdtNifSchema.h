#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace hdt
{
	// ── Basic type record ─────────────────────────────────────────────────────
	struct NifBasicType
	{
		size_t size = 0;
		bool isRef = false;  // block reference (Ref or Ptr)
		bool isPtr = false;  // tUpLink / back-reference (Ptr only, subset of isRef)
	};

	// ── Field definition (version conditions already pre-evaluated) ───────────
	struct NifFieldDef
	{
		std::string name;
		std::string type;
		std::string arr1;  // count-field name; empty → scalar
		std::string cond;  // runtime condition expression; empty → always present
	};

	// ── Type definition (compound or niobject) ────────────────────────────────
	struct NifTypeDef
	{
		std::string name;
		std::string inherit;              // parent type name; empty → no parent
		std::vector<NifFieldDef> fields;  // own fields only (after version filtering)
		bool isBlock = false;             // true if declared as <niobject>
		bool hasAnyRefs = false;          // true if type (or any ancestor/compound) contains a Ref/Ptr field
	};

	// ── Version context used for pre-evaluation ───────────────────────────────
	struct NifSchemaVersion
	{
		uint32_t version = 0;      // e.g. 0x14020007
		uint32_t userVersion = 0;  // e.g. 12
		uint32_t bsVersion = 0;    // e.g. 100
	};

	// ── Schema ────────────────────────────────────────────────────────────────
	// Parses a niftools-format nif.xml, pre-evaluates version conditions for the
	// supplied NifSchemaVersion, and exposes the resulting type model.
	class NifSchema
	{
	public:
		NifSchema() = default;

		// Parse XML from a string (the embedded SSE schema or an external file).
		// Returns false on parse failure; partial results are discarded.
		bool loadFromString(const char* xml, const NifSchemaVersion& ver);

		// Try to load from a file first; fall back to the built-in SSE schema.
		bool loadForSSE(const std::string& externalXmlPath = {});

		const NifBasicType* findBasic(const std::string& name) const;
		const NifTypeDef* findType(const std::string& name) const;
		bool isKnown(const std::string& name) const;

	private:
		std::map<std::string, NifBasicType> m_basics;
		std::map<std::string, NifTypeDef> m_types;
		NifSchemaVersion m_ver{};

		static uint32_t parseVerStr(const std::string& v);
		bool versionOk(const std::string& ver1, const std::string& ver2,
			const std::string& uver1, const std::string& uver2,
			const std::string& bsver1, const std::string& bsver2) const;
	};

	// ── Block walker ──────────────────────────────────────────────────────────
	enum class LinkFilter
	{
		All,      // all Ref and Ptr fields  (childLinks ∪ parentLinks)
		RefOnly,  // only Ref / tLink fields  (childLinks)
		PtrOnly,  // only Ptr / tUpLink fields (parentLinks)
	};

	// Given a schema and a parsed-NIF block's binary data, returns the byte
	// offsets of fields matching the requested LinkFilter.  Returns nullopt if
	// the type is unknown or the data is structurally inconsistent (caller
	// should fall back to the heuristic scanner).
	std::optional<std::vector<size_t>> walkBlockRefs(
		const NifSchema& schema,
		const std::string& blockTypeName,
		const uint8_t* data,
		size_t dataSize,
		int32_t totalBlocks,
		LinkFilter filter = LinkFilter::All,
		bool log = false);

	// Extended walk result used by compactNullArrayRefs.
	// For each Ref/Ptr field: the byte offset, the arr1 field name (empty for
	// scalar Refs), and — for each named count field — its byte offset in the block.
	struct BlockRefDetail
	{
		size_t offset;
		std::string arr1;  // empty → scalar Ref; non-empty → array element
	};
	struct BlockRefDetails
	{
		std::vector<BlockRefDetail> refs;
		std::map<std::string, size_t> countFieldPos;  // count field name → byte offset
	};
	std::optional<BlockRefDetails> walkBlockRefDetails(
		const NifSchema& schema,
		const std::string& blockTypeName,
		const uint8_t* data,
		size_t dataSize,
		int32_t totalBlocks);

	// The built-in SSE-focused nif.xml (defined in hdtNifSchema.cpp)
	const char* getBuiltinNifSchemaXml();

	// Process-wide schema singleton, loaded once for SSE on first call.
	const NifSchema& globalNifSchema();
}
