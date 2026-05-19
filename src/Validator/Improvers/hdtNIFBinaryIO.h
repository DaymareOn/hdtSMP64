#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
	class NifReader
	{
	public:
		NifReader(const std::vector<uint8_t>& data, size_t pos = 0);

		bool canRead(size_t bytes) const;
		size_t pos() const;
		void setBigEndian(bool isBigEndian);

		void skip(size_t bytes);
		uint8_t readU8();
		uint16_t readU16();
		uint32_t readU32();
		uint64_t readU64();
		float readF32();
		std::vector<uint8_t> readBytes(size_t bytes);
		std::string readSizedStr();
		std::string readShortSizedStr();

	private:
		const std::vector<uint8_t>& m_data;
		size_t m_pos = 0;
		bool m_bigEndian = false;
	};

	struct ParsedNif
	{
		std::vector<uint8_t> headerPrefix;

		uint32_t version   = 0;
		uint8_t  endianness = 0;           // 0=big, 1=little
		bool     hasExplicitEndiannessByte = true;
		uint32_t userVersion = 0;

		// BSStreamHeader fields (nif.xml BSStreamHeader, exact per-field conditions):
		uint32_t    bsVersion     = 0;
		std::string author;
		uint32_t    bsUnknownInt  = 0;      // cond: bsVersion > 130
		std::string processScript;          // cond: bsVersion < 131
		std::string exportScript;           // always present when BSStreamHeader present
		std::string maxFilepath;            // cond: bsVersion >= 103 && < 170
		std::vector<uint8_t> bsUnknownData; // cond: bsVersion >= 170 (ExportDataSF)

		std::vector<std::string>          blockTypes;
		std::vector<uint16_t>             blockTypeIndex;
		std::vector<std::vector<uint8_t>> blocks;
		std::vector<std::string>          strings;
		std::vector<uint32_t>             groups;

		// nif.xml Footer: Num Roots (uint) + Roots (Ref[Num Roots]), since 3.3.0.13
		std::vector<int32_t> footerRoots;
	};

	std::optional<ParsedNif> parseNif(const std::vector<uint8_t>& data, std::string* outError = nullptr);
	// Serialize parsed to bytes. Throws on invalid structure (e.g. short string > 255 bytes).
	std::vector<uint8_t> serializeNif(const ParsedNif& parsed);
	// Serialize and write to disk. Returns false on I/O or serialization error.
	bool writeNifFile(const ParsedNif& parsed, const std::string& dstPath);
	// Write already-serialized bytes to disk. Returns false on I/O error.
	bool writeNifBytes(const std::vector<uint8_t>& bytes, const std::string& dstPath);
	// Round-trip validate: serialize then re-parse and check structural consistency.
	// Returns nullopt on success, or a string describing the first failure found.
	std::optional<std::string> validateNifRoundTrip(const ParsedNif& parsed);
	// Round-trip validate from already-serialized bytes (skips the serialization step).
	std::optional<std::string> validateNifRoundTripFromBytes(
		const std::vector<uint8_t>& bytes, const ParsedNif& original);
	void appendU8(std::vector<uint8_t>& out, uint8_t v);
	void appendU16(std::vector<uint8_t>& out, uint16_t v);
	void appendU32(std::vector<uint8_t>& out, uint32_t v);
	void appendU64(std::vector<uint8_t>& out, uint64_t v);
	void appendF32(std::vector<uint8_t>& out, float v);
	bool hasTypeName(const std::string& typeName, std::initializer_list<const char*> names);
}