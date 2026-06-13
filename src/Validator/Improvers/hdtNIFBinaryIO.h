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
		// Advance past a sized string without constructing a std::string.
		void skipSizedStr();
		void skipShortSizedStr();

	private:
		const std::vector<uint8_t>& m_data;
		size_t m_pos = 0;
		bool m_bigEndian = false;
	};

	struct ParsedNif
	{
		std::vector<uint8_t> headerPrefix;

		uint32_t version = 0;
		uint8_t endianness = 0;  // 0=big, 1=little
		bool hasExplicitEndiannessByte = true;
		uint32_t userVersion = 0;

		// BSStreamHeader fields (nif.xml BSStreamHeader, exact per-field conditions):
		uint32_t bsVersion = 0;
		std::string author;
		uint32_t bsUnknownInt = 0;           // cond: bsVersion > 130
		std::string processScript;           // cond: bsVersion < 131
		std::string exportScript;            // always present when BSStreamHeader present
		std::string maxFilepath;             // cond: bsVersion >= 103 && < 170
		std::vector<uint8_t> bsUnknownData;  // cond: bsVersion >= 170 (ExportDataSF)

		std::vector<std::string> blockTypes;
		std::vector<uint16_t> blockTypeIndex;
		std::vector<std::vector<uint8_t>> blocks;
		std::vector<std::string> strings;
		std::vector<uint32_t> groups;

		// nif.xml Footer: Num Roots (uint) + Roots (Ref[Num Roots]), since 3.3.0.13
		std::vector<int32_t> footerRoots;
	};

	// Returns the type-name string for block idx, or nullopt if idx is out of range.
	inline std::optional<std::string> blockTypeOf(const ParsedNif& parsed, int32_t idx)
	{
		if (idx < 0 || idx >= static_cast<int32_t>(parsed.blockTypeIndex.size()))
			return std::nullopt;
		uint16_t tIdx = parsed.blockTypeIndex[static_cast<size_t>(idx)];
		// Mask off PhysX high-bit (0x8000) before bounds-checking and lookup
		uint16_t masked = tIdx & 0x7FFF;
		if (masked >= parsed.blockTypes.size())
			return std::nullopt;
		return parsed.blockTypes[masked];
	}

	// BSStreamHeader version stamped on Skyrim Special Edition / AE / VR NIFs.
	// Skyrim LE (Oldrim) uses lower values (typically 83) and a different geometry
	// layout (NiTriShape/NiTriShapeData rather than BSTriShape), which the FSMP
	// physics tooling does not support.
	inline constexpr uint32_t kSkyrimSEBsVersion = 100;

	// True when the NIF predates Skyrim SE (e.g. a Skyrim LE mesh with bsVersion 83).
	// Such meshes must be converted to SE format (e.g. with SSE NIF Optimizer) before
	// the decimation/trim passes can operate on them. A bsVersion of 0 (no
	// BSStreamHeader) is treated as "not classified LE" to avoid false positives.
	bool isPreSESkyrimNif(const ParsedNif& parsed);

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
	// Binary serialisation helpers — used by hdtNIFBinaryIO and hdtNIFDecimator.
	void appendU8(std::vector<uint8_t>& out, uint8_t v);
	void appendU16(std::vector<uint8_t>& out, uint16_t v);
	void appendU32(std::vector<uint8_t>& out, uint32_t v);
	void appendU64(std::vector<uint8_t>& out, uint64_t v);
	void appendF32(std::vector<uint8_t>& out, float v);
	bool hasTypeName(const std::string& typeName, std::initializer_list<const char*> names);
}
