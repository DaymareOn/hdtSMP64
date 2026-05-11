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
	};

	struct ParsedNif
	{
		std::vector<uint8_t> headerPrefix;

		uint32_t version = 0;
		uint8_t endianness = 0;
		uint32_t userVersion = 0;
		uint32_t bsVersion = 0;

		std::string author;
		std::string processScript;
		std::string exportScript;
		bool hasLegacyBsReservedU32 = false;
		uint32_t legacyBsReservedU32 = 0;

		std::vector<std::string> blockTypes;
		std::vector<uint16_t> blockTypeIndex;
		std::vector<std::vector<uint8_t>> blocks;
		std::vector<std::string> strings;
		std::vector<uint32_t> groups;
	};

	std::optional<ParsedNif> parseNif(const std::vector<uint8_t>& data);
	bool writeNifFile(const ParsedNif& parsed, const std::string& dstPath);
	void appendU8(std::vector<uint8_t>& out, uint8_t v);
	void appendU16(std::vector<uint8_t>& out, uint16_t v);
	void appendU32(std::vector<uint8_t>& out, uint32_t v);
	void appendU64(std::vector<uint8_t>& out, uint64_t v);
	void appendF32(std::vector<uint8_t>& out, float v);
	std::unordered_set<int32_t> collectPotentialRefs(const std::vector<uint8_t>& blockData, int32_t numBlocks);
	bool hasTypeName(const std::string& typeName, std::initializer_list<const char*> names);
}