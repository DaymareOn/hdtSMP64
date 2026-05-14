#include "hdtNIFBinaryIO.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace hdt
{
	static constexpr uint32_t kVersion_20_2_0_7 = 0x14020007u;
	static constexpr uint32_t kMaxStringLength = 4096u;

	NifReader::NifReader(const std::vector<uint8_t>& data, size_t pos) :
		m_data(data), m_pos(pos) {}

	bool NifReader::canRead(size_t bytes) const
	{
		return m_pos + bytes <= m_data.size();
	}

	size_t NifReader::pos() const
	{
		return m_pos;
	}

	void NifReader::skip(size_t bytes)
	{
		if (!canRead(bytes))
			throw std::runtime_error("NIF improver: skip past end");
		m_pos += bytes;
	}

	uint8_t NifReader::readU8()
	{
		if (!canRead(1))
			throw std::runtime_error("NIF improver: read past end (U8)");
		return m_data[m_pos++];
	}

	uint16_t NifReader::readU16()
	{
		if (!canRead(2))
			throw std::runtime_error("NIF improver: read past end (U16)");
		uint16_t v;
		std::memcpy(&v, m_data.data() + m_pos, 2);
		m_pos += 2;
		return v;
	}

	uint32_t NifReader::readU32()
	{
		if (!canRead(4))
			throw std::runtime_error("NIF improver: read past end (U32)");
		uint32_t v;
		std::memcpy(&v, m_data.data() + m_pos, 4);
		m_pos += 4;
		return v;
	}

	uint64_t NifReader::readU64()
	{
		if (!canRead(8))
			throw std::runtime_error("NIF improver: read past end (U64)");
		uint64_t v;
		std::memcpy(&v, m_data.data() + m_pos, 8);
		m_pos += 8;
		return v;
	}

	float NifReader::readF32()
	{
		if (!canRead(4))
			throw std::runtime_error("NIF improver: read past end (F32)");
		float v;
		std::memcpy(&v, m_data.data() + m_pos, 4);
		m_pos += 4;
		return v;
	}

	std::vector<uint8_t> NifReader::readBytes(size_t bytes)
	{
		if (!canRead(bytes))
			throw std::runtime_error("NIF improver: read past end (bytes)");
		std::vector<uint8_t> out(bytes);
		if (bytes)
			std::memcpy(out.data(), m_data.data() + m_pos, bytes);
		m_pos += bytes;
		return out;
	}

	std::string NifReader::readSizedStr()
	{
		uint32_t len = readU32();
		if (len > kMaxStringLength)
			throw std::runtime_error("NIF improver: implausible string length");
		if (!canRead(len))
			throw std::runtime_error("NIF improver: string out of bounds");
		std::string s(reinterpret_cast<const char*>(m_data.data() + m_pos), len);
		m_pos += len;
		return s;
	}

	std::string NifReader::readShortSizedStr()
	{
		uint8_t len = readU8();
		if (!canRead(len))
			throw std::runtime_error("NIF improver: short string out of bounds");
		std::string s(reinterpret_cast<const char*>(m_data.data() + m_pos), len);
		m_pos += len;
		return s;
	}

	static std::optional<size_t> findHeaderEnd(const std::vector<uint8_t>& data)
	{
		size_t limit = std::min<size_t>(data.size(), 200);
		for (size_t i = 0; i < limit; ++i) {
			if (data[i] == 0x00)
				return i + 1;
		}
		return std::nullopt;
	}

	std::optional<ParsedNif> parseNif(const std::vector<uint8_t>& data)
	{
		auto headerEndOpt = findHeaderEnd(data);
		if (!headerEndOpt.has_value())
			return std::nullopt;

		const size_t headerEnd = *headerEndOpt;
		std::string header(reinterpret_cast<const char*>(data.data()), headerEnd - 1);
		if (header.find("Gamebryo File Format") == std::string::npos)
			return std::nullopt;

		try {
			ParsedNif parsed;
			parsed.headerPrefix.assign(data.begin(), data.begin() + headerEnd);

			NifReader r(data, headerEnd);
			parsed.version = r.readU32();
			if (parsed.version != kVersion_20_2_0_7)
				return std::nullopt;

			parsed.endianness = r.readU8();
			parsed.userVersion = r.readU32();
			uint32_t numBlocks = r.readU32();
			if (numBlocks > 100000u)
				return std::nullopt;

			if (parsed.userVersion >= 10) {
				parsed.bsVersion = r.readU32();
				parsed.author = r.readShortSizedStr();
				if (parsed.bsVersion >= 130) {
					parsed.processScript = r.readShortSizedStr();
					parsed.exportScript = r.readShortSizedStr();
				} else {
					parsed.hasLegacyBsReservedU32 = true;
					parsed.legacyBsReservedU32 = r.readU32();
				}
			}

			uint16_t numBlockTypes = r.readU16();
			parsed.blockTypes.reserve(numBlockTypes);
			for (uint16_t i = 0; i < numBlockTypes; ++i)
				parsed.blockTypes.push_back(r.readSizedStr());

			parsed.blockTypeIndex.reserve(numBlocks);
			for (uint32_t i = 0; i < numBlocks; ++i)
				parsed.blockTypeIndex.push_back(r.readU16());

			std::vector<uint32_t> blockSizes;
			blockSizes.reserve(numBlocks);
			for (uint32_t i = 0; i < numBlocks; ++i)
				blockSizes.push_back(r.readU32());

			uint32_t numStrings = r.readU32();
			if (numStrings > 100000u)
				return std::nullopt;
			/* maxStringLen = */ r.readU32();
			parsed.strings.reserve(numStrings);
			for (uint32_t i = 0; i < numStrings; ++i)
				parsed.strings.push_back(r.readSizedStr());

			uint32_t numGroups = r.readU32();
			if (numGroups > 100000u)
				return std::nullopt;
			parsed.groups.reserve(numGroups);
			for (uint32_t i = 0; i < numGroups; ++i)
				parsed.groups.push_back(r.readU32());

			parsed.blocks.reserve(numBlocks);
			for (uint32_t i = 0; i < numBlocks; ++i) {
				uint32_t sz = blockSizes[i];
				if (!r.canRead(sz))
					return std::nullopt;
				std::vector<uint8_t> block(sz);
				if (sz) {
					std::memcpy(block.data(), data.data() + r.pos(), sz);
					r.skip(sz);
				}
				parsed.blocks.push_back(std::move(block));
			}

			return parsed;
		} catch (...) {
			return std::nullopt;
		}
	}

	void appendU8(std::vector<uint8_t>& out, uint8_t v)
	{
		out.push_back(v);
	}

	void appendU16(std::vector<uint8_t>& out, uint16_t v)
	{
		std::array<uint8_t, 2> b{};
		std::memcpy(b.data(), &v, 2);
		out.insert(out.end(), b.begin(), b.end());
	}

	void appendU32(std::vector<uint8_t>& out, uint32_t v)
	{
		std::array<uint8_t, 4> b{};
		std::memcpy(b.data(), &v, 4);
		out.insert(out.end(), b.begin(), b.end());
	}

	void appendU64(std::vector<uint8_t>& out, uint64_t v)
	{
		std::array<uint8_t, 8> b{};
		std::memcpy(b.data(), &v, 8);
		out.insert(out.end(), b.begin(), b.end());
	}

	void appendF32(std::vector<uint8_t>& out, float v)
	{
		std::array<uint8_t, 4> b{};
		std::memcpy(b.data(), &v, 4);
		out.insert(out.end(), b.begin(), b.end());
	}

	static void appendSizedStr(std::vector<uint8_t>& out, const std::string& s)
	{
		appendU32(out, static_cast<uint32_t>(s.size()));
		out.insert(out.end(), s.begin(), s.end());
	}

	static void appendShortSizedStr(std::vector<uint8_t>& out, const std::string& s)
	{
		if (s.size() > 255)
			throw std::runtime_error("NIF improver: short string too long");
		appendU8(out, static_cast<uint8_t>(s.size()));
		out.insert(out.end(), s.begin(), s.end());
	}

	bool writeNifFile(const ParsedNif& parsed, const std::string& dstPath)
	{
		try {
			std::vector<uint8_t> out;
			out.reserve(parsed.headerPrefix.size() + 1024);

			out.insert(out.end(), parsed.headerPrefix.begin(), parsed.headerPrefix.end());

			appendU32(out, parsed.version);
			appendU8(out, parsed.endianness);
			appendU32(out, parsed.userVersion);
			appendU32(out, static_cast<uint32_t>(parsed.blocks.size()));

			if (parsed.userVersion >= 10) {
				appendU32(out, parsed.bsVersion);
				appendShortSizedStr(out, parsed.author);
				if (parsed.bsVersion >= 130) {
					appendShortSizedStr(out, parsed.processScript);
					appendShortSizedStr(out, parsed.exportScript);
				} else {
					appendU32(out, parsed.legacyBsReservedU32);
				}
			}

			appendU16(out, static_cast<uint16_t>(parsed.blockTypes.size()));
			for (const auto& t : parsed.blockTypes)
				appendSizedStr(out, t);

			for (uint16_t idx : parsed.blockTypeIndex)
				appendU16(out, idx);

			for (const auto& block : parsed.blocks)
				appendU32(out, static_cast<uint32_t>(block.size()));

			uint32_t maxStrLen = 0;
			for (const auto& s : parsed.strings)
				maxStrLen = std::max(maxStrLen, static_cast<uint32_t>(s.size()));

			appendU32(out, static_cast<uint32_t>(parsed.strings.size()));
			appendU32(out, maxStrLen);
			for (const auto& s : parsed.strings)
				appendSizedStr(out, s);

			appendU32(out, static_cast<uint32_t>(parsed.groups.size()));
			for (uint32_t g : parsed.groups)
				appendU32(out, g);

			for (const auto& block : parsed.blocks)
				out.insert(out.end(), block.begin(), block.end());

			std::ofstream file(std::filesystem::u8path(dstPath), std::ios::binary | std::ios::trunc);
			if (!file.is_open())
				return false;
			file.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
			return file.good();
		} catch (...) {
			return false;
		}
	}

	// Heuristic: treat every 4-byte-aligned word whose value falls in [0, numBlocks) as a
	// potential block reference. This can produce false positives (e.g. a float whose bit
	// pattern happens to encode a small integer) but is intentionally conservative: a missed
	// reference would allow unsafe removal of a live block, whereas a false positive merely
	// prevents an optional optimisation.
	std::unordered_set<int32_t> collectPotentialRefs(const std::vector<uint8_t>& blockData, int32_t numBlocks)
	{
		std::unordered_set<int32_t> refs;
		if (blockData.size() < 4)
			return refs;

		for (size_t off = 0; off + 4 <= blockData.size(); off += 4) {
			int32_t v = -1;
			std::memcpy(&v, blockData.data() + off, 4);
			if (v >= 0 && v < numBlocks)
				refs.insert(v);
		}
		return refs;
	}

	bool hasTypeName(const std::string& typeName, std::initializer_list<const char*> names)
	{
		for (const char* n : names) {
			if (typeName == n)
				return true;
		}
		return false;
	}
}