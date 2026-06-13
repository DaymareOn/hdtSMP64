#include "hdtNIFBinaryIO.h"

#include "../Utils/hdtNIFBinaryUtils.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace hdt
{
	namespace
	{
		constexpr uint32_t kVersion_10_0_1_2 = 0x0A000102u;
		constexpr uint32_t kVersion_10_1_0_0 = 0x0A010000u;
		constexpr uint32_t kVersion_20_0_0_4 = 0x14000004u;
		constexpr uint32_t kVersion_20_0_0_5 = 0x14000005u;
		constexpr uint32_t kVersion_20_2_0_7 = 0x14020007u;
		constexpr uint32_t kMaxStringLength = 4096u;
		constexpr uint16_t kMaxBlockTypes = 16384u;
		constexpr size_t kHeaderScanLimit = 256u;

		// NifSkope nif.xml: #BSSTREAMHEADER# condition (expanded):
		//   (VER == 10.0.1.2) OR
		//   ((VER == 20.2.0.7 OR VER == 20.0.0.5 OR
		//     (VER >= 10.1.0.0 AND VER <= 20.0.0.4 AND USER <= 11))
		//    AND USER >= 3)
		bool hasBSStreamHeader(uint32_t version, uint32_t userVersion)
		{
			if (version == kVersion_10_0_1_2)
				return true;
			const bool verMatch = (version == kVersion_20_2_0_7) || (version == kVersion_20_0_0_5) || (version >= kVersion_10_1_0_0 && version <= kVersion_20_0_0_4 && userVersion <= 11u);
			return verMatch && userVersion >= 3u;
		}

		uint16_t byteSwap16(uint16_t v)
		{
			return static_cast<uint16_t>((v >> 8) | (v << 8));
		}

		uint32_t byteSwap32(uint32_t v)
		{
			return ((v & 0x000000FFu) << 24) |
			       ((v & 0x0000FF00u) << 8) |
			       ((v & 0x00FF0000u) >> 8) |
			       ((v & 0xFF000000u) >> 24);
		}

		uint64_t byteSwap64(uint64_t v)
		{
			return ((v & 0x00000000000000FFull) << 56) |
			       ((v & 0x000000000000FF00ull) << 40) |
			       ((v & 0x0000000000FF0000ull) << 24) |
			       ((v & 0x00000000FF000000ull) << 8) |
			       ((v & 0x000000FF00000000ull) >> 8) |
			       ((v & 0x0000FF0000000000ull) >> 24) |
			       ((v & 0x00FF000000000000ull) >> 40) |
			       ((v & 0xFF00000000000000ull) >> 56);
		}

		void appendShortSizedStr(std::vector<uint8_t>& out, const std::string& s)
		{
			if (s.size() > 255)
				throw std::runtime_error("NIF improver: short string too long");
			appendU8(out, static_cast<uint8_t>(s.size()));
			out.insert(out.end(), s.begin(), s.end());
		}

		// Read and store BS header fields from r into parsed.
		// Caller must have already verified hasBSStreamHeader.
		void parseBSStreamHeader(NifReader& r, ParsedNif& parsed)
		{
			parsed.bsVersion = r.readU32();
			parsed.author = r.readShortSizedStr();
			if (parsed.bsVersion > 130u)
				parsed.bsUnknownInt = r.readU32();
			if (parsed.bsVersion < 131u)
				parsed.processScript = r.readShortSizedStr();
			parsed.exportScript = r.readShortSizedStr();
			if (parsed.bsVersion >= 103u && parsed.bsVersion < 170u)
				parsed.maxFilepath = r.readShortSizedStr();
			if (parsed.bsVersion >= 170u) {
				const uint8_t len = r.readU8();
				parsed.bsUnknownData = r.readBytes(len);
			}
		}

		// Advance past BS header fields without storing them.
		// Used by the round-trip validator, which only needs structural metadata.
		void skipBSStreamHeader(NifReader& r, uint32_t bsVer)
		{
			r.skipShortSizedStr();  // author
			if (bsVer > 130u)
				r.readU32();
			if (bsVer < 131u)
				r.skipShortSizedStr();
			r.skipShortSizedStr();  // exportScript
			if (bsVer >= 103u && bsVer < 170u)
				r.skipShortSizedStr();
			if (bsVer >= 170u) {
				uint8_t len = r.readU8();
				r.skip(len);
			}
		}

	}  // namespace

	// ── NifReader ─────────────────────────────────────────────────────────────

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

	void NifReader::setBigEndian(bool isBigEndian)
	{
		m_bigEndian = isBigEndian;
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
		if (m_bigEndian)
			v = byteSwap16(v);
		return v;
	}

	uint32_t NifReader::readU32()
	{
		if (!canRead(4))
			throw std::runtime_error("NIF improver: read past end (U32)");
		uint32_t v;
		std::memcpy(&v, m_data.data() + m_pos, 4);
		m_pos += 4;
		if (m_bigEndian)
			v = byteSwap32(v);
		return v;
	}

	uint64_t NifReader::readU64()
	{
		if (!canRead(8))
			throw std::runtime_error("NIF improver: read past end (U64)");
		uint64_t v;
		std::memcpy(&v, m_data.data() + m_pos, 8);
		m_pos += 8;
		if (m_bigEndian)
			v = byteSwap64(v);
		return v;
	}

	float NifReader::readF32()
	{
		if (!canRead(4))
			throw std::runtime_error("NIF improver: read past end (F32)");
		uint32_t raw;
		std::memcpy(&raw, m_data.data() + m_pos, 4);
		m_pos += 4;
		if (m_bigEndian)
			raw = byteSwap32(raw);
		float v;
		std::memcpy(&v, &raw, 4);
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

	void NifReader::skipSizedStr()
	{
		uint32_t len = readU32();
		if (len > kMaxStringLength)
			throw std::runtime_error("NIF improver: implausible string length");
		skip(len);
	}

	void NifReader::skipShortSizedStr()
	{
		skip(readU8());
	}

	// ── Serialisation helpers ─────────────────────────────────────────────────
	// Used by hdtNIFBinaryIO (serializeNif) and hdtNIFDecimator.

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

	// ── Parse ─────────────────────────────────────────────────────────────────

	std::optional<ParsedNif> parseNif(const std::vector<uint8_t>& data, std::string* outError)
	{
		auto fail = [&](const std::string& msg) -> std::optional<ParsedNif> {
			if (outError)
				*outError = msg;
			return std::nullopt;
		};

		// ── Header string ─────────────────────────────────────────────────────
		// NifSkope nifstream.cpp tHeaderString: read chars until '\n', up to 80 chars.
		size_t headerEnd = 0;
		{
			const size_t limit = std::min(data.size(), kHeaderScanLimit);
			for (size_t i = 0; i < limit; ++i) {
				if (data[i] == '\n') {
					headerEnd = i + 1;
					break;
				}
			}
			if (headerEnd == 0)
				return fail("missing NIF header newline");
		}
		{
			size_t len = headerEnd;
			while (len > 0 && (data[len - 1] == '\0' || data[len - 1] == '\n' || data[len - 1] == '\r'))
				--len;
			const std::string hdr(reinterpret_cast<const char*>(data.data()), len);
			if (hdr.find(nif::kNifHeaderMagic) == std::string::npos &&
				hdr.find(nif::kNifHeaderMagicLegacy) == std::string::npos)
				return fail("not a NIF file: '" + hdr + "'");
		}

		try {
			ParsedNif parsed;
			parsed.headerPrefix.assign(data.begin(), data.begin() + headerEnd);
			NifReader r(data, headerEnd);

			// ── Version ───────────────────────────────────────────────────────
			// NifSkope nifstream.cpp tFileVersion: read 4 bytes as raw uint32
			// (always little-endian; endianness byte comes after).
			parsed.version = r.readU32();
			if (parsed.version != kVersion_20_2_0_7)
				return fail("unsupported NIF version: 0x" + [](uint32_t v) {
					char b[9] = {};
					std::snprintf(b, sizeof(b), "%08X", v);
					return std::string(b);
				}(parsed.version));

			// ── Endian Type ───────────────────────────────────────────────────
			// nif.xml: <field name="Endian Type" since="20.0.0.3">
			// NifSkope tFileVersion: if version >= 0x14000004, peek next byte to
			// set endianness; the byte is then consumed as the "Endian Type" field.
			// No guessing: presence is determined entirely by the version.
			parsed.hasExplicitEndiannessByte = (parsed.version >= kVersion_20_0_0_4);
			if (parsed.hasExplicitEndiannessByte) {
				parsed.endianness = r.readU8();  // 0 = big, 1 = little
				if (parsed.endianness == 0)
					r.setBigEndian(true);
				else if (parsed.endianness == 1)
					r.setBigEndian(false);
				else
					return fail("invalid endianness byte: " + std::to_string(parsed.endianness));
			}

			// ── User Version ──────────────────────────────────────────────────
			// nif.xml: ulittle32, since 10.0.1.8
			parsed.userVersion = r.readU32();

			// ── Num Blocks ────────────────────────────────────────────────────
			// nif.xml: ulittle32, since 3.1.0.1
			const uint32_t numBlocks = r.readU32();
			if (numBlocks > nif::kMaxBlocks)
				return fail("implausible block count: " + std::to_string(numBlocks));

			// ── BS Header ─────────────────────────────────────────────────────
			// nif.xml: <field name="BS Header" type="BSStreamHeader"
			//               cond="#BSSTREAMHEADER#"/>
			// BSStreamHeader fields (verbatim from nif.xml):
			//   BS Version      ulittle32          — always
			//   Author          ExportString       — always
			//   Unknown Int     uint  cond: BS Version > 130
			//   Process Script  ExportString  cond: BS Version < 131
			//   Export Script   ExportString       — always
			//   Max Filepath    ExportString  cond: BS Version >= 103 && < 170
			//   Unknown Data    ExportDataSF  cond: BS Version >= 170
			// ExportString = 1-byte length + that many chars (null counted in length).
			// ExportDataSF = 1-byte length + that many bytes.
			if (hasBSStreamHeader(parsed.version, parsed.userVersion))
				parseBSStreamHeader(r, parsed);

			// ── Num Block Types ───────────────────────────────────────────────
			// nif.xml: ushort, since 5.0.0.1
			const uint16_t numBlockTypes = r.readU16();
			if (numBlockTypes > kMaxBlockTypes)
				return fail("implausible block type count: " + std::to_string(numBlockTypes));
			parsed.blockTypes.reserve(numBlockTypes);
			for (uint16_t i = 0; i < numBlockTypes; ++i)
				parsed.blockTypes.push_back(r.readSizedStr());

			// ── Block Type Index ──────────────────────────────────────────────
			// nif.xml: BlockTypeIndex (ushort), length Num Blocks, since 5.0.0.1
			// NifSkope masks upper bit (0x8000 = PhysX flag) when looking up type.
			parsed.blockTypeIndex.reserve(numBlocks);
			for (uint32_t i = 0; i < numBlocks; ++i)
				parsed.blockTypeIndex.push_back(r.readU16());

			// ── Block Sizes ───────────────────────────────────────────────────
			// nif.xml: uint, length Num Blocks, since 20.2.0.5
			std::vector<uint32_t> blockSizes;
			blockSizes.reserve(numBlocks);
			for (uint32_t i = 0; i < numBlocks; ++i)
				blockSizes.push_back(r.readU32());

			// ── Strings ───────────────────────────────────────────────────────
			// nif.xml: Num Strings (uint) + Max String Length (uint) +
			//          Strings (SizedString[Num Strings]), since 20.1.0.1
			// SizedString = 4-byte LE length + that many chars.
			const uint32_t numStrings = r.readU32();
			if (numStrings > nif::kMaxStrings)
				return fail("implausible string count: " + std::to_string(numStrings));
			/* maxStringLen = */ r.readU32();
			parsed.strings.reserve(numStrings);
			for (uint32_t i = 0; i < numStrings; ++i)
				parsed.strings.push_back(r.readSizedStr());

			// ── Groups ────────────────────────────────────────────────────────
			// nif.xml: Num Groups (uint) + Groups (uint[Num Groups]), since 5.0.0.6
			const uint32_t numGroups = r.readU32();
			if (numGroups > nif::kMaxBlocks)
				return fail("implausible group count: " + std::to_string(numGroups));
			parsed.groups.reserve(numGroups);
			for (uint32_t i = 0; i < numGroups; ++i)
				parsed.groups.push_back(r.readU32());

			// ── Block data ────────────────────────────────────────────────────
			// NifSkope: reads each block with loadItem(), then seeks to
			// curpos + blockSize[c] to correct for any over/under-read.
			// We store raw bytes per the sizes declared in the header.
			parsed.blocks.reserve(numBlocks);
			for (uint32_t i = 0; i < numBlocks; ++i) {
				const uint32_t sz = blockSizes[i];
				if (!r.canRead(sz))
					return fail("block data out of bounds at block " + std::to_string(i));
				parsed.blocks.emplace_back(data.data() + r.pos(), data.data() + r.pos() + sz);
				r.skip(sz);
			}

			// ── Footer ────────────────────────────────────────────────────────
			// nif.xml Footer: Num Roots (uint) + Roots (Ref[Num Roots]), since 3.3.0.13
			// NifSkope: loadItem(getFooterItem(), stream)
			if (r.canRead(4)) {
				const uint32_t numRoots = r.readU32();
				if (numRoots <= nif::kMaxBlocks) {
					parsed.footerRoots.reserve(numRoots);
					for (uint32_t i = 0; i < numRoots; ++i) {
						if (!r.canRead(4))
							break;
						parsed.footerRoots.push_back(static_cast<int32_t>(r.readU32()));
					}
				}
			}

			return parsed;
		} catch (const std::exception& e) {
			return fail(std::string("parse exception: ") + e.what());
		} catch (...) {
			return fail("parse exception: unknown");
		}
	}

	// ── Serialize ─────────────────────────────────────────────────────────────

	std::vector<uint8_t> serializeNif(const ParsedNif& parsed)
	{
		// Pre-compute the exact output size to avoid any reallocation.
		// Without this, an 8 MB NIF triggers ~13 doubling reallocations under
		// 32-thread contention — each a malloc+memcpy+free on the global heap.
		size_t expectedSize = parsed.headerPrefix.size() + 4                // version
		                      + (parsed.hasExplicitEndiannessByte ? 1 : 0)  // endianness
		                      + 4 + 4;                                      // userVersion + numBlocks
		if (hasBSStreamHeader(parsed.version, parsed.userVersion)) {
			// Mirrors parseBSStreamHeader field-by-field.
			expectedSize += 4 + 1 + parsed.author.size();  // bsVersion + author
			if (parsed.bsVersion > 130u)
				expectedSize += 4;
			if (parsed.bsVersion < 131u)
				expectedSize += 1 + parsed.processScript.size();
			expectedSize += 1 + parsed.exportScript.size();
			if (parsed.bsVersion >= 103u && parsed.bsVersion < 170u)
				expectedSize += 1 + parsed.maxFilepath.size();
			if (parsed.bsVersion >= 170u)
				expectedSize += 1 + parsed.bsUnknownData.size();
		}
		expectedSize += 2;  // numBlockTypes
		for (const auto& t : parsed.blockTypes) expectedSize += 4 + t.size();
		expectedSize += 2 * parsed.blockTypeIndex.size();  // block type index
		expectedSize += 4 * parsed.blocks.size();          // block sizes
		expectedSize += 4 + 4;                             // numStrings + maxStringLen
		for (const auto& s : parsed.strings) expectedSize += 4 + s.size();
		expectedSize += 4 + 4 * parsed.groups.size();
		for (const auto& b : parsed.blocks) expectedSize += b.size();
		expectedSize += 4 + 4 * parsed.footerRoots.size();

		std::vector<uint8_t> out;
		out.reserve(expectedSize);
		const bool writeBigEndianPayload = parsed.hasExplicitEndiannessByte && parsed.endianness == 0;

		auto appendU16Endian = [&](uint16_t v) {
			if (writeBigEndianPayload)
				v = byteSwap16(v);
			appendU16(out, v);
		};
		auto appendU32Endian = [&](uint32_t v) {
			if (writeBigEndianPayload)
				v = byteSwap32(v);
			appendU32(out, v);
		};
		auto appendSizedStrEndian = [&](const std::string& s) {
			appendU32Endian(static_cast<uint32_t>(s.size()));
			out.insert(out.end(), s.begin(), s.end());
		};

		out.insert(out.end(), parsed.headerPrefix.begin(), parsed.headerPrefix.end());

		// Version: always little-endian (NifSkope tFileVersion reads raw, not via dataStream)
		appendU32(out, parsed.version);
		if (parsed.hasExplicitEndiannessByte)
			appendU8(out, parsed.endianness);
		appendU32Endian(parsed.userVersion);
		appendU32Endian(static_cast<uint32_t>(parsed.blocks.size()));

		// BS Header: exact mirror of parseBSStreamHeader — same hasBSStreamHeader condition,
		// same per-field bsVersion conditions (verbatim from nif.xml BSStreamHeader).
		if (hasBSStreamHeader(parsed.version, parsed.userVersion)) {
			appendU32Endian(parsed.bsVersion);
			appendShortSizedStr(out, parsed.author);
			if (parsed.bsVersion > 130u)
				appendU32Endian(parsed.bsUnknownInt);
			if (parsed.bsVersion < 131u)
				appendShortSizedStr(out, parsed.processScript);
			appendShortSizedStr(out, parsed.exportScript);
			if (parsed.bsVersion >= 103u && parsed.bsVersion < 170u)
				appendShortSizedStr(out, parsed.maxFilepath);
			if (parsed.bsVersion >= 170u) {
				if (parsed.bsUnknownData.size() > 255)
					throw std::runtime_error("NIF: bsUnknownData too long for ExportDataSF");
				appendU8(out, static_cast<uint8_t>(parsed.bsUnknownData.size()));
				out.insert(out.end(), parsed.bsUnknownData.begin(), parsed.bsUnknownData.end());
			}
		}

		appendU16Endian(static_cast<uint16_t>(parsed.blockTypes.size()));
		for (const auto& t : parsed.blockTypes)
			appendSizedStrEndian(t);

		for (uint16_t idx : parsed.blockTypeIndex)
			appendU16Endian(idx);

		for (const auto& block : parsed.blocks)
			appendU32Endian(static_cast<uint32_t>(block.size()));

		uint32_t maxStrLen = 0;
		for (const auto& s : parsed.strings)
			maxStrLen = std::max(maxStrLen, static_cast<uint32_t>(s.size()));

		appendU32Endian(static_cast<uint32_t>(parsed.strings.size()));
		appendU32Endian(maxStrLen);
		for (const auto& s : parsed.strings)
			appendSizedStrEndian(s);

		appendU32Endian(static_cast<uint32_t>(parsed.groups.size()));
		for (uint32_t g : parsed.groups)
			appendU32Endian(g);

		for (const auto& block : parsed.blocks)
			out.insert(out.end(), block.begin(), block.end());

		// Footer: Num Roots + Roots[]
		appendU32Endian(static_cast<uint32_t>(parsed.footerRoots.size()));
		for (int32_t root : parsed.footerRoots)
			appendU32Endian(static_cast<uint32_t>(root));

		return out;
	}

	// ── Write ─────────────────────────────────────────────────────────────────

	bool writeNifBytes(const std::vector<uint8_t>& bytes, const std::string& dstPath)
	{
		try {
			std::ofstream file(std::filesystem::u8path(dstPath), std::ios::binary | std::ios::trunc);
			if (!file.is_open())
				return false;
			file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
			return file.good();
		} catch (...) {
			return false;
		}
	}

	bool writeNifFile(const ParsedNif& parsed, const std::string& dstPath)
	{
		return writeNifBytes(serializeNif(parsed), dstPath);
	}

	// ── Validation ────────────────────────────────────────────────────────────

	// Lightweight structural check: parse only the header metadata and compare
	// block counts/sizes — never touch block data, so no per-block allocation.
	// Full parseNif on an 8 MB NIF allocates 200+ vectors under 32-thread
	// contention; this path allocates ~3 small vectors instead.
	std::optional<std::string> validateNifRoundTripFromBytes(
		const std::vector<uint8_t>& bytes, const ParsedNif& parsed)
	{
		try {
			NifReader r(bytes, 0);

			// Skip header line (up to first '\n')
			{
				bool found = false;
				for (size_t i = 0; i < std::min(bytes.size(), kHeaderScanLimit); ++i) {
					if (bytes[i] == '\n') {
						r.skip(i + 1);
						found = true;
						break;
					}
				}
				if (!found)
					return "re-parse: missing header newline";
			}

			uint32_t version = r.readU32();
			if (version != parsed.version)
				return "re-parse: version mismatch";

			if (parsed.hasExplicitEndiannessByte)
				r.setBigEndian(r.readU8() == 0);

			r.readU32();  // userVersion
			uint32_t numBlocks = r.readU32();
			if (numBlocks != static_cast<uint32_t>(parsed.blocks.size()))
				return "block count mismatch: expected " + std::to_string(parsed.blocks.size()) +
				       ", got " + std::to_string(numBlocks);

			if (hasBSStreamHeader(version, parsed.userVersion)) {
				uint32_t bsVer = r.readU32();
				skipBSStreamHeader(r, bsVer);
			}

			// Skip block type names; retain the count for index bounds-checking below.
			uint16_t numBlockTypes = r.readU16();
			for (uint16_t i = 0; i < numBlockTypes; ++i)
				r.skipSizedStr();

			// Read and validate block type index
			if (numBlocks != static_cast<uint32_t>(parsed.blockTypeIndex.size()))
				return "blockTypeIndex size mismatch";
			for (uint32_t i = 0; i < numBlocks; ++i) {
				uint16_t idx = r.readU16();
				if (idx >= numBlockTypes)
					return "block " + std::to_string(i) + " has out-of-range type index " +
					       std::to_string(idx);
			}

			// Read and validate block sizes
			for (uint32_t i = 0; i < numBlocks; ++i) {
				uint32_t sz = r.readU32();
				if (sz != static_cast<uint32_t>(parsed.blocks[i].size()))
					return "block " + std::to_string(i) + " size mismatch: expected " +
					       std::to_string(parsed.blocks[i].size()) + ", got " +
					       std::to_string(sz);
			}

			return std::nullopt;
		} catch (const std::exception& e) {
			return std::string("exception during round-trip: ") + e.what();
		} catch (...) {
			return "unknown exception during round-trip";
		}
	}

	std::optional<std::string> validateNifRoundTrip(const ParsedNif& parsed)
	{
		try {
			return validateNifRoundTripFromBytes(serializeNif(parsed), parsed);
		} catch (const std::exception& e) {
			return std::string("exception during round-trip: ") + e.what();
		} catch (...) {
			return "unknown exception during round-trip";
		}
	}

	// ── Utilities ─────────────────────────────────────────────────────────────

	bool hasTypeName(const std::string& typeName, std::initializer_list<const char*> names)
	{
		for (const char* n : names) {
			if (typeName == n)
				return true;
		}
		return false;
	}

	bool isPreSESkyrimNif(const ParsedNif& parsed)
	{
		return parsed.bsVersion != 0 && parsed.bsVersion < kSkyrimSEBsVersion;
	}
}
