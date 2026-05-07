#include "hdtNIFImprover.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
	// The "remove bogus nodes" algorithm (removing orphan NiNode blocks with
	// no children and no inbound references) is inspired by the spRemoveBogusNodes
	// spell in NifSkope (fo76utils/nifskope), copyright (c) 2005–2014, NIF File
	// Format Library and Tools, under the NifSkope 3-clause BSD license.
	// This is an independent clean-room reimplementation; no NifSkope source
	// code is copied or derived here.

	static constexpr uint32_t kVersion_20_2_0_7 = 0x14020007u;
	static constexpr uint32_t kMaxStringLength = 4096u;
	static constexpr std::streamoff kMaxNifFileSizeBytes = 256 * 1024 * 1024;

	class NifReader
	{
	public:
		NifReader(const std::vector<uint8_t>& data, size_t pos = 0) :
			m_data(data), m_pos(pos) {}

		bool canRead(size_t bytes) const { return m_pos + bytes <= m_data.size(); }
		size_t pos() const { return m_pos; }

		void skip(size_t bytes)
		{
			if (!canRead(bytes))
				throw std::runtime_error("NIF improver: skip past end");
			m_pos += bytes;
		}

		uint8_t readU8()
		{
			if (!canRead(1))
				throw std::runtime_error("NIF improver: read past end (U8)");
			return m_data[m_pos++];
		}

		uint16_t readU16()
		{
			if (!canRead(2))
				throw std::runtime_error("NIF improver: read past end (U16)");
			uint16_t v;
			std::memcpy(&v, m_data.data() + m_pos, 2);
			m_pos += 2;
			return v;
		}

		uint32_t readU32()
		{
			if (!canRead(4))
				throw std::runtime_error("NIF improver: read past end (U32)");
			uint32_t v;
			std::memcpy(&v, m_data.data() + m_pos, 4);
			m_pos += 4;
			return v;
		}

		std::string readSizedStr()
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

		std::string readShortSizedStr()
		{
			uint8_t len = readU8();
			if (!canRead(len))
				throw std::runtime_error("NIF improver: short string out of bounds");
			std::string s(reinterpret_cast<const char*>(m_data.data() + m_pos), len);
			m_pos += len;
			return s;
		}

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

	static std::optional<size_t> findHeaderEnd(const std::vector<uint8_t>& data)
	{
		size_t limit = std::min<size_t>(data.size(), 200);
		for (size_t i = 0; i < limit; ++i) {
			if (data[i] == 0x00)
				return i + 1;
		}
		return std::nullopt;
	}

	static std::optional<ParsedNif> parseNif(const std::vector<uint8_t>& data)
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

	static void appendU8(std::vector<uint8_t>& out, uint8_t v)
	{
		out.push_back(v);
	}

	static void appendU16(std::vector<uint8_t>& out, uint16_t v)
	{
		std::array<uint8_t, 2> b{};
		std::memcpy(b.data(), &v, 2);
		out.insert(out.end(), b.begin(), b.end());
	}

	static void appendU32(std::vector<uint8_t>& out, uint32_t v)
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

	static bool writeNifFile(const ParsedNif& parsed, const std::string& dstPath)
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

			std::ofstream file(dstPath, std::ios::binary | std::ios::trunc);
			if (!file.is_open())
				return false;
			file.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
			return file.good();
		} catch (...) {
			return false;
		}
	}

	static std::string stripDataPrefix(const std::string& path)
	{
		std::string norm = path;
		std::replace(norm.begin(), norm.end(), '\\', '/');

		std::string lower = norm;
		std::transform(lower.begin(), lower.end(), lower.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });

		if (lower.size() >= 5 && lower.substr(0, 5) == "data/")
			return norm.substr(5);
		return norm;
	}

	static std::unordered_set<int32_t> collectPotentialRefs(const std::vector<uint8_t>& blockData, int32_t numBlocks)
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

	static bool isImportantNodeName(const std::string& name, uint32_t bsVersion)
	{
		if (name == "ProjectileNode")
			return true;
		if (bsVersion < 83 && (name == "ShellCasingNode" || name == "##SightingNode"))
			return true;
		if (bsVersion >= 130 && name == "WorkshopConnectPoints")
			return true;
		return false;
	}

	static bool hasTypeName(const std::string& typeName, const std::initializer_list<const char*>& names)
	{
		for (const char* n : names) {
			if (typeName == n)
				return true;
		}
		return false;
	}

	bool GenerateImprovedNIF(
		const std::string& srcNIFPath,
		const std::string& outputDir,
		const NIFDecimationOptions& options)
	{
		namespace fs = std::filesystem;
		static std::atomic<bool> s_warnedDecimationUnsupported{ false };

		if (options.enableCollisionMeshDecimation && !s_warnedDecimationUnsupported.exchange(true)) {
			logger::warn(
				"[Validator] Offline collision mesh decimation is enabled, but this NIF improver pass currently performs conservative structural cleanup only.");
		}

		std::ifstream in(srcNIFPath, std::ios::binary | std::ios::ate);
		if (!in.is_open())
			return false;
		auto sz = in.tellg();
		if (sz <= 0 || sz > kMaxNifFileSizeBytes)
			return false;
		std::vector<uint8_t> data(static_cast<size_t>(sz));
		in.seekg(0);
		in.read(reinterpret_cast<char*>(data.data()), sz);
		if (!in.good() && !in.eof())
			return false;

		auto parsedOpt = parseNif(data);
		if (!parsedOpt.has_value())
			return false;
		auto& parsed = *parsedOpt;

		if (!parsed.groups.empty()) {
			// Conservatively skip grouped NIFs in this first implementation.
			return false;
		}

		bool changed = false;

		for (;;) {
			if (parsed.blocks.empty())
				break;

			const int32_t numBlocks = static_cast<int32_t>(parsed.blocks.size());
			const int32_t tailIdx = numBlocks - 1;
			if (tailIdx < 0 || tailIdx >= static_cast<int32_t>(parsed.blockTypeIndex.size()))
				break;

			uint16_t tIdx = parsed.blockTypeIndex[static_cast<size_t>(tailIdx)];
			if (tIdx >= parsed.blockTypes.size())
				break;
			const std::string& typeName = parsed.blockTypes[tIdx];

			bool isSpecialNodeClass = hasTypeName(typeName, { "BSBlastNode", "BSDamageStage", "BSDebrisNode", "BSValueNode" });
			if (typeName != "NiNode" && !isSpecialNodeClass)
				break;

			// Preserve special node classes by type name.
			if (isSpecialNodeClass)
				break;

			// Conservative outbound references: every aligned int32 in range.
			std::vector<std::unordered_set<int32_t>> outbound(parsed.blocks.size());
			for (size_t i = 0; i < parsed.blocks.size(); ++i)
				outbound[i] = collectPotentialRefs(parsed.blocks[i], numBlocks);

			// Node name index is the first int32 of NiObjectNET-derived blocks.
			std::string nodeName;
			const auto& tailBlock = parsed.blocks.back();
			if (tailBlock.size() >= 4) {
				int32_t nameIdx = -1;
				std::memcpy(&nameIdx, tailBlock.data(), 4);
				if (nameIdx >= 0 && nameIdx < static_cast<int32_t>(parsed.strings.size()))
					nodeName = parsed.strings[static_cast<size_t>(nameIdx)];
			}

			if (isImportantNodeName(nodeName, parsed.bsVersion))
				break;

			// tail is removable only when it has no potential outbound refs
			// and no inbound refs from any other block.
			if (!outbound.back().empty())
				break;

			bool hasInbound = false;
			for (size_t i = 0; i + 1 < outbound.size(); ++i) {
				if (outbound[i].count(tailIdx)) {
					hasInbound = true;
					break;
				}
			}
			if (hasInbound)
				break;

			parsed.blocks.pop_back();
			parsed.blockTypeIndex.pop_back();
			changed = true;
		}

		if (!changed)
			return false;

		std::string relative = stripDataPrefix(srcNIFPath);
		fs::path outPath = fs::path(outputDir) / relative;
		std::error_code ec;
		fs::create_directories(outPath.parent_path(), ec);
		if (ec)
			return false;

		return writeNifFile(parsed, outPath.string());
	}
}
