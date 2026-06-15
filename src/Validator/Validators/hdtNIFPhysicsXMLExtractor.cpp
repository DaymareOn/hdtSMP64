#include "hdtNIFValidator.h"

#include "../Parser/hdtNIFBinaryParser.h"
#include "../Utils/hdtNIFBinaryUtils.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
	namespace
	{
		// ── Helpers ───────────────────────────────────────────────────────────────

		// Scan raw NIF bytes for XML-like path strings ending in ".xml".
		// Used as a fallback when the structured parser fails or finds no XML refs.
		// Accepts paths of 8–260 bytes that contain at least one path separator.
		std::vector<std::string> extractXmlPathsFromRawBytes(const std::vector<uint8_t>& data)
		{
			std::vector<std::string> paths;
			std::unordered_set<std::string> seen;

			auto isPathByte = [](uint8_t ch) {
				if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9'))
					return true;
				switch (ch) {
				case '/':
				case '\\':
				case '_':
				case '-':
				case '.':
				case ' ':
				case '[':
				case ']':
				case '(':
				case ')':
				case '\'':
					return true;
				default:
					return false;
				}
			};

			for (size_t i = 0; i + 4 <= data.size(); ++i) {
				char c0 = static_cast<char>(std::tolower(data[i]));
				char c1 = static_cast<char>(std::tolower(data[i + 1]));
				char c2 = static_cast<char>(std::tolower(data[i + 2]));
				char c3 = static_cast<char>(std::tolower(data[i + 3]));
				if (!(c0 == '.' && c1 == 'x' && c2 == 'm' && c3 == 'l'))
					continue;

				size_t begin = i;
				while (begin > 0 && isPathByte(data[begin - 1]))
					--begin;

				// The path ends at its ".xml" extension; don't scan past it. NIF
				// string-table entries are length-prefixed (not null-terminated), so the
				// byte right after ".xml" is the next entry's length prefix, whose low
				// byte can be a path-valid character (e.g. 0x31 = '1') that would
				// otherwise be wrongly appended to the path.
				const size_t end = i + 4;

				if (end <= begin)
					continue;

				std::string candidate(reinterpret_cast<const char*>(data.data() + begin), end - begin);
				if (candidate.size() < 8 || candidate.size() > 260)
					continue;
				if (candidate.find('/') == std::string::npos && candidate.find('\\') == std::string::npos)
					continue;

				std::replace(candidate.begin(), candidate.end(), '\\', '/');
				if (!candidate.empty() && candidate[0] == '/')
					candidate.erase(candidate.begin());

				if (seen.insert(candidate).second)
					paths.push_back(std::move(candidate));
			}

			return paths;
		}

		// Populate result with raw-byte fallback XML paths when structured parsing
		// failed or returned no refs.  Sets hasPhysicsData when paths are found.
		void applyFallbackPaths(NIFScanResult& result, const std::vector<uint8_t>& data)
		{
			auto paths = extractXmlPathsFromRawBytes(data);
			if (paths.empty())
				return;
			result.hasPhysicsData = true;
			result.allPhysicsXmlPaths = std::move(paths);
			result.physicsXmlPath = result.allPhysicsXmlPaths.front();
		}

	}  // namespace

	// ── Public API ────────────────────────────────────────────────────────────

	// Returns hasPhysicsData=false when no physics marker is present.
	// Structured parse failures fall back to a raw-byte XML path scan.
	NIFScanResult ExtractPhysicsXmlRefsFromNIFs(const std::string& nifPath)
	{
		NIFScanResult result;

		// ── File I/O ──────────────────────────────────────────────────────────

		std::ifstream file(std::filesystem::u8path(nifPath), std::ios::binary | std::ios::ate);
		if (!file.is_open()) {
			result.errors.push_back("Cannot open: " + nifPath);
			return result;
		}

		auto fileSize = file.tellg();
		if (fileSize <= 0 || static_cast<size_t>(fileSize) > nif::kMaxNifFileSize) {
			result.errors.push_back(fileSize <= 0 ? "File is empty or unreadable: " + nifPath : "File exceeds max supported size: " + nifPath);
			return result;
		}

		const size_t totalSize = static_cast<size_t>(fileSize);

		// ── Physics marker probe ──────────────────────────────────────────────

		// Read a prefix to check for the physics marker before reading the full file.
		// The marker lives in the NIF string table, which starts at roughly
		// 6 × numBlocks + ~2 KB into the file. For non-physics NIFs the pre-string-
		// table area contains only binary integers, so the ASCII marker can't appear
		// there by coincidence — the probe always gives a correct negative result
		// regardless of NIF size. Only a physics NIF with >5 100 blocks would require
		// more than 32 KB, and such a NIF doesn't exist in practice (physics NIFs are
		// simple skinned meshes with <500 blocks).
		// ~93% of NIFs have no physics data; this avoids reading their full content.
		static constexpr size_t kMarkerProbeSize = 32 * 1024;
		const size_t probeSize = std::min(totalSize, kMarkerProbeSize);

		std::vector<uint8_t> data(probeSize);
		file.seekg(0);
		file.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(probeSize));

		if (!nif::ContainsAsciiSequence(data.data(), data.size(), nif::kPhysicsMarker))
			return result;  // not a physics NIF — skip reading the rest of the file

		// ── Full read ─────────────────────────────────────────────────────────

		if (probeSize < totalSize) {
			data.resize(totalSize);
			file.seekg(static_cast<std::streamoff>(probeSize));
			file.read(reinterpret_cast<char*>(data.data() + probeSize),
				static_cast<std::streamsize>(totalSize - probeSize));
		}
		file.close();

		// ── Header validation ─────────────────────────────────────────────────

		// Header text can be LF/CRLF-terminated, sometimes followed by optional NUL bytes.
		// Do not use the first NUL as header boundary: that can be inside binary fields.
		size_t headerEnd = 0;
		for (size_t i = 0; i < std::min(data.size(), nif::kHeaderProbeLimit); ++i) {
			if (data[i] == '\n') {
				headerEnd = i;
				break;
			}
		}
		if (headerEnd == 0) {
			for (size_t i = 0; i < std::min(data.size(), nif::kHeaderProbeLimit); ++i) {
				if (data[i] == 0x00) {
					headerEnd = i;
					break;
				}
			}
		}
		if (headerEnd == 0) {
			result.errors.push_back("NIF header terminator not found: " + nifPath);
			applyFallbackPaths(result, data);
			return result;
		}

		std::string headerStr(reinterpret_cast<const char*>(data.data()), headerEnd);
		if (!headerStr.empty() && headerStr.back() == '\r')
			headerStr.pop_back();

		const bool hasKnownMagic =
			headerStr.find(nif::kNifHeaderMagic) != std::string::npos ||
			headerStr.find(nif::kNifHeaderMagicLegacy) != std::string::npos;
		if (!hasKnownMagic) {
			result.errors.push_back("Missing NIF header magic: " + nifPath);
			applyFallbackPaths(result, data);
			return result;
		}

		// ── Parse and extract ─────────────────────────────────────────────────

		try {
			auto parsedOpt = parseNif(data);
			if (parsedOpt.has_value()) {
				const auto& parsed = *parsedOpt;

				for (size_t i = 0; i < parsed.blockTypeIndex.size(); ++i) {
					uint16_t tIdx = parsed.blockTypeIndex[i];
					// Mask off PhysX high-bit (0x8000) before bounds-checking
					uint16_t masked = tIdx & 0x7FFF;
					if (masked >= parsed.blockTypes.size())
						continue;
					const auto& bt = parsed.blockTypes[masked];
					if (bt == nif::kTypeBSTriShape || bt == nif::kTypeBSDynamicTriShape)
						result.hasGeometry = true;
					else if (bt == nif::kTypeNiSkinInstance || bt == nif::kTypeBSSkinInstance)
						result.hasSkinning = true;
				}

				bool hasMarker = false;
				for (const auto& s : parsed.strings) {
					if (s == nif::kPhysicsMarker) {
						hasMarker = true;
						break;
					}
				}

				if (hasMarker) {
					result.hasPhysicsData = true;
					result.allPhysicsXmlPaths = nif::FindXmlPathsInNif(parsed);
					if (!result.allPhysicsXmlPaths.empty()) {
						result.physicsXmlPath = result.allPhysicsXmlPaths[0];
					} else {
						// The marker string is present but no NiStringExtraData block
						// references it — a leftover/orphaned marker (e.g. the physics
						// extra-data block was stripped, leaving its strings in the table).
						// The NIF has no functional physics; flag it for a warning rather
						// than treating its stale string-table paths as real references.
						result.hasOrphanedPhysicsMarker = true;
					}
				}
				return result;
			}
			// parseNif returned no value — fall through to raw-byte fallback
		} catch (...) {
			// Parse error — fall through to raw-byte fallback
		}

		applyFallbackPaths(result, data);
		return result;
	}

}  // namespace hdt
