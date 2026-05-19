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
	/// Scans a NIF file to find FSMP physics data and referenced XML config paths.
	/// Performs strict header-based parsing and reports scan/parse failures in result.errors.
	/// Returns a default result with hasPhysicsData=false when no physics marker exists.
	NIFScanResult ExtractPhysicsXmlRefsFromNIFs(const std::string& nifPath)
	{
		NIFScanResult result;

		std::ifstream file(std::filesystem::u8path(nifPath), std::ios::binary | std::ios::ate);
		if (!file.is_open()) {
			result.errors.push_back("Cannot open: " + nifPath);
			return result;
		}

		auto fileSize = file.tellg();
		if (fileSize <= 0 || static_cast<size_t>(fileSize) > nif::kMaxNifFileSize) {
			if (fileSize <= 0)
				result.errors.push_back("File is empty or unreadable: " + nifPath);
			else
				result.errors.push_back("File exceeds max supported size: " + nifPath);
			return result;
		}

		const size_t totalSize = static_cast<size_t>(fileSize);

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

		auto containsAscii = [&](const char* needle) {
			const size_t nlen = std::strlen(needle);
			if (nlen == 0 || data.size() < nlen)
				return false;

			for (size_t i = 0; i + nlen <= data.size(); ++i) {
				if (std::memcmp(data.data() + i, needle, nlen) == 0)
					return true;
			}

			return false;
		};

		const bool maybePhysicsNif = containsAscii(nif::kPhysicsMarker);

		if (!maybePhysicsNif) {
			return result;  // not a physics NIF — skip reading the rest of the file
		}

		// Physics marker found — read the full file for proper parsing.
		if (probeSize < totalSize) {
			data.resize(totalSize);
			file.seekg(static_cast<std::streamoff>(probeSize));
			file.read(reinterpret_cast<char*>(data.data() + probeSize),
			          static_cast<std::streamsize>(totalSize - probeSize));
		}
		file.close();

		auto extractXmlLikePathsFromRawBytes = [&]() {
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

				size_t end = i + 4;
				while (end < data.size() && isPathByte(data[end]))
					++end;

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
		};

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
			if (maybePhysicsNif)
				result.errors.push_back("NIF header terminator not found: " + nifPath);
			return result;
		}

		std::string headerStr(reinterpret_cast<const char*>(data.data()), headerEnd);
		if (!headerStr.empty() && headerStr.back() == '\r')
			headerStr.pop_back();

		const bool hasKnownMagic =
			headerStr.find(nif::kNifHeaderMagic) != std::string::npos ||
			headerStr.find("NetImmerse File Format") != std::string::npos;
		if (!hasKnownMagic) {
			if (maybePhysicsNif)
				result.errors.push_back("Missing NIF header magic: " + nifPath);
			return result;
		}

		try {
			auto parsedOpt = parseNif(data);
			if (parsedOpt.has_value()) {
				const auto& parsed = *parsedOpt;

				for (size_t i = 0; i < parsed.blockTypeIndex.size(); ++i) {
					uint16_t tIdx = parsed.blockTypeIndex[i];
					if (tIdx >= parsed.blockTypes.size())
						continue;
					const auto& bt = parsed.blockTypes[tIdx];
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
						auto fallbackPaths = extractXmlLikePathsFromRawBytes();
						if (!fallbackPaths.empty()) {
							result.allPhysicsXmlPaths = std::move(fallbackPaths);
							result.physicsXmlPath = result.allPhysicsXmlPaths.front();
						} else {
							result.errors.push_back("Physics marker found but no XML references were extracted: " + nifPath);
						}
					}
				}
				return result;
			}

			if (maybePhysicsNif) {
				auto fallbackPaths = extractXmlLikePathsFromRawBytes();
				if (!fallbackPaths.empty()) {
					result.hasPhysicsData = true;
					result.allPhysicsXmlPaths = std::move(fallbackPaths);
					result.physicsXmlPath = result.allPhysicsXmlPaths.front();
				}
			}
		} catch (const std::exception&) {
			if (maybePhysicsNif) {
				auto fallbackPaths = extractXmlLikePathsFromRawBytes();
				if (!fallbackPaths.empty()) {
					result.hasPhysicsData = true;
					result.allPhysicsXmlPaths = std::move(fallbackPaths);
					result.physicsXmlPath = result.allPhysicsXmlPaths.front();
				}
			}
		} catch (...) {
			if (maybePhysicsNif) {
				auto fallbackPaths = extractXmlLikePathsFromRawBytes();
				if (!fallbackPaths.empty()) {
					result.hasPhysicsData = true;
					result.allPhysicsXmlPaths = std::move(fallbackPaths);
					result.physicsXmlPath = result.allPhysicsXmlPaths.front();
				}
			}
		}

		return result;
	}

}  // namespace hdt
