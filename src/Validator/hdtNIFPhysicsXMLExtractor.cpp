#include "hdtNIFValidator.h"

#include "Parser/hdtNIFBinaryParser.h"
#include "Utils/hdtNIFBinaryUtils.h"

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

namespace hdt
{
	/// Scans a NIF file to find FSMP physics data and referenced XML config paths.
	/// Performs strict header-based parsing and reports scan/parse failures in result.errors.
	/// Returns a default result with hasPhysicsData=false when no physics marker exists.
	NIFScanResult ExtractPhysicsXmlRefsFromNIFs(const std::string& nifPath)
	{
		NIFScanResult result;

		std::ifstream file(nifPath, std::ios::binary | std::ios::ate);
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

		std::vector<uint8_t> data(static_cast<size_t>(fileSize));
		file.seekg(0);
		file.read(reinterpret_cast<char*>(data.data()), fileSize);
		file.close();

		size_t headerEnd = 0;
		for (size_t i = 0; i < std::min(data.size(), nif::kHeaderProbeLimit); ++i) {
			if (data[i] == 0x00) {
				headerEnd = i + 1;
				break;
			}
		}
		if (headerEnd == 0) {
			result.errors.push_back("NIF header terminator not found: " + nifPath);
			return result;
		}

		std::string headerStr(reinterpret_cast<const char*>(data.data()), headerEnd - 1);
		if (headerStr.find(nif::kNifHeaderMagic) == std::string::npos) {
			result.errors.push_back("Missing NIF header magic: " + nifPath);
			return result;
		}

		try {
			nif::NifHeaderInfo headerInfo;
			if (nif::ParseNifHeader(data, headerEnd, headerInfo)) {
				result.hasGeometry = headerInfo.hasGeometry;
				result.hasSkinning = headerInfo.hasSkinning;

				bool hasMarker = false;
				for (const auto& s : headerInfo.strings) {
					if (s == nif::kPhysicsMarker) {
						hasMarker = true;
						break;
					}
				}

				if (hasMarker) {
					result.hasPhysicsData = true;
					result.allPhysicsXmlPaths = nif::FindXmlPathsInHeader(headerInfo, data);
					if (!result.allPhysicsXmlPaths.empty()) {
						result.physicsXmlPath = result.allPhysicsXmlPaths[0];
					} else {
						result.errors.push_back("Physics marker found but no XML references were extracted: " + nifPath);
					}
				}
				return result;
			}

			result.errors.push_back("Failed to parse NIF header: " + nifPath);
		} catch (const std::exception& e) {
			result.errors.push_back("Exception while parsing NIF header '" + nifPath + "': " + e.what());
		} catch (...) {
			result.errors.push_back("Unknown exception while parsing NIF header: " + nifPath);
		}

		return result;
	}

}  // namespace hdt
