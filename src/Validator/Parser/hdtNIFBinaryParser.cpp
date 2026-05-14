#include "hdtNIFBinaryParser.h"

#include "../Improvers/hdtNIFBinaryIO.h"
#include "../Utils/hdtNIFBinaryUtils.h"

#include <cstring>
#include <utility>

namespace hdt
{
	namespace nif
	{

	/// Parses the NIF v20.2.0.7 header payload beginning after the magic string.
	/// Fills outInfo with type tables, string table, block layout, and quick feature flags.
	/// Returns false for unsupported/invalid headers; throws only on malformed buffered reads.
	bool ParseNifHeader(
		const std::vector<uint8_t>& data,
		size_t headerDataStartOffset,
		NifHeaderInfo& outInfo)
	{
		NifReader r(data, headerDataStartOffset);

		uint32_t version = r.readU32();
		if (version != kVersion_20_2_0_7)
			return false;

		/* endianness = */ r.readU8();
		uint32_t userVersion = r.readU32();
		uint32_t numBlocks = r.readU32();
		if (numBlocks > kMaxBlocks)
			return false;

		uint32_t bsVersion = (userVersion >= 10) ? r.readU32() : 0u;

		if (userVersion >= 10) {
			r.readShortSizedStr();
			if (bsVersion >= 130) {
				r.readShortSizedStr();
				r.readShortSizedStr();
			} else {
				r.skip(4);
			}
		}

		uint16_t numBlockTypes = r.readU16();
		NifHeaderInfo info;
		info.blockTypes.reserve(numBlockTypes);
		for (uint16_t i = 0; i < numBlockTypes; ++i) {
			info.blockTypes.push_back(r.readSizedStr());
		}

		info.blockTypeIndex.reserve(numBlocks);
		for (uint32_t i = 0; i < numBlocks; ++i) {
			info.blockTypeIndex.push_back(r.readU16());
		}

		info.blockSizes.reserve(numBlocks);
		for (uint32_t i = 0; i < numBlocks; ++i) {
			info.blockSizes.push_back(r.readU32());
		}

		uint32_t numStrings = r.readU32();
		if (numStrings > kMaxStrings)
			return false;
		/* maxStringLen = */ r.readU32();

		info.strings.reserve(numStrings);
		for (uint32_t i = 0; i < numStrings; ++i) {
			info.strings.push_back(r.readSizedStr());
		}

		uint32_t numGroups = r.readU32();
		if (numGroups > kMaxBlocks)
			return false;
		r.skip(numGroups * 4u);
		info.blockDataOffset = r.pos();

		for (uint32_t i = 0; i < static_cast<uint32_t>(info.blockTypeIndex.size()); ++i) {
			uint16_t tIdx = info.blockTypeIndex[i];
			if (tIdx < static_cast<uint16_t>(info.blockTypes.size())) {
				const std::string& bt = info.blockTypes[tIdx];
				if (bt == kTypeBSTriShape || bt == kTypeBSDynamicTriShape) {
					info.hasGeometry = true;
				} else if (bt == kTypeNiSkinInstance || bt == kTypeBSSkinInstance) {
					info.hasSkinning = true;
				}
			}
		}

		outInfo = std::move(info);
		return true;
	}

	/// Scans parsed block metadata to collect XML path strings referenced by
	/// NiStringExtraData entries named with the FSMP physics marker.
	/// Returns all discovered paths (including duplicates if present in the file).
	std::vector<std::string> FindXmlPathsInHeader(
		const NifHeaderInfo& info,
		const std::vector<uint8_t>& rawData)
	{
		int markerIdx = -1;
		for (int i = 0; i < static_cast<int>(info.strings.size()); ++i) {
			if (info.strings[i] == kPhysicsMarker) {
				markerIdx = i;
				break;
			}
		}
		if (markerIdx < 0)
			return {};

		int niStrExtraTypeIdx = -1;
		for (int i = 0; i < static_cast<int>(info.blockTypes.size()); ++i) {
			if (info.blockTypes[i] == kTypeNiStringExtraData) {
				niStrExtraTypeIdx = i;
				break;
			}
		}
		if (niStrExtraTypeIdx < 0)
			return {};

		std::vector<std::string> paths;
		size_t blockOffset = info.blockDataOffset;
		for (size_t i = 0; i < info.blockSizes.size(); ++i) {
			uint32_t blockSize = info.blockSizes[i];
			uint16_t typeIdx = info.blockTypeIndex[i];

			if (typeIdx == static_cast<uint16_t>(niStrExtraTypeIdx) &&
				blockSize >= kNiStringExtraDataMinBlockSize &&
				blockOffset + kNiStringExtraDataMinBlockSize <= rawData.size()) {
				uint32_t nameIdx;
				std::memcpy(&nameIdx, rawData.data() + blockOffset, 4);
				uint32_t valueIdx;
				std::memcpy(&valueIdx, rawData.data() + blockOffset + 8, 4);

				if (static_cast<int>(nameIdx) == markerIdx && valueIdx < static_cast<uint32_t>(info.strings.size())) {
					paths.push_back(info.strings[valueIdx]);
				}
			}

			blockOffset += blockSize;
		}

		return paths;
	}

	}  // namespace nif

}  // namespace hdt
