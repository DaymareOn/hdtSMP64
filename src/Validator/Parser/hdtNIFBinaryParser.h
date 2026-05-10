#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hdt
{
	namespace nif
	{
	struct NifHeaderInfo
	{
		std::vector<std::string> blockTypes;
		std::vector<uint16_t> blockTypeIndex;
		std::vector<uint32_t> blockSizes;
		std::vector<std::string> strings;
		size_t blockDataOffset = 0;
		bool hasGeometry = false;
		bool hasSkinning = false;
	};

	// Parses NIF v20.2.0.7 header data from the first byte after the NIF magic header string.
	bool ParseNifHeader(
		const std::vector<uint8_t>& data,
		size_t headerDataStartOffset,
		NifHeaderInfo& outInfo);

	// Extracts all XML string references from NiStringExtraData blocks named with the physics marker.
	std::vector<std::string> FindXmlPathsInHeader(
		const NifHeaderInfo& info,
		const std::vector<uint8_t>& rawData);

	}  // namespace nif

}  // namespace hdt
