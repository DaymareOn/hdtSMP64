#include "hdtNIFBinaryParser.h"

#include "../Utils/hdtNIFBinaryUtils.h"

#include <cstring>

namespace hdt
{
	namespace nif
	{

		std::vector<std::string> FindXmlPathsInNif(const ParsedNif& parsed)
		{
			int markerIdx = -1;
			for (int i = 0; i < static_cast<int>(parsed.strings.size()); ++i) {
				if (parsed.strings[static_cast<size_t>(i)] == kPhysicsMarker) {
					markerIdx = i;
					break;
				}
			}
			if (markerIdx < 0)
				return {};

			int niStrExtraTypeIdx = -1;
			for (int i = 0; i < static_cast<int>(parsed.blockTypes.size()); ++i) {
				if (parsed.blockTypes[static_cast<size_t>(i)] == kTypeNiStringExtraData) {
					niStrExtraTypeIdx = i;
					break;
				}
			}
			if (niStrExtraTypeIdx < 0)
				return {};

			std::vector<std::string> paths;
			for (size_t i = 0; i < parsed.blocks.size(); ++i) {
				if (i >= parsed.blockTypeIndex.size())
					break;
				// Mask off PhysX high-bit (0x8000) before comparison
				uint16_t masked = parsed.blockTypeIndex[i] & 0x7FFF;
				if (masked != static_cast<uint16_t>(niStrExtraTypeIdx))
					continue;

				const auto& block = parsed.blocks[i];
				if (block.size() < kNiStringExtraDataMinBlockSize)
					continue;

				uint32_t nameIdx = 0, valueIdx = 0;
				std::memcpy(&nameIdx, block.data(), 4);
				std::memcpy(&valueIdx, block.data() + 4, 4);

				if (static_cast<int>(nameIdx) == markerIdx &&
					valueIdx < static_cast<uint32_t>(parsed.strings.size())) {
					paths.push_back(parsed.strings[valueIdx]);
				}
			}
			return paths;
		}

	}  // namespace nif

}  // namespace hdt
