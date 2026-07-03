#include "hdtNIFBinaryParser.h"

#include "../../hdtStringUtils.h"
#include "../Schema/hdtNifSchema.h"
#include "../Utils/hdtNIFBinaryUtils.h"

#include <cstring>
#include <unordered_set>

namespace hdt
{
	namespace nif
	{
		namespace
		{
			// True if `typeName` is, or transitively inherits from, "NiNode" per the schema —
			// i.e. a scene-graph node that carries a NiObjectNET Name at block offset 0. Walking the
			// inherit chain (bounded to guard against a malformed/cyclic schema) keeps the skin-bone
			// read honest: geometry, properties and unknown types resolve to false and are skipped.
			bool inheritsFromNiNode(const NifSchema& schema, const std::string& typeName)
			{
				std::string t = typeName;
				for (int guard = 0; guard < 64 && !t.empty(); ++guard) {
					if (t == "NiNode")
						return true;
					const NifTypeDef* def = schema.findType(t);
					if (!def)
						return false;
					t = def->inherit;
				}
				return false;
			}
		}  // namespace

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

		std::vector<std::string> ExtractSkinBoundBoneNames(const ParsedNif& parsed)
		{
			const NifSchema& schema = globalNifSchema();
			const int32_t numBlocks = static_cast<int32_t>(parsed.blocks.size());
			std::unordered_set<std::string> nameSet;

			for (int32_t i = 0; i < numBlocks; ++i) {
				auto tOpt = blockTypeOf(parsed, i);
				if (!tOpt)
					continue;
				if (*tOpt != kTypeNiSkinInstance && *tOpt != kTypeBSDismemberSkinInstance &&
					*tOpt != kTypeBSSkinInstance)
					continue;

				const auto& block = parsed.blocks[static_cast<size_t>(i)];
				auto details = walkBlockRefDetails(schema, *tOpt, block.data(), block.size(), numBlocks);
				if (!details)
					continue;

				for (const auto& ref : details->refs) {
					// The Bones array is the skin instance's only array-valued Ref; the scalar
					// Data / Skin Partition / Skeleton Root refs carry an empty arr1 and are not bones.
					if (ref.arr1.empty())
						continue;
					if (ref.offset + 4 > block.size())
						continue;
					int32_t boneIdx = -1;
					std::memcpy(&boneIdx, block.data() + ref.offset, 4);
					if (boneIdx < 0 || boneIdx >= numBlocks)
						continue;

					auto boneTypeOpt = blockTypeOf(parsed, boneIdx);
					if (!boneTypeOpt || !inheritsFromNiNode(schema, *boneTypeOpt))
						continue;  // not a scene node — no reliable Name at offset 0

					const auto& boneBlock = parsed.blocks[static_cast<size_t>(boneIdx)];
					if (boneBlock.size() < 4)
						continue;
					uint32_t nameIdx = 0;
					std::memcpy(&nameIdx, boneBlock.data(), 4);
					if (nameIdx >= parsed.strings.size())
						continue;
					const std::string& name = parsed.strings[nameIdx];
					if (!name.empty())
						nameSet.insert(ToLowerAscii(name));
				}
			}

			return std::vector<std::string>(nameSet.begin(), nameSet.end());
		}

	}  // namespace nif

}  // namespace hdt
