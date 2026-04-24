#include "hdtNIFValidator.h"

#include "NetImmerseUtils.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace hdt
{
	// ---- Minimal NIF binary reader ----

	class NifReader
	{
	public:
		NifReader(const std::vector<uint8_t>& data) : m_data(data), m_pos(0) {}

		bool canRead(size_t bytes) const { return m_pos + bytes <= m_data.size(); }
		size_t pos() const { return m_pos; }
		const uint8_t* rawAt(size_t offset) const { return m_data.data() + offset; }
		size_t size() const { return m_data.size(); }

		uint8_t readU8()
		{
			if (!canRead(1))
				throw std::runtime_error("NIF: read past end (U8)");
			return m_data[m_pos++];
		}

		uint16_t readU16()
		{
			if (!canRead(2))
				throw std::runtime_error("NIF: read past end (U16)");
			uint16_t v;
			std::memcpy(&v, m_data.data() + m_pos, 2);
			m_pos += 2;
			return v;
		}

		uint32_t readU32()
		{
			if (!canRead(4))
				throw std::runtime_error("NIF: read past end (U32)");
			uint32_t v;
			std::memcpy(&v, m_data.data() + m_pos, 4);
			m_pos += 4;
			return v;
		}

		// Read a uint32-length-prefixed string (NIF string table format)
		std::string readSizedStr()
		{
			uint32_t len = readU32();
			if (len > 4096)
				throw std::runtime_error("NIF: implausible string length");
			if (!canRead(len))
				throw std::runtime_error("NIF: string out of bounds");
			std::string s((char*)m_data.data() + m_pos, len);
			m_pos += len;
			return s;
		}

		// Read a uint8-length-prefixed string (BS header format)
		std::string readShortSizedStr()
		{
			uint8_t len = readU8();
			if (!canRead(len))
				throw std::runtime_error("NIF: short string out of bounds");
			std::string s((char*)m_data.data() + m_pos, len);
			m_pos += len;
			return s;
		}

		void skip(size_t bytes)
		{
			if (!canRead(bytes))
				throw std::runtime_error("NIF: skip past end");
			m_pos += bytes;
		}

	private:
		const std::vector<uint8_t>& m_data;
		size_t m_pos;
	};

	static const char kPhysicsMarker[] = "HDT Skinned Mesh Physics Object";

	// ---- NIF header parser: returns physics XML path if found ----

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

	// Returns nullopt on parse failure; caller falls back to string scan.
	static std::optional<NifHeaderInfo> parseNifHeader(NifReader& r)
	{
		static constexpr uint32_t kVersion_20_2_0_7 = 0x14020007u;

		uint32_t version = r.readU32();
		if (version != kVersion_20_2_0_7)
			return std::nullopt;

		/* endianness = */ r.readU8();
		uint32_t userVersion = r.readU32();
		uint32_t numBlocks = r.readU32();
		if (numBlocks > 100000u)
			return std::nullopt;

		uint32_t bsVersion = (userVersion >= 10) ? r.readU32() : 0u;

		// BS extended header strings
		if (userVersion >= 10) {
			r.readShortSizedStr();  // author
			if (bsVersion >= 130) {
				r.readShortSizedStr();  // process script
				r.readShortSizedStr();  // export script
			} else {
				// Older BS header (LE NIFs, BSVersion < 130): one extra uint32 follows
				// the author string. This is an undocumented reserved/unknown field
				// present in Fallout 3/NV and Skyrim LE NIF headers.
				r.skip(4);
			}
		}

		// Block types
		uint16_t numBlockTypes = r.readU16();
		NifHeaderInfo info;
		info.blockTypes.reserve(numBlockTypes);
		for (uint16_t i = 0; i < numBlockTypes; ++i) {
			info.blockTypes.push_back(r.readSizedStr());
		}

		// Block type indices
		info.blockTypeIndex.reserve(numBlocks);
		for (uint32_t i = 0; i < numBlocks; ++i) {
			info.blockTypeIndex.push_back(r.readU16());
		}

		// Block sizes
		info.blockSizes.reserve(numBlocks);
		for (uint32_t i = 0; i < numBlocks; ++i) {
			info.blockSizes.push_back(r.readU32());
		}

		// String table
		uint32_t numStrings = r.readU32();
		if (numStrings > 100000u)
			return std::nullopt;
		/* maxStringLen = */ r.readU32();
		info.strings.reserve(numStrings);
		for (uint32_t i = 0; i < numStrings; ++i) {
			info.strings.push_back(r.readSizedStr());
		}

		// Groups
		uint32_t numGroups = r.readU32();
		r.skip(numGroups * 4u);

		info.blockDataOffset = r.pos();

		// Count geometry and skinning blocks
		for (uint32_t i = 0; i < (uint32_t)info.blockTypeIndex.size(); ++i) {
			uint16_t tIdx = info.blockTypeIndex[i];
			if (tIdx < (uint16_t)info.blockTypes.size()) {
				const std::string& bt = info.blockTypes[tIdx];
				if (bt == "BSTriShape" || bt == "BSDynamicTriShape") {
					info.hasGeometry = true;
				} else if (bt == "NiSkinInstance" || bt == "BSSkin::Instance") {
					info.hasSkinning = true;
				}
			}
		}

		return info;
	}

	// Search string table for physics marker; extract XML path from matching NiStringExtraData.
	static std::string findXmlPathInHeader(const NifHeaderInfo& info,
		const std::vector<uint8_t>& rawData)
	{
		// Find the string index of the physics marker
		int markerIdx = -1;
		for (int i = 0; i < (int)info.strings.size(); ++i) {
			if (info.strings[i] == kPhysicsMarker) {
				markerIdx = i;
				break;
			}
		}
		if (markerIdx < 0)
			return {};

		// Find the block type index for NiStringExtraData
		int niStrExtraTypeIdx = -1;
		for (int i = 0; i < (int)info.blockTypes.size(); ++i) {
			if (info.blockTypes[i] == "NiStringExtraData") {
				niStrExtraTypeIdx = i;
				break;
			}
		}
		if (niStrExtraTypeIdx < 0)
			return {};

		// Walk blocks, find NiStringExtraData with name == markerIdx
		// NiStringExtraData block layout (v20.2.0.7):
		//   name_ref:     uint32 (string table index)
		//   next_extra:   int32  (block ref, -1 = none)
		//   string_data:  uint32 (string table index for the value)
		size_t blockOffset = info.blockDataOffset;
		for (size_t i = 0; i < info.blockSizes.size(); ++i) {
			uint32_t blockSize = info.blockSizes[i];
			uint16_t typeIdx = info.blockTypeIndex[i];

			if (typeIdx == (uint16_t)niStrExtraTypeIdx && blockSize >= 12 &&
				blockOffset + 12 <= rawData.size()) {
				uint32_t nameIdx;
				std::memcpy(&nameIdx, rawData.data() + blockOffset, 4);
				// bytes 4-7 = nextExtra (skip)
				uint32_t valueIdx;
				std::memcpy(&valueIdx, rawData.data() + blockOffset + 8, 4);

				if ((int)nameIdx == markerIdx && valueIdx < (uint32_t)info.strings.size()) {
					return info.strings[valueIdx];
				}
			}

			blockOffset += blockSize;
		}

		return {};
	}

	// Fallback: raw byte search for the physics marker string.
	static NIFScanResult stringScanFallback(const std::vector<uint8_t>& data)
	{
		NIFScanResult result;
		const size_t markerLen = std::strlen(kPhysicsMarker);

		for (size_t i = 0; i + markerLen <= data.size(); ++i) {
			if (std::memcmp(data.data() + i, kPhysicsMarker, markerLen) != 0)
				continue;

			result.hasPhysicsData = true;

			// Search forward for a length-prefixed XML path string
			size_t searchEnd = std::min(data.size(), i + markerLen + 1024);
			for (size_t j = i + markerLen; j + 4 < searchEnd;) {
				uint32_t strLen;
				std::memcpy(&strLen, data.data() + j, 4);

				if (strLen > 0 && strLen < 260 && j + 4 + strLen <= data.size()) {
					bool printable = true;
					for (uint32_t k = 0; k < strLen && printable; ++k) {
						uint8_t c = data[j + 4 + k];
						printable = (c >= 0x20 && c <= 0x7E);
					}
					if (printable) {
						std::string candidate((char*)data.data() + j + 4, strLen);
						std::string lower = candidate;
						std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
						if (lower.find(".xml") != std::string::npos) {
							result.physicsXmlPath = candidate;
							return result;
						}
						j += 4 + strLen;
						continue;
					}
				}
				++j;
			}
			break;
		}

		return result;
	}

	// ---- NIF binary scanner ----

	NIFScanResult ScanNIFBinary(const std::string& nifPath)
	{
		NIFScanResult result;

		std::ifstream file(nifPath, std::ios::binary | std::ios::ate);
		if (!file.is_open()) {
			result.errors.push_back("Cannot open: " + nifPath);
			return result;
		}
		auto fileSize = file.tellg();
		if (fileSize <= 0 || fileSize > 256 * 1024 * 1024) {
			return result;
		}

		std::vector<uint8_t> data(static_cast<size_t>(fileSize));
		file.seekg(0);
		file.read((char*)data.data(), fileSize);
		file.close();

		// Find the null-terminated ASCII header
		size_t headerEnd = 0;
		for (size_t i = 0; i < std::min(data.size(), size_t(200)); ++i) {
			if (data[i] == 0x00) {
				headerEnd = i + 1;
				break;
			}
		}
		if (headerEnd == 0) {
			return result;  // Not a NIF; skip silently
		}

		// Validate header
		std::string headerStr((char*)data.data(), headerEnd - 1);
		if (headerStr.find("Gamebryo File Format") == std::string::npos) {
			return result;
		}

		// Try full header parse first
		try {
			NifReader r(data);
			r.skip(headerEnd);

			auto headerInfo = parseNifHeader(r);
			if (headerInfo.has_value()) {
				result.hasGeometry = headerInfo->hasGeometry;
				result.hasSkinning = headerInfo->hasSkinning;

				// Check if any string in the table is the physics marker
				bool hasMarker = false;
				for (const auto& s : headerInfo->strings) {
					if (s == kPhysicsMarker) {
						hasMarker = true;
						break;
					}
				}

				if (hasMarker) {
					result.hasPhysicsData = true;
					result.physicsXmlPath = findXmlPathInHeader(*headerInfo, data);
				}
				return result;
			}
		} catch (...) {
			// Fall through to string scan
		}

		// Fallback string scan
		return stringScanFallback(data);
	}

	// ---- NIF structural validator (runtime, requires loaded RE::NiNode*) ----

	static void collectBones(RE::NiNode* node, std::vector<std::string>& boneNames,
		std::vector<std::string>& errors)
	{
		if (!node)
			return;

		const char* name = node->name.c_str();
		if (name && name[0] != '\0') {
			boneNames.push_back(name);
		}

		for (auto& child : node->GetChildren()) {
			if (!child)
				continue;
			RE::NiNode* childNode = castNiNode(child.get());
			if (childNode) {
				collectBones(childNode, boneNames, errors);
			}
		}
	}

	static bool checkTransformValid(const RE::NiTransform& xfm, const std::string& boneName,
		std::vector<std::string>& errors)
	{
		bool ok = true;
		const auto& t = xfm.translate;
		if (std::isnan(t.x) || std::isnan(t.y) || std::isnan(t.z) ||
			std::isinf(t.x) || std::isinf(t.y) || std::isinf(t.z)) {
			errors.push_back("Bone '" + boneName + "': NaN/inf translation");
			ok = false;
		}
		if (std::isnan(xfm.scale) || std::isinf(xfm.scale)) {
			errors.push_back("Bone '" + boneName + "': NaN/inf scale");
			ok = false;
		}
		if (xfm.scale < 1e-6f) {
			errors.push_back(
				"Bone '" + boneName + "': scale is zero or near-zero (" +
				std::to_string(xfm.scale) + ")");
			ok = false;
		}
		if (xfm.scale > 1000.f) {
			errors.push_back(
				"Bone '" + boneName + "': scale is extreme (" +
				std::to_string(xfm.scale) + ")");
			ok = false;
		}
		return ok;
	}

	static void validateSkinning(RE::NiAVObject* obj, NIFStructuralResult& result)
	{
		if (!obj)
			return;

		RE::BSTriShape* triShape = castBSTriShape(obj);
		if (triShape) {
			auto& runtimeData = triShape->GetGeometryRuntimeData();
			if (!runtimeData.skinInstance) {
				result.warnings.push_back(
					std::string("Mesh '") + triShape->name.c_str() +
					"' has no NiSkinInstance");
			} else {
				result.hasSkinningData = true;
				RE::NiSkinInstance* skinInst = runtimeData.skinInstance.get();

				if (!skinInst->skinData) {
					result.errors.push_back(
						std::string("Mesh '") + triShape->name.c_str() +
						"' has null skinData");
				} else {
					RE::NiSkinData* skinData = skinInst->skinData.get();
					result.boneCount += skinData->bones;

					if (skinData->bones == 0) {
						result.errors.push_back(
							std::string("Mesh '") + triShape->name.c_str() +
							"' has zero bones in skinData");
					}

					for (uint32_t i = 0; i < skinData->bones; ++i) {
						if (!skinInst->bones[i]) {
							result.errors.push_back(
								std::string("Mesh '") + triShape->name.c_str() +
								"' has null bone[" + std::to_string(i) + "]");
						}
					}
				}
			}
		}

		RE::NiNode* node = castNiNode(obj);
		if (node) {
			for (auto& child : node->GetChildren()) {
				if (child) {
					validateSkinning(child.get(), result);
				}
			}
		}
	}

	NIFStructuralResult ValidateNIFStructure(RE::NiNode* root, const std::string& nifPath)
	{
		NIFStructuralResult result;

		if (!root) {
			result.isValid = false;
			result.errors.push_back(nifPath + ": root NiNode is null");
			return result;
		}

		// Phase 6.1: Collect skeleton hierarchy (bone names)
		collectBones(root, result.boneNames, result.errors);
		result.boneCount = static_cast<uint32_t>(result.boneNames.size());

		if (result.boneCount == 0) {
			result.isValid = false;
			result.errors.push_back(nifPath + ": no bones found in skeleton");
		}

		// Phase 6.2: Check all bone transforms for NaN/inf/extreme values
		std::function<void(RE::NiNode*)> checkTransforms = [&](RE::NiNode* node) {
			if (!node)
				return;
			std::string name = node->name.c_str() ? node->name.c_str() : "<unnamed>";
			checkTransformValid(node->world, name, result.errors);
			for (auto& child : node->GetChildren()) {
				RE::NiNode* childNode = castNiNode(child.get());
				if (childNode) {
					checkTransforms(childNode);
				}
			}
		};
		checkTransforms(root);

		// Phase 6.3: Validate skinning data
		validateSkinning(root, result);

		if (!result.hasSkinningData) {
			result.warnings.push_back(nifPath + ": no skinned meshes found");
		}

		// Phase 6.4: Root scale sanity check
		if (root->world.scale > 1e-6f && std::abs(root->world.scale - 1.0f) > 0.5f) {
			result.warnings.push_back(nifPath + ": root scale " +
				std::to_string(root->world.scale) + " deviates significantly from 1.0");
		}

		result.isValid = result.errors.empty();
		return result;
	}

}  // namespace hdt
