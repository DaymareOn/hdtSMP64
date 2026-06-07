#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

namespace hdt
{
	namespace nif
	{
		constexpr uint32_t kVersion_20_2_0_7 = 0x14020007u;
		constexpr uint32_t kMaxBlocks = 100000u;
		constexpr uint32_t kMaxStrings = 100000u;
		constexpr uint32_t kMaxSizedStringLen = 4096u;
		// NiStringExtraData in NIF 20.2.0.7 (SSE): Name(4) + String Data(4) = 8 bytes.
		// The "Next Extra Data" ref present in older formats was moved to NiObjectNET in 10.2.0.0.
		constexpr uint32_t kNiStringExtraDataMinBlockSize = 8u;

		constexpr size_t kHeaderProbeLimit = 200u;
		constexpr size_t kMaxNifFileSize = 256ull * 1024ull * 1024ull;

		// ── SSE NIF format constants ──────────────────────────────────────────────

		/// BSStream version field for Skyrim Special Edition NIFs.
		constexpr uint32_t kSSEBsVersion = 100u;
		/// Serialised byte size of one triangle (3 × uint16_t).
		constexpr uint32_t kTriangleByteSize = 6u;

		// Vertex attribute flags packed into bits [44:56] of the vertex descriptor.
		constexpr uint16_t kVertexAttrVertex = 1u << 0;
		constexpr uint16_t kVertexAttrUV = 1u << 1;
		constexpr uint16_t kVertexAttrUV2 = 1u << 2;
		constexpr uint16_t kVertexAttrNormal = 1u << 3;
		constexpr uint16_t kVertexAttrTangent = 1u << 4;
		constexpr uint16_t kVertexAttrColor = 1u << 5;
		constexpr uint16_t kVertexAttrSkinned = 1u << 6;

		// ── Block type name constants ──────────────────────────────────────────────

		constexpr const char* kPhysicsMarker = "HDT Skinned Mesh Physics Object";
		constexpr const char* kNifHeaderMagic = "Gamebryo File Format";
		constexpr const char* kNifHeaderMagicLegacy = "NetImmerse File Format";
		constexpr const char* kTypeNiStringExtraData = "NiStringExtraData";
		constexpr const char* kTypeBSTriShape = "BSTriShape";
		constexpr const char* kTypeBSDynamicTriShape = "BSDynamicTriShape";
		constexpr const char* kTypeNiSkinInstance = "NiSkinInstance";
		constexpr const char* kTypeBSDismemberSkinInstance = "BSDismemberSkinInstance";
		constexpr const char* kTypeBSSkinInstance = "BSSkin::Instance";
		constexpr const char* kTypeNiSkinPartition = "NiSkinPartition";
		constexpr const char* kTypeNiSkinData = "NiSkinData";

		// Allowlist of SSE skinned-mesh geometry block types the structural checks analyse.
		// Derived layouts (BSSubIndexTriShape, BSMeshLODTriShape, …) carry extra trailing
		// fields the parser does not model, so they are intentionally excluded.
		inline bool isSupportedSSETriShapeType(std::string_view type)
		{
			return type == kTypeBSTriShape || type == kTypeBSDynamicTriShape;
		}

		// Returns true if `needle` appears anywhere in the byte buffer.
		inline bool ContainsAsciiSequence(const uint8_t* data, size_t size, const char* needle)
		{
			const size_t nlen = std::strlen(needle);
			if (nlen == 0 || size < nlen)
				return false;
			for (size_t i = 0; i + nlen <= size; ++i) {
				if (std::memcmp(data + i, needle, nlen) == 0)
					return true;
			}
			return false;
		}

	}  // namespace nif

}  // namespace hdt
