#pragma once

#include <cstddef>
#include <cstdint>

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

		constexpr const char* kPhysicsMarker = "HDT Skinned Mesh Physics Object";
		constexpr const char* kNifHeaderMagic = "Gamebryo File Format";
		constexpr const char* kTypeNiStringExtraData = "NiStringExtraData";
		constexpr const char* kTypeBSTriShape = "BSTriShape";
		constexpr const char* kTypeBSDynamicTriShape = "BSDynamicTriShape";
		constexpr const char* kTypeNiSkinInstance = "NiSkinInstance";
		constexpr const char* kTypeBSSkinInstance = "BSSkin::Instance";

	}  // namespace nif

}  // namespace hdt
