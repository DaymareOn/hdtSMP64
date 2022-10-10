#pragma once

#include <cstdint>
#include <map>
using namespace std;

#define V1_4_15  0 // 0, supported,		sksevr 2_00_12, vr
#define V1_5_97  1 // 1, supported,		skse64 2_00_20, se
#define	V1_6_318 2 // 2, unsupported,	skse64 2_01_05, ae
#define	V1_6_323 3 // 3, unsupported,	skse64 2_01_05, ae
#define	V1_6_342 4 // 4, unsupported,	skse64 2_01_05, ae
#define	V1_6_353 5 // 5, supported,		skse64 2_01_05, ae
#define	V1_6_629 6 // 6, unsupported,	skse64 2_02_02, ae629+
#define	V1_6_640 7 // 7, supported,		skse64 2_02_02, ae629+
//#define V1_6_659 8 // 8, supported,	skse64 2_02_02, ae629+
//#define V1_6_678 9 // 9, supported,	skse64 2_02_02, ae629+

#if CURRENTVERSION == V1_4_15
#define SKYRIMVR
#endif

#if CURRENTVERSION >= V1_6_318
#define ANNIVERSARY_EDITION
#endif

#if CURRENTVERSION >= V1_6_318 && CURRENTVERSION <= V1_6_353
#define ANNIVERSARY_EDITION_353MINUS
#endif

namespace hdt
{
	// signatures generated with IDA SigMaker plugin for SSE
	// comment is SSE, uncommented is VR. VR address found by Ghidra compare using known SSE offset to find same function
	// (based on function signature and logic)
	namespace offset
	{
		struct offsetData
		{
			int id;
			uintptr_t V [8];
		};

		struct
		{
			offsetData GameStepTimer_SlowTime;
			offsetData ArmorAttachFunction;
			offsetData BSFaceGenNiNode_SkinAllGeometry;
			offsetData BSFaceGenNiNode_SkinSingleGeometry;
			offsetData GameLoopFunction;
			offsetData GameShutdownFunction;
			offsetData TESNPC_GetFaceGeomPath;
			offsetData BSFaceGenModelExtraData_BoneLimit;
		}
		constexpr functionsOffsets =
		{
			{ 410199, { 0x030C3A08, 0x02F6B948, 0x030064C8, 0x030064c8, 0x03007708, 0x03007708, 0x03006808, 0x03006808 }},
			{ 15712,  { 0x001DB9E0, 0x001CAFB0, 0x001D6740, 0x001d66b0, 0x001d66a0, 0x001d66a0, 0x001d83b0, 0x001d83b0 }},
			{ 26986,  { 0x003e8120, 0x003D87B0, 0x003F08C0, 0x003f0830, 0x003f09c0, 0x003f0830, 0x003f2990, 0x003f2990 }},
			{ 26987,  { 0x003e81b0, 0x003D8840, 0x003F0A50, 0x003f09c0, 0x003f0b50, 0x003f09c0, 0x003f2b20, 0x003f2b20 }},
			{ 36564,  { 0x005BAB10, 0x005B2FF0, 0x005D9F50, 0x005D9CC0, 0x005dae80, 0x005dace0, 0x005ec310, 0x005ec240 }},
			{ 105623, { 0x012CC630, 0x01293D20, 0x013B9A90, 0x013b99f0, 0x013ba910, 0x013ba9a0, 0x013b8230, 0x013b8160 }},
			{ 24726,  { 0x000372b30,0x00363210, 0x0037A240, 0x0037a1b0, 0x0037a340, 0x0037a1b0, 0x0037c1e0, 0x0037c1e0 }},
			{ 0,      { 0x00037ae28,0x0036B4C8, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }}
		};

		constexpr auto GameStepTimer_SlowTime				= functionsOffsets.GameStepTimer_SlowTime				.V[CURRENTVERSION];
		constexpr auto ArmorAttachFunction					= functionsOffsets.ArmorAttachFunction					.V[CURRENTVERSION];
		constexpr auto BSFaceGenNiNode_SkinAllGeometry		= functionsOffsets.BSFaceGenNiNode_SkinAllGeometry		.V[CURRENTVERSION];
		constexpr auto BSFaceGenNiNode_SkinSingleGeometry	= functionsOffsets.BSFaceGenNiNode_SkinSingleGeometry	.V[CURRENTVERSION];
		constexpr auto GameLoopFunction						= functionsOffsets.GameLoopFunction						.V[CURRENTVERSION];
		constexpr auto GameShutdownFunction					= functionsOffsets.GameShutdownFunction					.V[CURRENTVERSION];
		constexpr auto TESNPC_GetFaceGeomPath				= functionsOffsets.TESNPC_GetFaceGeomPath				.V[CURRENTVERSION];
		constexpr auto BSFaceGenModelExtraData_BoneLimit	= functionsOffsets.BSFaceGenModelExtraData_BoneLimit	.V[CURRENTVERSION];

		// .text:00000001403D88D4                 cmp     ebx, 8
		// patch 8 -> 7
		// The same for AE/SE/VR.
		constexpr std::uintptr_t BSFaceGenNiNode_SkinSingleGeometry_bug = BSFaceGenNiNode_SkinSingleGeometry + 0x96;
	}
}
