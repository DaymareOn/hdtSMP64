#pragma once

using namespace std;

#define V1_4_15  0 // 0, supported,		sksevr 2_00_12, vr
#define V1_5_97  1 // 1, supported,		skse64 2_00_20, se
#define	V1_6_318 2 // 2, unsupported,	skse64 2_01_05, ae
#define	V1_6_323 3 // 3, unsupported,	skse64 2_01_05, ae
#define	V1_6_342 4 // 4, unsupported,	skse64 2_01_05, ae
#define	V1_6_353 5 // 5, supported,		skse64 2_01_05, ae
#define	V1_6_629 6 // 6, unsupported,	skse64 2_02_02, ae629+
#define	V1_6_640 7 // 7, supported,		skse64 2_02_02, ae629+
#define V1_6_659 8 // 8, supported,		skse64 2_02_02 gog, ae629+
//#define V1_6_678 9 // x, unsupported,	skse64 2_02_02, ae629+
#define V1_6_1130 9 // 9, supported,	skse64 2_02_05, ae629+
#define V1_6_1170 10 // 10, supported,    skse64 2_02_06, ae629+
#define V1_6_1179 11 // 11, supported,    skse64 2_02_06 gog, ae629+

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
			uintptr_t V[12];
		};

		struct
		{
			offsetData GameStepTimer_SlowTime;
			offsetData GameStepTimer_RealTime;
			offsetData ArmorAttachFunction;
			offsetData ItemUnequipFunction;
			offsetData BSFaceGenNiNode_SkinAllGeometry;
			offsetData BSFaceGenNiNode_SkinSingleGeometry;
			offsetData GameLoopFunction;
			offsetData GameShutdownFunction;
			offsetData TESNPC_GetFaceGeomPath;
			offsetData BSFaceGenModelExtraData_BoneLimit;
			offsetData Actor_CalculateLOS;
			offsetData SkyPointer;
			offsetData FrameSyncPoint;
		}

		/*
		 The GameLoopFunction offset providing the optimal parallelization (ie the soonest possible in the man frame loop)
		 has been considered in 3 places:
		 - the original one in Main::Update_1405B2FF0 at the end (SMP running after line 234 on 1.5.97)
		 - Main::Update_1405B2FF0 (SMP running after Line 230, 190): CTD
		 I keep the original.
		 The FrameSyncPoint offset providing the optimal parallelization (ie the latest possible in the main frame loop)
		 has been considered in 8 places:
		 - Main::Update_1405B2FF0 (SMP running before Line 26, 33, 84, 123, 136, 168): working
		 - Main::Update_1405B2FF0 (SMP running before/after Line 170): working
		 - Main::Update_1405B2FF0 (SMP running before Line 190): NOT working
		 I keep "after line 170".
		*/

		constexpr functionsOffsets =
		{

			{ 410199, { 0x030C3A08, 0x02F6B948, 0x030064C8, 0x030064c8, 0x03007708, 0x03007708, 0x03006808, 0x03006808, 0x030007c8, 0x031b2f48, 0x031cc288, 0x031cd6c8}},
			{ 0,      { 0x030C3A0C, 0x02F6B94C, 0x00000000, 0x00000000, 0x00000000, 0x0300770C, 0x00000000, 0x0300680C, 0x030007cc, 0x031b2f4c, 0x031cc28c, 0x031cd6cc}},
			{ 15712,  { 0x001DB9E0, 0x001CAFB0, 0x001D6740, 0x001d66b0, 0x001d66a0, 0x001d66a0, 0x001d83b0, 0x001d83b0, 0x001d81b0, 0x002177a0, 0x00217890, 0x002176c0}},
			{ 0,      { 0x006411a0, 0x00638190, 0x00000000, 0x00000000, 0x00000000, 0x0065eca0, 0x00000000, 0x00670210, 0x0066fd10, 0x006ca500, 0x006ca010, 0x006cc240}},
			{ 26986,  { 0x003e8120, 0x003D87B0, 0x003F08C0, 0x003f0830, 0x003f09c0, 0x003f0830, 0x003f2990, 0x003f2990, 0x003f2920, 0x00432190, 0x00432280, 0x004323c0}},
			{ 26987,  { 0x003e81b0, 0x003D8840, 0x003F0A50, 0x003f09c0, 0x003f0b50, 0x003f09c0, 0x003f2b20, 0x003f2b20, 0x003f2ab0, 0x00432320, 0x00432410, 0x00432550}},
			{ 36564,  { 0x005BAB10, 0x005B2FF0, 0x005D9F50, 0x005D9CC0, 0x005dae80, 0x005dace0, 0x005ec310, 0x005ec240, 0x005ebd40, 0x00646390, 0x00645ea0, 0x00648100}},
			{ 105623, { 0x012CC630, 0x01293D20, 0x013B9A90, 0x013b99f0, 0x013ba910, 0x013ba9a0, 0x013b8230, 0x013b8160, 0x013b33e0, 0x01475880, 0x0147df70, 0x0147f010}},//no longer used
			{ 24726,  { 0x00372b30, 0x00363210, 0x0037A240, 0x0037a1b0, 0x0037a340, 0x0037a1b0, 0x0037c1e0, 0x0037c1e0, 0x0037c170, 0x003bbc50, 0x003bbd40, 0x003bbe80}},
			{ 24313,  { 0x0037ae28, 0x0036B4C8, 0x00000000, 0x00000000, 0x00000000, 0x00382365, 0x00000000, 0x00384495, 0x00384425, 0x003C3F05, 0x003C3FF5, 0x003C4135}},//id:24836 + 0x75 - Don't forget to add 0x75 when defining offsets for a new skyrim version!
			{ 37770,  { 0x00605b10, 0x005fd2c0, 0x006241F0, 0x00000000, 0x00000000, 0x00624f90, 0x00000000, 0x006364f0, 0x00635ff0, 0x006907d0, 0x006902e0, 0x00692510}},
			{ 401652, { 0x02FC62C8, 0x02f013d8, 0x02F9BAF8, 0x00000000, 0x00000000, 0x02f9cc80, 0x00000000, 0x02f9b000, 0x02f95000, 0x0312bde0, 0x031390e0, 0x0313a4f0}},
			{ 0,      { 0x01198ba0, 0x0113E950, 0x00000000, 0x00000000, 0x00000000, 0x0119c660, 0x00000000, 0x011adc10, 0x011ad350, 0x01244590, 0x002f0740, 0x002f0570}}, // Update_1405B2FF0, after line 170 // call just before for 1.6.1130
		};

		constexpr auto GameStepTimer_SlowTime             = functionsOffsets.GameStepTimer_SlowTime            .V[CURRENTVERSION];
		constexpr auto GameStepTimer_RealTime             = functionsOffsets.GameStepTimer_RealTime            .V[CURRENTVERSION];
		constexpr auto ArmorAttachFunction                = functionsOffsets.ArmorAttachFunction               .V[CURRENTVERSION];
		constexpr auto ItemUnequipFunction				  = functionsOffsets.ItemUnequipFunction			   .V[CURRENTVERSION];
		constexpr auto BSFaceGenNiNode_SkinAllGeometry    = functionsOffsets.BSFaceGenNiNode_SkinAllGeometry   .V[CURRENTVERSION];
		constexpr auto BSFaceGenNiNode_SkinSingleGeometry = functionsOffsets.BSFaceGenNiNode_SkinSingleGeometry.V[CURRENTVERSION];
		constexpr auto GameLoopFunction                   = functionsOffsets.GameLoopFunction                  .V[CURRENTVERSION];
		constexpr auto GameShutdownFunction               = functionsOffsets.GameShutdownFunction              .V[CURRENTVERSION];
		constexpr auto TESNPC_GetFaceGeomPath             = functionsOffsets.TESNPC_GetFaceGeomPath            .V[CURRENTVERSION];
		constexpr auto BSFaceGenModelExtraData_BoneLimit  = functionsOffsets.BSFaceGenModelExtraData_BoneLimit .V[CURRENTVERSION];
		constexpr auto Actor_CalculateLOS                 = functionsOffsets.Actor_CalculateLOS                .V[CURRENTVERSION];
		constexpr auto SkyPointer                         = functionsOffsets.SkyPointer                        .V[CURRENTVERSION];
		constexpr auto FrameSyncPoint                     = functionsOffsets.FrameSyncPoint                    .V[CURRENTVERSION];

		// .text:00000001403D88D4                 cmp     ebx, 8
		// patch 8 -> 7
		// The same for AE/SE/VR.
		constexpr std::uintptr_t BSFaceGenNiNode_SkinSingleGeometry_bug = BSFaceGenNiNode_SkinSingleGeometry + 0x96;
	}
}
