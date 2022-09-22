#pragma once

#include <cstdint>

//function			                 1.5.97        1.6.318 	     id        1.6.323      1.6.342      1.6.353      1.6.629		1.6.640
//GameStepTimer_SlowTime             0x02F6B948    0x030064C8    410199    0x030064c8   0x03007708   0x03007708   0x03006808	0x03006808
//ArmorAttachFunction                0x001CAFB0    0x001D6740    15712     0x001d66b0   0x001d66a0   0x001d66a0   0x001d83b0	0x001d83b0
//BSFaceGenNiNode_SkinAllGeometry    0x003D87B0    0x003F08C0    26986     0x003f0830   0x003f09c0   0x003f0830   0x003f2990	0x003f2990
//BSFaceGenNiNode_SkinSingleGeometry 0x003D8840    0x003F0A50    26987     0x003f09c0   0x003f0b50   0x003f09c0   0x003f2b20	0x003f2b20
//GameLoopFunction                   0x005B2FF0    0x005D9F50    36564     0x005D9CC0   0x005dae80   0x005dace0   0x005ec310	0x005ec240
//GameShutdownFunction               0x01293D20    0x013B9A90    105623    0x013b99f0   0x013ba910   0x013ba9a0   0x013b8230	0x013b8160
//TESNPC_GetFaceGeomPath             0x00363210    0x0037A240    24726     0x0037a1b0   0x0037a340   0x0037a1b0   0x0037c1e0    0x0037c1e0
//BSFaceGenModelExtraData_BoneLimit  0x0036B4C8

namespace hdt
{
	// signatures generated with IDA SigMaker plugin for SSE
	// comment is SSE, uncommented is VR. VR address found by Ghidra compare using known SSE offset to find same function
	// (based on function signature and logic)
	namespace offset
	{
#ifndef SKYRIMVR
		// hdtSkyrimPhysicsWorld.cpp
		// 74 35 45 33 C0 33 D2
#ifdef ANNIVERSARY_EDITION
		constexpr std::uintptr_t GameStepTimer_SlowTime = 0x03006808;
#else
		constexpr std::uintptr_t GameStepTimer_SlowTime = 0x02F6B948;
#endif

		// Hooks.cpp
		// E8 ? ? ? ? 48 8B E8 FF C7
#ifdef ANNIVERSARY_EDITION
		constexpr std::uintptr_t ArmorAttachFunction = 0x001d83b0;
#else
		constexpr std::uintptr_t ArmorAttachFunction = 0x001CAFB0;
#endif

		// BSFaceGenNiNode last vfunc
#ifdef ANNIVERSARY_EDITION
		constexpr std::uintptr_t BSFaceGenNiNode_SkinAllGeometry = 0x003f2990;
		constexpr std::uintptr_t BSFaceGenNiNode_SkinSingleGeometry = 0x003f2b20;
#else
		constexpr std::uintptr_t BSFaceGenNiNode_SkinAllGeometry = 0x003D87B0;
		constexpr std::uintptr_t BSFaceGenNiNode_SkinSingleGeometry = 0x003D8840;
#endif

		// Hooks.cpp
		// function responsible for majority of main game thread loop
#ifdef ANNIVERSARY_EDITION
		// E8 ? ? ? ? 84 DB 74 24
		constexpr std::uintptr_t GameLoopFunction = 0x005ec240;
		// E8 ? ? ? ? E8 ? ? ? ? E8 ? ? ? ? 48 8B 0D ? ? ? ? 48 85 C9 74 0C E8 ? ? ? ?
		constexpr std::uintptr_t GameShutdownFunction = 0x013b8160;
#else
		// E8 ? ? ? ? 84 DB 74 24
		constexpr std::uintptr_t GameLoopFunction = 0x005B2FF0;
		// E8 ? ? ? ? E8 ? ? ? ? E8 ? ? ? ? 48 8B 0D ? ? ? ? 48 85 C9 74 0C E8 ? ? ? ?
		constexpr std::uintptr_t GameShutdownFunction = 0x01293D20;
#endif

		// FaceGeom string
#ifdef ANNIVERSARY_EDITION
		constexpr std::uintptr_t TESNPC_GetFaceGeomPath = 0x0037c1e0;
#else
		constexpr std::uintptr_t TESNPC_GetFaceGeomPath = 0x00363210;
#endif

		// BSFaceGenModelExtraData Bone Limit
		// 8B 70 58 EB 02
		constexpr std::uintptr_t BSFaceGenModelExtraData_BoneLimit = 0x0036B4C8;

#else
		constexpr std::uintptr_t GameStepTimer_SlowTime = 0x030C3A08;
		constexpr std::uintptr_t ArmorAttachFunction = 0x001DB9E0;
		constexpr std::uintptr_t BSFaceGenNiNode_SkinAllGeometry = 0x003e8120;
		constexpr std::uintptr_t BSFaceGenNiNode_SkinSingleGeometry = 0x003e81b0;
		constexpr std::uintptr_t GameLoopFunction = 0x005BAB10;
		constexpr std::uintptr_t GameShutdownFunction = 0x012CC630;
		constexpr std::uintptr_t TESNPC_GetFaceGeomPath = 0x000372b30;
		constexpr std::uintptr_t BSFaceGenModelExtraData_BoneLimit = 0x00037ae28;
#endif

		// .text:00000001403D88D4                 cmp     ebx, 8
		// patch 8 -> 7
		// The same for AE/SE/VR.
		constexpr std::uintptr_t BSFaceGenNiNode_SkinSingleGeometry_bug = BSFaceGenNiNode_SkinSingleGeometry + 0x96;
	}
}
