#pragma once

// Build information captured at compile time
// This file provides macros and functions to report build configuration

#include "Offsets.h"

namespace hdt
{
	namespace BuildInfo
	{
		// Game version string
		constexpr const char* GetGameVersionString()
		{
#if CURRENTVERSION == V1_4_15
			return "VR 1.4.15";
#elif CURRENTVERSION == V1_5_97
			return "SE 1.5.97";
#elif CURRENTVERSION == V1_6_318
			return "AE 1.6.318";
#elif CURRENTVERSION == V1_6_323
			return "AE 1.6.323";
#elif CURRENTVERSION == V1_6_342
			return "AE 1.6.342";
#elif CURRENTVERSION == V1_6_353
			return "AE 1.6.353";
#elif CURRENTVERSION == V1_6_629
			return "AE 1.6.629";
#elif CURRENTVERSION == V1_6_640
			return "AE 1.6.640";
#elif CURRENTVERSION == V1_6_659
			return "AE 1.6.659";
#elif CURRENTVERSION == V1_6_1130
			return "AE 1.6.1130";
#elif CURRENTVERSION == V1_6_1170
			return "AE 1.6.1170";
#elif CURRENTVERSION == V1_6_1179
			return "AE 1.6.1179";
#else
			return "Unknown";
#endif
		}

		// CUDA status
		constexpr const char* GetCudaStatus()
		{
#ifdef CUDA
			return "CUDA";
#else
			return "NOCUDA";
#endif
		}

		// AVX level - detected from compiler flags
		constexpr const char* GetAVXLevel()
		{
#if defined(__AVX512F__) || defined(__AVX512__)
			return "AVX512";
#elif defined(__AVX2__)
			return "AVX2";
#elif defined(__AVX__)
			return "AVX";
#else
			return "NoAVX";
#endif
		}

		// Build date and time
		constexpr const char* GetBuildDate() { return __DATE__; }
		constexpr const char* GetBuildTime() { return __TIME__; }

		// Compiler info
		constexpr const char* GetCompilerInfo()
		{
#if defined(_MSC_VER)
#if _MSC_VER >= 1940
			return "MSVC 19.4x (VS2022 17.10+)";
#elif _MSC_VER >= 1930
			return "MSVC 19.3x (VS2022)";
#elif _MSC_VER >= 1920
			return "MSVC 19.2x (VS2019)";
#else
			return "MSVC (older)";
#endif
#else
			return "Unknown compiler";
#endif
		}

		// Build type
		constexpr const char* GetBuildType()
		{
#if defined(_DEBUG) || defined(DEBUG)
			return "Debug";
#else
			return "Release";
#endif
		}
	}
}
