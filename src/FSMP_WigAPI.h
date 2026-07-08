#pragma once

// Public C-ABI that FSMP (hdtsmp64.dll) exports so a renderer -- e.g. a Community Shaders
// feature -- can pull simulated wig strands each frame. No import library is required:
// resolve the FSMPWig_* symbols with GetProcAddress on hdtsmp64.dll, mirroring the
// DynamicWetness / ENB integration style. All positions are world-space, Skyrim units
// (Z up, same axes the game renders in), laid out strand-major (strand s bead v at
// index s*vertsPerStrand + v).

#include <cstdint>

#ifdef FSMPWIG_EXPORTS
#	define FSMPWIG_API __declspec(dllexport)
#else
#	define FSMPWIG_API  // consumers resolve via GetProcAddress; no import decoration
#endif

extern "C"
{
	/// One published wig instance. headWorld is the anchor (head) node's world transform as a
	/// row-major 4x4; colorRoot/colorTip are linear RGB for a simple root-to-tip gradient.
	struct FSMPWigInstanceDesc
	{
		std::uint32_t instanceId;
		std::uint32_t strandCount;
		std::uint32_t vertsPerStrand;
		float headWorld[16];
		float colorRoot[3];
		float colorTip[3];
		float roughness;
		float strandRadius;  // world units
	};

	/// Data-format version; bump on any layout change to the structs or the position stream.
	FSMPWIG_API std::uint32_t FSMPWig_GetVersion(void);

	/// Turn the strand simulation on or off (0/1). Off by default until a renderer opts in
	/// (or the marker file Data/SKSE/Plugins/FSMPWig/enable.txt exists at load).
	FSMPWIG_API void FSMPWig_SetEnabled(std::uint32_t enabled);

	/// Number of wig instances currently published (0 if disabled or nothing is bound yet).
	FSMPWIG_API std::uint32_t FSMPWig_GetInstanceCount(void);

	/// Fill outDesc for the instance at index. Returns 1 on success, 0 if index is invalid.
	FSMPWIG_API std::uint32_t FSMPWig_GetInstance(std::uint32_t index, FSMPWigInstanceDesc* outDesc);

	/// Copy world-space bead positions (3 floats each, strand-major) for the instance at index
	/// into dst, writing at most dstCapacityFloats floats. Returns the number of floats written
	/// (0 if index invalid). Safe to call from the render thread: it reads a locked snapshot.
	FSMPWIG_API std::uint32_t FSMPWig_CopyPositions(std::uint32_t index, float* dst, std::uint32_t dstCapacityFloats);
}
