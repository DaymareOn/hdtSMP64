#pragma once

#include <string>

namespace hdt
{
	/// Author-editable strand-wig parameters. Defaults match the hard-coded PoC values, so a
	/// missing config file changes nothing. Groom shape + simulation only for now; render colour
	/// and width are the next config step.
	struct StrandConfig
	{
		int strandCount = 320;       // simulated guide strands per wig
		int vertsPerStrand = 12;     // beads per strand
		float length = 22.0f;        // base strand length, Skyrim units
		float lengthVariation = 0.25f;  // per-strand length spread, fraction of length (0..1)
		float stiffness = 0.18f;     // shape retention toward the rest pose (0..1)
		float damping = 0.92f;       // velocity retained each substep (0..1)
		// Render material: linear RGB at the root and tip, and world-space ribbon half-width base.
		float colorRoot[3] = { 0.05f, 0.03f, 0.02f };
		float colorTip[3] = { 0.14f, 0.09f, 0.05f };
		float width = 0.4f;          // ribbon half-width base, world units (x the render radius slider)
		// Authored groom: a TressFX .tfx file in FSMPWig/grooms/ supplying the strand geometry
		// (head-local, Skyrim units). Empty -> the built-in procedural scalp cap. groomScale is a
		// uniform multiplier to rescale authored units onto the Skyrim head.
		std::string groomFile;
		float groomScale = 1.0f;
		// Global gating policy (only read from the top-level wig.xml, not per-wig files): when true,
		// strand wigs attach only to actors wearing a hair/wig-slot armor (true equip-driven mode);
		// when false (default), to every SMP-active actor. Per-actor files (wigs/<actorFormID>.xml)
		// always attach regardless. Defaults false so the visible behaviour is unchanged out of the box.
		bool attachToWigArmorOnly = false;
	};

	/// Load a `<strand-hair>` config from an XML file, parsed with FSMP's own XML reader (same
	/// dialect as the SMP configs). Returns false and leaves `out` at defaults when the file is
	/// absent or empty. Every field is a trust boundary: unknown tags are ignored and all values
	/// are clamped to safe ranges before returning, so a malformed file can never destabilise the
	/// solver.
	bool loadStrandConfig(const std::string& path, StrandConfig& out);
}
