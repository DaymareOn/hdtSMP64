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
	};

	/// Load a `<strand-hair>` config from an XML file, parsed with FSMP's own XML reader (same
	/// dialect as the SMP configs). Returns false and leaves `out` at defaults when the file is
	/// absent or empty. Every field is a trust boundary: unknown tags are ignored and all values
	/// are clamped to safe ranges before returning, so a malformed file can never destabilise the
	/// solver.
	bool loadStrandConfig(const std::string& path, StrandConfig& out);
}
