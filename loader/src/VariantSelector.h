#pragma once

#include "AvxVariant.h"

#include <filesystem>
#include <optional>
#include <vector>

namespace loader
{
	// The decision of which variants to try, in order. Produced by buildSelection and consumed by
	// the load loop in main.cpp.
	struct Selection
	{
		std::vector<Variant> candidates;  // most-preferred first; always ends with NoAvx
		Variant start{ Variant::NoAvx };  // the first/preferred candidate (for logging)
	};

	// Outcome of reading the override INI's ForceVariant key.
	struct ForceResult
	{
		std::optional<Variant> variant;  // a pinned tier, or nullopt for "auto"/missing/garbage
		bool explicitChoice{ false };    // true when the key was present and meaningful (variant or
										 // the literal "auto") -> the crash ceiling should be reset
	};

	// Read the optional override INI and extract `ForceVariant`. The file crosses a trust boundary
	// (user- or MCM-editable), so parsing is defensive: comments (';'/'#') and unknown keys are
	// skipped, the value is trimmed and lowercased, and only the known tokens (auto/noavx/avx/avx2/
	// avx512) are honoured. A missing file or unparsable value is treated as "auto".
	ForceResult readForcedVariant(const std::filesystem::path& iniFile) noexcept;

	// Build the ordered list of variants to attempt. The starting tier is:
	//   - the forced variant, clamped down to what the CPU actually supports (we never execute an
	//     ISA the hardware lacks, even when explicitly pinned), or
	//   - when not forced, the lower of the CPU maximum and the persisted crash ceiling.
	// From the start tier the list walks downward through every lower tier to NoAvx, so a candidate
	// that fails to load always has a safer fallback and physics can never be left entirely off.
	Selection buildSelection(Variant cpuMax, Variant ceiling, std::optional<Variant> forced);
}
