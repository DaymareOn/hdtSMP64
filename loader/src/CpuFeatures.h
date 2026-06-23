#pragma once

#include "AvxVariant.h"

namespace loader
{
	// Probe the running CPU *and* the OS and return the highest instruction-set tier that is safe to
	// actually execute. Two checks are required for each tier, not one:
	//   1. The CPUID feature bits must advertise the instructions.
	//   2. The OS must have enabled the matching register state (queried via XGETBV/XCR0). A feature
	//      bit set while the OS has not enabled YMM/ZMM state means the instructions still fault with
	//      an illegal-instruction (#UD) — so the OS check is what makes auto-selection crash-safe.
	//
	// The tiers also encode the requirements of our actual builds, not just the bare ISA name:
	//   - Avx2 additionally requires FMA, because the hand-written AVX2 path uses _mm_fmadd_ps.
	//   - Avx512 requires the F+CD+DQ+BW+VL subsets, the full surface MSVC's /arch:AVX512 may emit.
	Variant detectHighestSupportedVariant() noexcept;
}
