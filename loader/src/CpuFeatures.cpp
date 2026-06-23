#include "CpuFeatures.h"

#include <intrin.h>

namespace loader
{
	namespace
	{
		// Read extended control register XCR0 via XGETBV. XCR0 records which register files the OS
		// has agreed to save/restore across context switches; if a bit is clear the corresponding
		// instructions trap even when CPUID claims support. Index 0 selects XCR0 (the only register
		// we need); wrapped in a helper so the call site reads as intent, not magic numbers.
		unsigned long long readXcr0() noexcept
		{
			return _xgetbv(0);
		}
	}

	Variant detectHighestSupportedVariant() noexcept
	{
		int regs[4] = { 0, 0, 0, 0 };

		// Leaf 0 reports the highest standard leaf the CPU understands, so we don't blindly query
		// leaf 7 (AVX2/AVX-512 bits) on CPUs too old to have it.
		__cpuid(regs, 0);
		const int maxLeaf = regs[0];

		// Leaf 1, ECX: SSE/AVX/FMA feature bits plus OSXSAVE (bit 27), which tells us XGETBV is even
		// legal to execute. Without OSXSAVE we must assume the OS has enabled no extended state.
		__cpuid(regs, 1);
		const unsigned int ecx1 = static_cast<unsigned int>(regs[2]);
		const bool osxsave = (ecx1 & (1u << 27)) != 0;
		const bool cpuAvx = (ecx1 & (1u << 28)) != 0;
		const bool cpuFma = (ecx1 & (1u << 12)) != 0;

		bool osYmm = false;  // OS saves XMM (XCR0 bit 1) + YMM (bit 2): required for any AVX
		bool osZmm = false;  // additionally OPMASK (bit 5) + ZMM_Hi256 (bit 6) + Hi16_ZMM (bit 7)
		if (osxsave) {
			const unsigned long long xcr0 = readXcr0();
			osYmm = (xcr0 & 0x6u) == 0x6u;
			osZmm = osYmm && (xcr0 & 0xE0u) == 0xE0u;
		}

		// No usable AVX at all -> the plain (NoAvx) build is the only safe choice.
		if (!(cpuAvx && osYmm)) {
			return Variant::NoAvx;
		}

		bool cpuAvx2 = false;
		bool cpuAvx512 = false;
		if (maxLeaf >= 7) {
			__cpuidex(regs, 7, 0);
			const unsigned int ebx7 = static_cast<unsigned int>(regs[1]);
			cpuAvx2 = (ebx7 & (1u << 5)) != 0;

			// MSVC /arch:AVX512 may emit instructions from F, CD, DQ, BW and VL, so require them all
			// before trusting the AVX-512 build; F alone is not enough.
			const bool f = (ebx7 & (1u << 16)) != 0;
			const bool dq = (ebx7 & (1u << 17)) != 0;
			const bool cd = (ebx7 & (1u << 28)) != 0;
			const bool bw = (ebx7 & (1u << 30)) != 0;
			const bool vl = (ebx7 & (1u << 31)) != 0;
			cpuAvx512 = f && dq && cd && bw && vl;
		}

		if (cpuAvx512 && osZmm) {
			return Variant::Avx512;
		}
		if (cpuAvx2 && cpuFma) {
			return Variant::Avx2;
		}
		return Variant::Avx;
	}
}
