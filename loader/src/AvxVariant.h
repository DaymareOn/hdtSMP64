#pragma once

#include <array>
#include <string_view>

namespace loader
{
	// The CPU instruction-set tiers we ship an optimized build for, ordered from least to most
	// capable. The integer value doubles as a ranking: a higher value needs a more capable CPU and
	// is preferred whenever the hardware (and the persisted crash ceiling) allow it. Comparisons
	// such as `a < b` therefore mean "tier a is less advanced than tier b".
	enum class Variant : int
	{
		NoAvx = 0,
		Avx = 1,
		Avx2 = 2,
		Avx512 = 3,
	};

	// Every variant, highest preference first. Used to walk down from a starting tier to the
	// guaranteed-safe NoAvx fallback when building the candidate list.
	inline constexpr std::array<Variant, 4> kVariantsHighToLow{
		Variant::Avx512,
		Variant::Avx2,
		Variant::Avx,
		Variant::NoAvx
	};

	// The lowercase token used both in the variant DLL filename (hdtSMP64_<token>.dll) and in the
	// override INI / crash-state files. Kept in one place so the build, the loader, and the docs
	// can never disagree about spelling.
	constexpr std::string_view variantToken(Variant v) noexcept
	{
		switch (v) {
		case Variant::Avx512:
			return "avx512";
		case Variant::Avx2:
			return "avx2";
		case Variant::Avx:
			return "avx";
		case Variant::NoAvx:
		default:
			return "noavx";
		}
	}

	// Turn a token back into a Variant. The input is expected to be already trimmed and lowercased
	// (it crosses a trust boundary: it comes from a user-editable INI or an on-disk state file), so
	// anything unrecognized is rejected by returning false and leaving `out` untouched.
	inline bool variantFromToken(std::string_view token, Variant& out) noexcept
	{
		if (token == "avx512") {
			out = Variant::Avx512;
		} else if (token == "avx2") {
			out = Variant::Avx2;
		} else if (token == "avx") {
			out = Variant::Avx;
		} else if (token == "noavx") {
			out = Variant::NoAvx;
		} else {
			return false;
		}
		return true;
	}

	// The next-lower tier, used when a crash forces us to step down. NoAvx is the floor and maps to
	// itself: if even NoAvx is implicated there is nothing safer to fall back to.
	constexpr Variant oneTierLower(Variant v) noexcept
	{
		switch (v) {
		case Variant::Avx512:
			return Variant::Avx2;
		case Variant::Avx2:
			return Variant::Avx;
		case Variant::Avx:
		case Variant::NoAvx:
		default:
			return Variant::NoAvx;
		}
	}
}
