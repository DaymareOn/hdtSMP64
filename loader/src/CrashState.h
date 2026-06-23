#pragma once

#include "AvxVariant.h"

#include <filesystem>
#include <optional>

namespace loader
{
	// Persisted "safe mode" state that survives across game launches so a variant which destabilizes
	// the game is not retried forever. It is the mechanism behind the requested behaviour "fall back
	// to a lesser AVX if the current one crashes" for crashes that happen *after* load (mid-game),
	// which in-process exception handling cannot catch.
	//
	// A single tiny text file holds three facts:
	//   - ceiling: the highest variant we are still willing to attempt. Lowered one tier when a
	//              variant is judged guilty; never raised automatically (only an explicit user choice
	//              or deleting the file resets it).
	//   - attempt: the variant we are about to load this session. Written *before* the load and
	//              cleared once the session has run stably for a short window. If, on the next
	//              launch, an attempt is still recorded, the previous session ended before reaching
	//              stability.
	//   - strikes: how many consecutive sessions ended before stability. A single early exit (e.g.
	//              the user alt-F4s at the main menu) is NOT a crash, so one strike does not demote
	//              anything; only when strikes reach the limit do we blame the variant and lower the
	//              ceiling. This keeps clean quick-quits from silently downgrading a working build.
	//
	// (The in-process SEH path uses blame() instead, which demotes immediately because a hardware
	// fault during load is unambiguous - no strike accounting needed.)
	//
	// Threading: every method except markStable() runs on the SKSE load thread during
	// SKSEPlugin_Load and they run in a fixed sequence (load -> consumePendingCrash -> beginAttempt).
	// markStable() runs later on a single detached timer thread after SKSEPlugin_Load has returned,
	// so it never overlaps the others; no locking is required.
	class CrashState
	{
	public:
		// Outcome of inspecting the previous session, for logging.
		enum class Check
		{
			CleanLastSession,  // last session shut down stably (or there was no prior session)
			StrikeRecorded,    // last session ended early; counted, but ceiling left unchanged
			CeilingLowered     // strikes reached the limit; the variant was blamed and demoted
		};

		explicit CrashState(std::filesystem::path file);

		// Read the state file into memory. A missing or malformed file is not an error: it simply
		// yields the defaults (ceiling = highest tier, no pending attempt, no strikes).
		void load();

		// Inspect whether the previous session left a pending attempt (ended before stability). If so
		// record a strike and, once the strike limit is reached, lower the ceiling one tier below the
		// blamed variant and reset the strike count. Always clears the stale attempt and persists.
		Check consumePendingCrash();

		// Highest variant still permitted by past crash history (Avx512 if nothing was ever blamed).
		Variant ceiling() const noexcept { return m_ceiling; }

		// Record that we are about to load `v`, and flush to disk immediately so that a hard crash
		// during the variant's load or first seconds of running is observable on the next launch.
		void beginAttempt(Variant v);

		// Declare the current attempt stable: drop the pending attempt, reset the strike count, and
		// persist. Called from the stability timer once the process has survived the watch window.
		void markStable();

		// Permanently blame a variant whose load/init faulted in-process (the SEH path), capping the
		// ceiling one tier below it and clearing any pending attempt and strikes. This both lets the
		// current session fall through to a lower variant and stops future launches from retrying the
		// bad tier. Persists.
		void blame(Variant v);

		// Forget all crash history (ceiling back to the maximum, no pending attempt, no strikes).
		// Used when the user makes an explicit choice (forces a variant, or selects auto in the MCM),
		// so an old crash no longer constrains a deliberate decision.
		void resetCeiling();

	private:
		// Consecutive unstable sessions required before a variant is demoted. Two means a single
		// clean early-exit is forgiven; a genuinely crashing build still trips it on the next launch.
		static constexpr int kStrikeLimit = 2;

		void save() const;

		std::filesystem::path m_file;
		Variant m_ceiling{ Variant::Avx512 };
		std::optional<Variant> m_attempt;
		int m_strikes{ 0 };
	};
}
