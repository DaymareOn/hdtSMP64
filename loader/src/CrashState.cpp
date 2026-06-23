#include "CrashState.h"

#include <charconv>
#include <fstream>
#include <string>
#include <string_view>

namespace loader
{
	namespace
	{
		// Trim ASCII whitespace from both ends of a view. The state file is machine-written but may
		// be touched by a user, so we stay lenient about stray spaces around the values.
		std::string_view trim(std::string_view s) noexcept
		{
			const auto notSpace = [](char c) { return c != ' ' && c != '\t' && c != '\r' && c != '\n'; };
			while (!s.empty() && !notSpace(s.front())) {
				s.remove_prefix(1);
			}
			while (!s.empty() && !notSpace(s.back())) {
				s.remove_suffix(1);
			}
			return s;
		}
	}

	CrashState::CrashState(std::filesystem::path file) :
		m_file(std::move(file))
	{
	}

	void CrashState::load()
	{
		// Reset to defaults first so a partial/garbage file degrades to "no history" rather than to
		// stale in-memory values.
		m_ceiling = Variant::Avx512;
		m_attempt.reset();
		m_strikes = 0;

		std::ifstream ifs(m_file);
		if (!ifs) {
			return;
		}

		std::string line;
		while (std::getline(ifs, line)) {
			const auto eq = line.find('=');
			if (eq == std::string::npos) {
				continue;
			}
			const std::string_view key = trim(std::string_view(line).substr(0, eq));
			const std::string_view value = trim(std::string_view(line).substr(eq + 1));

			if (key == "strikes") {
				// A small non-negative integer; anything unparsable stays 0.
				int n = 0;
				const auto [ptr, errc] = std::from_chars(value.data(), value.data() + value.size(), n);
				if (errc == std::errc{} && n >= 0) {
					m_strikes = n;
				}
				continue;
			}

			Variant parsed{};
			if (!variantFromToken(value, parsed)) {
				continue;  // unknown token -> ignore the line, keep the default
			}
			if (key == "ceiling") {
				m_ceiling = parsed;
			} else if (key == "attempt") {
				m_attempt = parsed;
			}
		}
	}

	CrashState::Check CrashState::consumePendingCrash()
	{
		if (!m_attempt.has_value()) {
			// Last session cleared its attempt -> it reached stability. A clean run resets strikes.
			if (m_strikes != 0) {
				m_strikes = 0;
				save();
			}
			return Check::CleanLastSession;
		}

		// The recorded attempt was never marked stable -> the previous session ended before the
		// stability window. That may be a crash or just an early quit, so count it as a strike rather
		// than demoting immediately.
		const Variant blamed = *m_attempt;
		++m_strikes;
		m_attempt.reset();

		if (m_strikes < kStrikeLimit) {
			save();
			return Check::StrikeRecorded;
		}

		// Enough consecutive unstable sessions: blame the variant. Cap the ceiling one step below it,
		// keeping the lower value if it was already demoted further, then reset the strike count.
		const Variant lowered = oneTierLower(blamed);
		if (lowered < m_ceiling) {
			m_ceiling = lowered;
		}
		m_strikes = 0;
		save();
		return Check::CeilingLowered;
	}

	void CrashState::beginAttempt(Variant v)
	{
		m_attempt = v;
		save();
	}

	void CrashState::markStable()
	{
		m_attempt.reset();
		m_strikes = 0;
		save();
	}

	void CrashState::blame(Variant v)
	{
		const Variant lowered = oneTierLower(v);
		if (lowered < m_ceiling) {
			m_ceiling = lowered;
		}
		m_attempt.reset();
		m_strikes = 0;
		save();
	}

	void CrashState::resetCeiling()
	{
		m_ceiling = Variant::Avx512;
		m_attempt.reset();
		m_strikes = 0;
		save();
	}

	void CrashState::save() const
	{
		std::error_code ec;
		std::filesystem::create_directories(m_file.parent_path(), ec);

		std::ofstream ofs(m_file, std::ios::out | std::ios::trunc);
		if (!ofs) {
			return;  // best effort: if we cannot persist, crash-fallback simply won't engage
		}
		ofs << "ceiling=" << variantToken(m_ceiling) << '\n';
		ofs << "strikes=" << m_strikes << '\n';
		if (m_attempt.has_value()) {
			ofs << "attempt=" << variantToken(*m_attempt) << '\n';
		}
	}
}
