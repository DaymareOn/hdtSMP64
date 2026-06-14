#include "replayProfiler.h"

#include <LinearMath/btQuickprof.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
	using clock = std::chrono::steady_clock;

	struct OpenScope
	{
		const char* name;
		clock::time_point start;
	};

	struct ScopeStat
	{
		double totalMs = 0.0;
		std::uint64_t calls = 0;
	};

	std::vector<OpenScope> g_stack;
	std::unordered_map<std::string, ScopeStat> g_stats;

	// btEnterProfileZoneFunc / btLeaveProfileZoneFunc signatures. Single-threaded by contract (the
	// profiling pass pins TBB to one worker), so plain globals are safe and add no locking overhead.

	/// Enter hook: pushes a scope (name + start time) onto the stack. Bound via btSetCustomEnterProfileZoneFunc.
	void enterZone(const char* name)
	{
		g_stack.push_back({ name, clock::now() });
	}

	/// Leave hook: pops the current scope and adds its elapsed time to that name's running total.
	void leaveZone()
	{
		if (g_stack.empty())
			return;
		const OpenScope o = g_stack.back();
		g_stack.pop_back();
		const double ms = std::chrono::duration<double, std::milli>(clock::now() - o.start).count();
		ScopeStat& s = g_stats[o.name ? o.name : "(null)"];
		s.totalMs += ms;
		++s.calls;
	}
}

namespace hdt::replay::prof
{
	void install()
	{
		btSetCustomEnterProfileZoneFunc(enterZone);
		btSetCustomLeaveProfileZoneFunc(leaveZone);
		btSetProfileEnabled(true);
	}

	void uninstall()
	{
		btSetProfileEnabled(false);
	}

	void reset()
	{
		g_stack.clear();
		g_stats.clear();
	}

	void report()
	{
		std::vector<std::pair<std::string, ScopeStat>> rows(g_stats.begin(), g_stats.end());
		std::sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) {
			return a.second.totalMs > b.second.totalMs;
		});

		std::printf("  %-34s %12s %10s %12s\n", "scope", "total ms", "calls", "ms/call");
		for (const auto& r : rows) {
			const double perCall = r.second.calls ? r.second.totalMs / static_cast<double>(r.second.calls) : 0.0;
			std::printf("  %-34s %12.4f %10llu %12.6f\n",
				r.first.c_str(), r.second.totalMs,
				static_cast<unsigned long long>(r.second.calls), perCall);
		}
	}
}
