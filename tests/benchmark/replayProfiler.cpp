#include "replayProfiler.h"

#include <LinearMath/btQuickprof.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

namespace hdt::replay::prof
{
	ScopeNode& ScopeNode::child(std::string_view a_name)
	{
		for (auto& c : children) {
			if (c->name == a_name)
				return *c;
		}
		children.push_back(std::make_unique<ScopeNode>(std::string(a_name)));
		return *children.back();
	}
}

namespace
{
	using clock = std::chrono::steady_clock;
	using hdt::replay::prof::ScopeNode;

	struct OpenScope
	{
		ScopeNode* node;
		clock::time_point start;
	};

	// Synthetic root: its children are the top-level scopes. Never emitted itself.
	ScopeNode g_root{ "Root" };
	std::vector<OpenScope> g_stack;

	// btEnterProfileZoneFunc / btLeaveProfileZoneFunc signatures. Single-threaded by contract (the
	// profiling pass pins TBB to one worker), so plain globals are safe and add no locking overhead.

	/// Enter hook: descends into the current scope's child for `name` (creating it on first sight),
	/// bumps its call count, and pushes it with the entry time. Bound via btSetCustomEnterProfileZoneFunc.
	void enterZone(const char* name)
	{
		ScopeNode& parent = g_stack.empty() ? g_root : *g_stack.back().node;
		ScopeNode& node = parent.child(name ? name : "(null)");
		++node.calls;
		g_stack.push_back({ &node, clock::now() });
	}

	/// Leave hook: pops the current scope and adds its elapsed wall time (nanoseconds) to that node's
	/// inclusive total. Bound via btSetCustomLeaveProfileZoneFunc.
	void leaveZone()
	{
		if (g_stack.empty())
			return;
		const OpenScope o = g_stack.back();
		g_stack.pop_back();
		o.node->totalNs += static_cast<std::uint64_t>(
			std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - o.start).count());
	}

	/// One flat row for the text report: a scope name with its inclusive time and call count summed
	/// across every place that name appears in the tree (preserves the pre-nesting report semantics).
	struct FlatRow
	{
		std::uint64_t totalNs = 0;
		std::uint64_t calls = 0;
	};

	/// Walks the tree (excluding the synthetic root) and accumulates each name's inclusive time and
	/// calls into `out`, so the report still groups purely by name regardless of where a scope nested.
	void flattenByName(const ScopeNode& node, bool isRoot, std::unordered_map<std::string, FlatRow>& out)
	{
		if (!isRoot) {
			FlatRow& row = out[node.name];
			row.totalNs += node.totalNs;
			row.calls += node.calls;
		}
		for (const auto& c : node.children)
			flattenByName(*c, false, out);
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
		g_root.children.clear();
		g_root.totalNs = 0;
		g_root.calls = 0;
		g_stack.clear();
	}

	void report()
	{
		std::unordered_map<std::string, FlatRow> map;
		flattenByName(g_root, true, map);

		std::vector<std::pair<std::string, FlatRow>> rows(map.begin(), map.end());
		std::sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) {
			return a.second.totalNs > b.second.totalNs;
		});

		std::printf("  %-34s %12s %10s %12s\n", "scope", "total ms", "calls", "ms/call");
		for (const auto& r : rows) {
			const double totalMs = static_cast<double>(r.second.totalNs) / 1'000'000.0;
			const double perCall = r.second.calls ? totalMs / static_cast<double>(r.second.calls) : 0.0;
			std::printf("  %-34s %12.4f %10llu %12.6f\n",
				r.first.c_str(), totalMs,
				static_cast<unsigned long long>(r.second.calls), perCall);
		}
	}

	const ScopeNode& tree()
	{
		return g_root;
	}
}
