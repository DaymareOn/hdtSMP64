#include "hdtPhysicsProfiler.h"

#include "SMPDebug.h"  // menuConsoleAppendLines --- profile tables mirror into the menu's Output panel

#include <LinearMath/btQuickprof.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hdt::physicsprofiler
{
	using Clock = std::chrono::steady_clock;

	namespace
	{
		// One profiler-output line: to the log (as before) AND to the menu's Output panel, so the profile
		// tables themselves are readable in the configuration menu without opening the log file.
		template <typename... Args>
		void plog(fmt::format_string<Args...> a_fmt, Args&&... a_args)
		{
			std::string line = fmt::format(a_fmt, std::forward<Args>(a_args)...);
			logger::info("{}", line);
			hdt::menuConsoleAppendLines(line);
		}

		struct DumpOptions
		{
			double m_minScopeMs = 0.001;
			std::uint32_t m_maxDepth = 64;
			std::uint32_t m_flatLimit = 32;
			bool m_logTree = true;
			bool m_logFlat = true;
			bool m_logThreads = true;
		};

		struct Node
		{
			std::string m_name;
			std::uint64_t m_totalNs{};
			std::uint64_t m_calls{};
			std::vector<std::unique_ptr<Node>> m_children;

			explicit Node(std::string a_name) :
				m_name(std::move(a_name))
			{}

			Node& child(std::string_view a_name)
			{
				if (auto it = std::ranges::find_if(m_children, [&](const auto& a_entry) { return a_entry->m_name == a_name; }); it != m_children.end()) {
					return **it;
				}

				m_children.emplace_back(std::make_unique<Node>(std::string(a_name)));
				return *m_children.back();
			}

			void resetStats()
			{
				m_totalNs = 0;
				m_calls = 0;

				for (auto& child : m_children) {
					child->resetStats();
				}
			}
		};

		struct Frame
		{
			Node* m_node{};
			Clock::time_point m_start{};
		};

		struct ThreadState
		{
			std::uint32_t m_index{};
			Node m_root{ "Root" };
			std::vector<Frame> m_stack;

			explicit ThreadState(std::uint32_t a_index) :
				m_index(a_index)
			{}
		};

		struct AggregateNode
		{
			std::string m_name;
			std::uint64_t m_totalNs{};
			std::uint64_t m_calls{};
			std::vector<std::unique_ptr<AggregateNode>> m_children;

			explicit AggregateNode(std::string a_name) :
				m_name(std::move(a_name))
			{}

			AggregateNode& child(std::string_view a_name)
			{
				if (auto it = std::ranges::find_if(m_children, [&](const auto& a_entry) { return a_entry->m_name == a_name; }); it != m_children.end()) {
					return **it;
				}

				m_children.emplace_back(std::make_unique<AggregateNode>(std::string(a_name)));
				return *m_children.back();
			}
		};

		struct FlatRow
		{
			std::string m_name;
			std::uint64_t m_totalNs{};
			std::uint64_t m_calls{};
		};

		struct ThreadRow
		{
			std::uint32_t m_index{};
			std::uint64_t m_totalNs{};
			std::uint64_t m_calls{};
		};

		std::mutex g_threadsLock;
		std::vector<std::unique_ptr<ThreadState>> g_threads;
		std::atomic_uint32_t g_nextThreadIndex{ 0 };
		std::atomic_uint32_t g_activeScopes{ 0 };
		std::uint64_t g_windowFrames{ 0 };
		std::uint64_t g_totalFrames{ 0 };
		std::uint64_t g_sampleWindowFrames{ 0 };
		bool g_captureEnabled = false;
		std::uint64_t g_printIntervalFrames = 0;
		Clock::time_point g_windowStart = Clock::now();

		thread_local ThreadState* g_threadState = nullptr;

		void enter(const char* a_name) noexcept;
		void leave() noexcept;

		void emptyEnter(const char*) noexcept
		{}

		void emptyLeave() noexcept
		{}

		double nsToMs(std::uint64_t a_ns)
		{
			return static_cast<double>(a_ns) / 1'000'000.0;
		}

		std::uint64_t elapsedNs(Clock::time_point a_start, Clock::time_point a_end)
		{
			return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(a_end - a_start).count());
		}

		ThreadState& getThreadState()
		{
			if (g_threadState) {
				return *g_threadState;
			}

			auto threadState = std::make_unique<ThreadState>(g_nextThreadIndex.fetch_add(1, std::memory_order_relaxed));
			g_threadState = threadState.get();

			std::scoped_lock lock(g_threadsLock);
			g_threads.emplace_back(std::move(threadState));

			return *g_threadState;
		}

		std::uint64_t getChildTotal(const Node& a_node)
		{
			std::uint64_t total = 0;

			for (const auto& child : a_node.m_children) {
				total += child->m_totalNs;
			}

			return total;
		}

		std::uint64_t getChildCalls(const Node& a_node)
		{
			std::uint64_t calls = 0;

			for (const auto& child : a_node.m_children) {
				calls += child->m_calls;
			}

			return calls;
		}

		std::uint64_t getChildTotal(const AggregateNode& a_node)
		{
			std::uint64_t total = 0;

			for (const auto& child : a_node.m_children) {
				total += child->m_totalNs;
			}

			return total;
		}

		void mergeNode(AggregateNode& a_dst, const Node& a_src)
		{
			a_dst.m_totalNs += a_src.m_totalNs;
			a_dst.m_calls += a_src.m_calls;

			for (const auto& child : a_src.m_children) {
				mergeNode(a_dst.child(child->m_name), *child);
			}
		}

		void mergeRoot(AggregateNode& a_dst, const Node& a_root)
		{
			for (const auto& child : a_root.m_children) {
				mergeNode(a_dst.child(child->m_name), *child);
			}
		}

		void addFlatRows(std::unordered_map<std::string, FlatRow>& a_rows, const AggregateNode& a_node)
		{
			if (a_node.m_name != "Recorded CPU") {
				auto& row = a_rows[a_node.m_name];
				row.m_name = a_node.m_name;
				row.m_totalNs += a_node.m_totalNs;
				row.m_calls += a_node.m_calls;
			}

			for (const auto& child : a_node.m_children) {
				addFlatRows(a_rows, *child);
			}
		}

		void logRow(std::string_view a_scope, std::uint64_t a_ns, std::uint64_t a_parentNs, std::uint64_t a_calls, std::uint64_t a_frames)
		{
			const auto totalMs = nsToMs(a_ns);
			const auto msPerFrame = a_frames > 0 ? totalMs / static_cast<double>(a_frames) : 0.0;
			const auto msPerCall = a_calls > 0 ? totalMs / static_cast<double>(a_calls) : 0.0;
			const auto pct = a_parentNs > 0 ? static_cast<double>(a_ns) * 100.0 / static_cast<double>(a_parentNs) : 0.0;

			if (a_calls > 0) {
				plog("{:<80} {:>12.3f} ms {:>10.3f} ms/frame {:>10.3f} ms/call {:>8.2f}% {:>8}", a_scope, totalMs, msPerFrame, msPerCall, pct, a_calls);
			} else {
				plog("{:<80} {:>12.3f} ms {:>10.3f} ms/frame {:>17} {:>8.2f}% {:>8}", a_scope, totalMs, msPerFrame, "-", pct, "-");
			}
		}

		void logTreeRecursive(const AggregateNode& a_node, std::string a_prefix, std::uint32_t a_depth, std::uint64_t a_frames, const DumpOptions& a_options)
		{
			if (a_depth >= a_options.m_maxDepth) {
				return;
			}

			std::vector<const AggregateNode*> children;
			children.reserve(a_node.m_children.size());

			for (const auto& child : a_node.m_children) {
				if (nsToMs(child->m_totalNs) >= a_options.m_minScopeMs) {
					children.emplace_back(child.get());
				}
			}

			std::ranges::sort(children, [](const auto* a_lhs, const auto* a_rhs) {
				return a_lhs->m_totalNs > a_rhs->m_totalNs;
			});

			for (std::size_t i = 0; i < children.size(); ++i) {
				const auto* child = children[i];
				const bool last = i + 1 == children.size();
				const auto scope = a_prefix + (last ? "`-- " : "|-- ") + child->m_name;

				logRow(scope, child->m_totalNs, a_node.m_totalNs, child->m_calls, a_frames);
				logTreeRecursive(*child, a_prefix + (last ? "    " : "|   "), a_depth + 1, a_frames, a_options);
			}

			if (a_node.m_name == "Recorded CPU") {
				return;
			}

			const auto accounted = getChildTotal(a_node);

			if (a_node.m_totalNs > accounted) {
				const auto self = a_node.m_totalNs - accounted;

				if (nsToMs(self) >= a_options.m_minScopeMs) {
					logRow(a_prefix + "`-- [self]", self, a_node.m_totalNs, 0, a_frames);
				}
			}
		}

		void logTreeHeader(const AggregateNode& a_root, std::uint64_t a_frames, Clock::time_point a_windowStart)
		{
			const auto wallNs = elapsedNs(a_windowStart, Clock::now());
			const auto wallMs = nsToMs(wallNs);
			const auto cpuMs = nsToMs(a_root.m_totalNs);
			const auto parallelism = wallNs > 0 ? static_cast<double>(a_root.m_totalNs) / static_cast<double>(wallNs) : 0.0;

			plog("Physics profile: {} frames, {:.3f} ms wall, {:.3f} ms recorded CPU, {:.2f}x recorded parallelism", a_frames, wallMs, cpuMs, parallelism);
			plog("{:<80} {:>16} {:>18} {:>18} {:>9} {:>8}", "Scope", "Total CPU", "Avg/frame", "Avg/call", "Parent%", "Calls");
			plog("{:-<154}", "");

			logRow("Recorded CPU", a_root.m_totalNs, a_root.m_totalNs, 0, a_frames);
		}

		void logFlatRows(const AggregateNode& a_root, std::uint64_t a_frames, const DumpOptions& a_options)
		{
			std::unordered_map<std::string, FlatRow> map;
			addFlatRows(map, a_root);

			std::vector<FlatRow> rows;
			rows.reserve(map.size());

			for (auto& [name, row] : map) {
				if (nsToMs(row.m_totalNs) >= a_options.m_minScopeMs) {
					rows.emplace_back(std::move(row));
				}
			}

			std::ranges::sort(rows, [](const auto& a_lhs, const auto& a_rhs) {
				return a_lhs.m_totalNs > a_rhs.m_totalNs;
			});

			plog("Physics profile flat hotspots");
			plog("{:<60} {:>12} {:>18} {:>18} {:>8}", "Scope", "Total CPU", "Avg/frame", "Avg/call", "Calls");
			plog("{:-<122}", "");

			for (std::size_t i = 0; i < std::min<std::size_t>(rows.size(), a_options.m_flatLimit); ++i) {
				const auto& row = rows[i];
				const auto totalMs = nsToMs(row.m_totalNs);
				const auto msPerFrame = a_frames > 0 ? totalMs / static_cast<double>(a_frames) : 0.0;
				const auto msPerCall = row.m_calls > 0 ? totalMs / static_cast<double>(row.m_calls) : 0.0;

				plog("{:<60} {:>10.3f} ms {:>10.3f} ms/frame {:>10.3f} ms/call {:>8}", row.m_name, totalMs, msPerFrame, msPerCall, row.m_calls);
			}
		}

		void logThreads(std::vector<ThreadRow>& a_rows, std::uint64_t a_frames)
		{
			std::ranges::sort(a_rows, [](const auto& a_lhs, const auto& a_rhs) {
				return a_lhs.m_totalNs > a_rhs.m_totalNs;
			});

			plog("Physics profile threads");
			plog("{:<12} {:>12} {:>18} {:>8}", "Thread", "Total CPU", "Avg/frame", "Calls");
			plog("{:-<62}", "");

			for (const auto& row : a_rows) {
				const auto totalMs = nsToMs(row.m_totalNs);
				const auto msPerFrame = a_frames > 0 ? totalMs / static_cast<double>(a_frames) : 0.0;

				plog("T{:<10} {:>10.3f} ms {:>10.3f} ms/frame {:>8}", row.m_index, totalMs, msPerFrame, row.m_calls);
			}
		}

		void resetUnlocked(bool a_resetTotalFrames)
		{
			for (auto& thread : g_threads) {
				thread->m_root.resetStats();
				thread->m_stack.clear();
			}

			g_windowFrames = 0;

			if (a_resetTotalFrames) {
				g_totalFrames = 0;
			}

			g_windowStart = Clock::now();
		}

		void install()
		{
			{
				std::scoped_lock lock(g_threadsLock);
				resetUnlocked(true);
			}

			btSetCustomEnterProfileZoneFunc(&enter);
			btSetCustomLeaveProfileZoneFunc(&leave);
		}

		void uninstall()
		{
			btSetCustomEnterProfileZoneFunc(&emptyEnter);
			btSetCustomLeaveProfileZoneFunc(&emptyLeave);
		}

		void enter(const char* a_name) noexcept
		{
			if (!a_name) {
				return;
			}

			g_activeScopes.fetch_add(1, std::memory_order_acq_rel);

			try {
				auto& thread = getThreadState();
				auto& parent = thread.m_stack.empty() ? thread.m_root : *thread.m_stack.back().m_node;
				auto& node = parent.child(a_name);

				++node.m_calls;
				thread.m_stack.emplace_back(&node, Clock::now());
			} catch (...) {
				g_activeScopes.fetch_sub(1, std::memory_order_acq_rel);
			}
		}

		void leave() noexcept
		{
			try {
				auto* thread = g_threadState;

				if (!thread || thread->m_stack.empty()) {
					return;
				}

				const auto now = Clock::now();
				const auto frame = thread->m_stack.back();

				thread->m_stack.pop_back();
				frame.m_node->m_totalNs += elapsedNs(frame.m_start, now);

				g_activeScopes.fetch_sub(1, std::memory_order_acq_rel);
			} catch (...) {
			}
		}

		bool dumpAndReset(const DumpOptions& a_options)
		{
			const auto activeScopes = g_activeScopes.load(std::memory_order_acquire);

			if (activeScopes != 0) {  // Just debug in case we fucked up somewhere
				logger::warn("Physics profile dump skipped because {} profile scopes are still active", activeScopes);
				hdt::menuConsoleAppendLines(
					fmt::format("Physics profile dump skipped because {} profile scopes are still active", activeScopes));
				return false;
			}

			AggregateNode root{ "Recorded CPU" };
			std::vector<ThreadRow> threads;
			std::uint64_t frames = 0;
			Clock::time_point windowStart;

			{
				std::scoped_lock lock(g_threadsLock);

				frames = std::max<std::uint64_t>(1, g_windowFrames);
				windowStart = g_windowStart;

				for (const auto& thread : g_threads) {
					const auto total = getChildTotal(thread->m_root);
					const auto calls = getChildCalls(thread->m_root);

					if (total == 0 && calls == 0) {
						continue;
					}

					threads.emplace_back(thread->m_index, total, calls);
					mergeRoot(root, thread->m_root);
				}

				root.m_totalNs = getChildTotal(root);
				resetUnlocked(false);
			}

			if (root.m_totalNs == 0) {
				plog("Physics profile had no recorded scopes");
				return true;
			}

			if (a_options.m_logTree) {
				logTreeHeader(root, frames, windowStart);
				logTreeRecursive(root, "", 0, frames, a_options);
			}

			if (a_options.m_logFlat) {
				logFlatRows(root, frames, a_options);
			}

			if (a_options.m_logThreads) {
				logThreads(threads, frames);
			}

			return true;
		}

		bool dumpEvery(std::uint64_t a_printIntervalFrames, const DumpOptions& a_options)
		{
			if (a_printIntervalFrames == 0) {
				return false;
			}

			if (g_totalFrames != 0 && g_totalFrames % a_printIntervalFrames == 0) {
				return dumpAndReset(a_options);
			}

			if (g_sampleWindowFrames != 0 && g_windowFrames >= g_sampleWindowFrames) {
				if (g_activeScopes.load(std::memory_order_acquire) == 0) {
					std::scoped_lock lock(g_threadsLock);
					resetUnlocked(false);
				}
			}

			return false;
		}
	}

	void setCapture(bool a_enabled, std::uint64_t a_sampleFrames, std::uint64_t a_printFrames)
	{
		if (a_enabled) {
			const auto sampleFrames = std::max<std::uint64_t>(1, a_sampleFrames);
			const auto printFrames = std::max<std::uint64_t>(1, a_printFrames);

			btSetProfileEnabled(false);
			g_sampleWindowFrames = sampleFrames;
			g_printIntervalFrames = printFrames;
			install();
			g_captureEnabled = true;
			btSetProfileEnabled(true);
		} else {
			btSetProfileEnabled(false);
			g_captureEnabled = false;
			g_printIntervalFrames = 0;
			g_sampleWindowFrames = 0;
			uninstall();

			std::scoped_lock lock(g_threadsLock);
			resetUnlocked(true);
		}
	}

	void advanceFrame()
	{
		if (!g_captureEnabled) {
			return;
		}

		++g_windowFrames;
		++g_totalFrames;
		dumpEvery(g_printIntervalFrames, {});
	}
}
