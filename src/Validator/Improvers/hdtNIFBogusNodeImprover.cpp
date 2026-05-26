#include "hdtNIFBogusNodeImprover.h"

#include "hdtNIFBinaryIO.h"

#include "../Schema/hdtNifSchema.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
	// ── Bogus-node profiling ──────────────────────────────────────────────────
	static std::atomic<int64_t> g_bnProfCacheBuild {0};
	static std::atomic<int64_t> g_bnProfDetection  {0};
	static std::atomic<int64_t> g_bnProfRemap       {0};
	static std::atomic<int64_t> g_bnProfCacheUpdate {0};
	static std::atomic<int64_t> g_bnTotalPasses     {0};
	static std::atomic<int64_t> g_bnTotalRemovals   {0};
	static std::atomic<int64_t> g_bnNifCount        {0};

	static auto bnNow() { return std::chrono::high_resolution_clock::now(); }
	static int64_t bnNs(std::chrono::high_resolution_clock::time_point a,
	                    std::chrono::high_resolution_clock::time_point b)
	{
		return std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count();
	}

	void resetBogusNodeTimings()
	{
		g_bnProfCacheBuild.store(0); g_bnProfDetection.store(0);
		g_bnProfRemap     .store(0); g_bnProfCacheUpdate.store(0);
		g_bnTotalPasses   .store(0); g_bnTotalRemovals .store(0);
		g_bnNifCount      .store(0);
	}

	void logBogusNodeTimings()
	{
		const int64_t n = g_bnNifCount.load();
		logger::info("[BogusNodes][PROF] logBogusNodeTimings called, n={}", n);
		if (n == 0) return;
		auto ms = [](int64_t ns) { return ns / 1'000'000; };
		const int64_t passes   = g_bnTotalPasses.load();
		const int64_t removals = g_bnTotalRemovals.load();
		logger::info("[BogusNodes][PROF] ── bogusNodes sub-breakdown ({} NIFs, {} passes, {} removals) ──",
			n, passes, removals);
		logger::info("[BogusNodes][PROF]   cache-build   {:>8} ms  ({} µs/nif)", ms(g_bnProfCacheBuild),  g_bnProfCacheBuild  / n / 1000);
		logger::info("[BogusNodes][PROF]   detection     {:>8} ms  ({} µs/nif)", ms(g_bnProfDetection),   g_bnProfDetection   / n / 1000);
		logger::info("[BogusNodes][PROF]   remap         {:>8} ms  ({} µs/nif)", ms(g_bnProfRemap),       g_bnProfRemap       / n / 1000);
		logger::info("[BogusNodes][PROF]   cache-update  {:>8} ms  ({} µs/nif)", ms(g_bnProfCacheUpdate), g_bnProfCacheUpdate / n / 1000);
		logger::info("[BogusNodes][PROF]   passes/nif    {:.1f}  removals/nif {:.1f}",
			static_cast<double>(passes) / n, static_cast<double>(removals) / n);
	}

	// The "remove bogus nodes" algorithm is a clean-room port of the
	// spRemoveBogusNodes spell from NifSkope (fo76utils/nifskope),
	// copyright (c) 2005-2014 NIF File Format Library and Tools,
	// under the NifSkope 3-clause BSD license.

	static bool isImportantNodeName(const std::string& name, uint32_t bsVersion)
	{
		if (name == "ProjectileNode")
			return true;
		if (bsVersion < 83 && (name == "ShellCasingNode" || name == "##SightingNode"))
			return true;
		if (bsVersion >= 130 && name == "WorkshopConnectPoints")
			return true;
		return false;
	}

	static bool isImportantSubtype(const std::string& typeName)
	{
		return typeName == "BSBlastNode"
		    || typeName == "BSDamageStage"
		    || typeName == "BSDebrisNode"
		    || typeName == "BSValueNode";
	}

	// ── Logging helpers (must precede link helpers that use them) ────────────
	static std::string setToStr(const std::unordered_set<int32_t>& s)
	{
		if (s.empty()) return "{}";
		std::string r = "{";
		for (int32_t v : s) r += std::to_string(v) + ",";
		r.back() = '}';
		return r;
	}

	// ── Link helpers ─────────────────────────────────────────────────────────
	static std::unordered_set<int32_t> getChildLinks(
		const NifSchema&             schema,
		const std::string&           typeName,
		const std::vector<uint8_t>&  block,
		int32_t                      numBlocks,
		bool                         walkerLog = false)
	{
		auto opt = walkBlockRefs(schema, typeName, block.data(), block.size(),
		                         numBlocks, LinkFilter::RefOnly, walkerLog);
		std::unordered_set<int32_t> result;
		if (!opt.has_value())
			return result;  // unknown type → no data, not incorrect data
		for (size_t off : *opt) {
			if (off + 4 > block.size()) continue;
			int32_t v = -1;
			std::memcpy(&v, block.data() + off, 4);
			if (v >= 0 && v < numBlocks)
				result.insert(v);
		}
		return result;
	}

	static std::unordered_set<int32_t> getParentLinks(
		const NifSchema&             schema,
		const std::string&           typeName,
		const std::vector<uint8_t>&  block,
		int32_t                      numBlocks,
		int32_t                      blockIdx  = -1,   // -1 = not logged
		bool                         walkerLog = false)
	{
		auto opt = walkBlockRefs(schema, typeName, block.data(), block.size(),
		                         numBlocks, LinkFilter::PtrOnly, walkerLog);

		const bool doLog = false;//(blockIdx >= 0);
		if (doLog)
			logger::info("[BogusNodes]       getParentLinks(block={} type='{}'): walker={}",
			             blockIdx, typeName, opt.has_value() ? "OK" : "nullopt(unknown type)");

		std::unordered_set<int32_t> result;
		if (!opt.has_value()) {
			if (doLog)
				logger::info("[BogusNodes]         → unknown type, result={{}}");
			return result;
		}

		if (doLog)
			logger::info("[BogusNodes]         Ptr-field offsets found: {} offset(s)", opt->size());

		for (size_t off : *opt) {
			if (off + 4 > block.size()) {
				if (doLog)
					logger::info("[BogusNodes]         off={} → out of bounds (blockSize={}), skip",
					             off, block.size());
				continue;
			}
			int32_t v = -1;
			std::memcpy(&v, block.data() + off, 4);
			bool inRange = (v >= 0 && v < numBlocks);
			if (doLog)
				logger::info("[BogusNodes]         off={} value={} inRange={}", off, v, inRange);
			if (inRange)
				result.insert(v);
		}

		if (doLog)
			logger::info("[BogusNodes]         → result={}", setToStr(result));
		return result;
	}

	// ── Batch remap helper ───────────────────────────────────────────────────
	// Removes all blocks in `toRemoveSorted` (sorted ascending) from `parsed`
	// in a single walker pass over the remaining blocks, rather than one pass
	// per removed block.  Correctness: bogus nodes have empty child/parent link
	// sets, so they cannot reference each other — all removals in one batch are
	// mutually independent.
	static void batchRemapAndRemove(ParsedNif& parsed,
	                                const std::vector<int32_t>& toRemoveSorted,
	                                const NifSchema& schema)
	{
		if (toRemoveSorted.empty()) return;

		const int32_t totalBefore = static_cast<int32_t>(parsed.blocks.size());
		const std::unordered_set<int32_t> removeSet(toRemoveSorted.begin(), toRemoveSorted.end());

		// Given an old ref value, return the adjusted value after batch removal.
		auto adjustRef = [&](int32_t v) -> int32_t {
			if (v < 0 || v >= totalBefore) return v;
			if (removeSet.count(v)) return -1;
			auto it = std::lower_bound(toRemoveSorted.begin(), toRemoveSorted.end(), v);
			return v - static_cast<int32_t>(it - toRemoveSorted.begin());
		};

		for (int32_t i = 0; i < totalBefore; ++i) {
			if (removeSet.count(i)) continue;

			uint16_t tIdx = parsed.blockTypeIndex[static_cast<size_t>(i)];
			const std::string& typeName =
				(tIdx < parsed.blockTypes.size()) ? parsed.blockTypes[tIdx] : std::string{};
			auto& block = parsed.blocks[static_cast<size_t>(i)];

			auto detailOpt = walkBlockRefDetails(schema, typeName,
			                                     block.data(), block.size(),
			                                     totalBefore + 1);
			if (!detailOpt.has_value()) continue;

			std::map<std::string, std::vector<size_t>> toDelete;

			for (const auto& rd : detailOpt->refs) {
				if (rd.offset + 4 > block.size()) continue;
				int32_t v = -1;
				std::memcpy(&v, block.data() + rd.offset, 4);
				int32_t newV = adjustRef(v);
				if (newV == v) continue;

				if (removeSet.count(v) && !rd.arr1.empty()) {
					toDelete[rd.arr1].push_back(rd.offset);
				} else {
					std::memcpy(block.data() + rd.offset, &newV, 4);
				}
			}

			for (auto& [arr1Name, offsets] : toDelete) {
				auto cpit = detailOpt->countFieldPos.find(arr1Name);
				if (cpit == detailOpt->countFieldPos.end()) continue;
				const size_t countPos = cpit->second;
				if (countPos + 4 > block.size()) continue;

				std::sort(offsets.begin(), offsets.end(), std::greater<size_t>());
				for (size_t off : offsets) {
					if (off + 4 > block.size()) continue;
					block.erase(block.begin() + static_cast<ptrdiff_t>(off),
					            block.begin() + static_cast<ptrdiff_t>(off) + 4);
				}

				uint32_t count = 0;
				std::memcpy(&count, block.data() + countPos, 4);
				const uint32_t n = static_cast<uint32_t>(offsets.size());
				if (n <= count) count -= n;
				std::memcpy(block.data() + countPos, &count, 4);
			}
		}

		// Erase removed blocks in reverse order to preserve valid indices.
		for (auto it = toRemoveSorted.rbegin(); it != toRemoveSorted.rend(); ++it) {
			parsed.blocks.erase(parsed.blocks.begin() + *it);
			parsed.blockTypeIndex.erase(parsed.blockTypeIndex.begin() + *it);
		}
	}

	void removeBlocksAndRemap(ParsedNif& parsed, const std::vector<int32_t>& toRemoveSorted)
	{
		batchRemapAndRemove(parsed, toRemoveSorted, globalNifSchema());
	}

	// ── Main algorithm ────────────────────────────────────────────────────────
	bool removeBogusNiNodes(ParsedNif& parsed,
	                        const std::unordered_set<std::string>* xmlProtectedNodeNames)
	{
		const NifSchema& schema = globalNifSchema();
		bool anyChanged = false;

		int32_t numBlocks = static_cast<int32_t>(parsed.blocks.size());

		auto typeOf = [&](int32_t i) -> const std::string& {
			uint16_t tIdx = parsed.blockTypeIndex[static_cast<size_t>(i)];
			static const std::string kEmpty{};
			return (tIdx < parsed.blockTypes.size()) ? parsed.blockTypes[tIdx] : kEmpty;
		};

		// ── Build initial link-set cache ──────────────────────────────────────
		auto _tc0 = bnNow();
		std::vector<std::unordered_set<int32_t>> childLinks(numBlocks);
		std::vector<std::unordered_set<int32_t>> parentLinks(numBlocks);
		for (int32_t i = 0; i < numBlocks; ++i) {
			childLinks[i]  = getChildLinks (schema, typeOf(i), parsed.blocks[i], numBlocks);
			parentLinks[i] = getParentLinks(schema, typeOf(i), parsed.blocks[i], numBlocks);
		}
		g_bnProfCacheBuild.fetch_add(bnNs(_tc0, bnNow()));
		g_bnNifCount.fetch_add(1);

		bool anyRemovedInBatch;
		do {
			anyRemovedInBatch = false;

			// ── Detection: collect ALL bogus nodes in one scan ────────────────
			// Bogus nodes have empty child/parent sets so they cannot reference
			// each other → all found in one scan are independent and safe to
			// batch-remove in a single remap pass.
			auto _td0 = bnNow();
			std::vector<int32_t> toRemove;
			for (int32_t b = 0; b < numBlocks; ++b) {
				if (typeOf(b) != "NiNode") continue;
				if (!childLinks[b].empty() || !parentLinks[b].empty()) continue;

				std::string nodeName;
				const auto& block = parsed.blocks[static_cast<size_t>(b)];
				if (block.size() >= 4) {
					int32_t nameIdx = -1;
					std::memcpy(&nameIdx, block.data(), 4);
					if (nameIdx >= 0 && nameIdx < static_cast<int32_t>(parsed.strings.size()))
						nodeName = parsed.strings[static_cast<size_t>(nameIdx)];
				}

				if (xmlProtectedNodeNames && xmlProtectedNodeNames->count(nodeName))
					continue;

				const std::string& typeName = typeOf(b);
				bool impSubtype = isImportantSubtype(typeName);
				int x = impSubtype ? 1 : 0;
				if (!x && parsed.bsVersion >= 14 && isImportantNodeName(nodeName, parsed.bsVersion))
					x = 1;

				for (int32_t c = 0; c < numBlocks && x < 2; ++c) {
					if (c == b) continue;
					if (childLinks[c].count(b) > 0) ++x;
					if (x < 2 && parentLinks[c].count(b) > 0) x = 2;
				}

				if (x < 2)
					toRemove.push_back(b);
			}
			g_bnProfDetection.fetch_add(bnNs(_td0, bnNow()));

			if (toRemove.empty()) break;

			anyRemovedInBatch = true;
			anyChanged = true;
			g_bnTotalPasses.fetch_add(1);
			g_bnTotalRemovals.fetch_add(static_cast<int64_t>(toRemove.size()));

			// ── Remap: single batch pass for all removals ─────────────────────
			auto _tr0 = bnNow();
			batchRemapAndRemove(parsed, toRemove, schema);
			g_bnProfRemap.fetch_add(bnNs(_tr0, bnNow()));

			// ── Cache update ──────────────────────────────────────────────────
			auto _tu0 = bnNow();
			{
				const std::unordered_set<int32_t> removeSet(toRemove.begin(), toRemove.end());

				// Erase removed entries in reverse index order.
				for (auto it = toRemove.rbegin(); it != toRemove.rend(); ++it) {
					childLinks.erase(childLinks.begin() + *it);
					parentLinks.erase(parentLinks.begin() + *it);
				}
				numBlocks -= static_cast<int32_t>(toRemove.size());

				// Remap remaining set values: remove deleted indices, decrement > deleted.
				for (int32_t i = 0; i < numBlocks; ++i) {
					auto remap = [&](std::unordered_set<int32_t>& s) {
						std::unordered_set<int32_t> updated;
						updated.reserve(s.size());
						for (int32_t v : s) {
							if (removeSet.count(v)) continue;
							auto it = std::lower_bound(toRemove.begin(), toRemove.end(), v);
							updated.insert(v - static_cast<int32_t>(it - toRemove.begin()));
						}
						s = std::move(updated);
					};
					remap(childLinks[i]);
					remap(parentLinks[i]);
				}
			}
			g_bnProfCacheUpdate.fetch_add(bnNs(_tu0, bnNow()));

		} while (anyRemovedInBatch);

		return anyChanged;
	}
}
