#include "hdtNIFImprover.h"

#include "hdtNIFBinaryIO.h"
#include "hdtNIFBogusNodeImprover.h"
#include "hdtNIFOrphanedSkinImprover.h"
#include "hdtNIFSkinMeshRepair.h"
#include "hdtNIFDecimator.h"

#include "../Utils/hdtNIFBinaryUtils.h"
#include "../Utils/hdtStringUtils.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace hdt
{
	namespace
	{
		constexpr size_t kUnsetOffset = static_cast<size_t>(-1);
	}

	// ── Phase-3 profiling ─────────────────────────────────────────────────────
	// Accumulated CPU-nanoseconds per operation, across all threads.
	// Call logNIFImproverTimings() after the parallel loop to dump a breakdown.
	static std::atomic<int64_t> g_profFileRead      {0};
	static std::atomic<int64_t> g_profParse         {0};
	static std::atomic<int64_t> g_profBogusNodes    {0};
	static std::atomic<int64_t> g_profOrphanedSkin  {0};
	static std::atomic<int64_t> g_profSkinMeshRepair{0};
	static std::atomic<int64_t> g_profXmlStrip      {0};
	static std::atomic<int64_t> g_profSerialize     {0};
	static std::atomic<int64_t> g_profValidate      {0};
	static std::atomic<int64_t> g_profWrite         {0};
	static std::atomic<int64_t> g_profNifCount      {0};

	static auto now() { return std::chrono::high_resolution_clock::now(); }
	static int64_t elapsedNs(std::chrono::high_resolution_clock::time_point a,
	                         std::chrono::high_resolution_clock::time_point b)
	{
		return std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count();
	}

	void resetNIFImproverTimings()
	{
		g_profFileRead    .store(0); g_profParse          .store(0);
		g_profBogusNodes  .store(0); g_profOrphanedSkin   .store(0);
		g_profSkinMeshRepair.store(0); g_profXmlStrip     .store(0);
		g_profSerialize   .store(0); g_profValidate       .store(0);
		g_profWrite       .store(0); g_profNifCount       .store(0);
	}

	void logNIFImproverTimings()
	{
		const int64_t n = g_profNifCount.load();
		logger::info("[NIFImprover][PROF] logNIFImproverTimings called, n={}", n);
		if (n == 0) return;
		auto ms = [](int64_t ns) { return ns / 1'000'000; };
		logger::info("[NIFImprover][PROF] ── Phase-3 CPU-time breakdown ({} NIFs) ──", n);
		logger::info("[NIFImprover][PROF]   file-read     {:>8} ms  ({} µs/nif)", ms(g_profFileRead),     g_profFileRead     / n / 1000);
		logger::info("[NIFImprover][PROF]   parseNif      {:>8} ms  ({} µs/nif)", ms(g_profParse),        g_profParse        / n / 1000);
		logger::info("[NIFImprover][PROF]   bogusNodes      {:>8} ms  ({} µs/nif)", ms(g_profBogusNodes),    g_profBogusNodes    / n / 1000);
		logger::info("[NIFImprover][PROF]   orphanedSkin    {:>8} ms  ({} µs/nif)", ms(g_profOrphanedSkin),  g_profOrphanedSkin  / n / 1000);
		logger::info("[NIFImprover][PROF]   skinMeshRepair  {:>8} ms  ({} µs/nif)", ms(g_profSkinMeshRepair),g_profSkinMeshRepair/ n / 1000);
		logger::info("[NIFImprover][PROF]   xmlStrip        {:>8} ms  ({} µs/nif)", ms(g_profXmlStrip),      g_profXmlStrip      / n / 1000);
		logger::info("[NIFImprover][PROF]   serialize       {:>8} ms  ({} µs/nif)", ms(g_profSerialize),     g_profSerialize     / n / 1000);
		logger::info("[NIFImprover][PROF]   validate        {:>8} ms  ({} µs/nif)", ms(g_profValidate),      g_profValidate      / n / 1000);
		logger::info("[NIFImprover][PROF]   write           {:>8} ms  ({} µs/nif)", ms(g_profWrite),         g_profWrite         / n / 1000);
		const int64_t total = g_profFileRead + g_profParse + g_profBogusNodes + g_profOrphanedSkin +
		                      g_profSkinMeshRepair + g_profXmlStrip + g_profSerialize + g_profValidate + g_profWrite;
		logger::info("[NIFImprover][PROF]   TOTAL       {:>8} ms  (accounted CPU time across all threads)", ms(total));
		logBogusNodeTimings();
	}

	// ── Physics XML ref stripping ─────────────────────────────────────────────

	struct TypedStripResult
	{
		std::vector<int32_t> blocksToRemove;
		int markerMatchedBlocks    = 0;
		int alternateLayoutMatches = 0;
		int valueMatchedFallbacks  = 0;
	};

	// Main pass: scan blocks whose type index matches NiStringExtraData.
	// For each block, locate the physics marker name lane and the XML value lane,
	// then record the block index for removal.
	static TypedStripResult stripXmlRefsFromTypedBlocks(
		const ParsedNif& parsed,
		const std::unordered_set<std::string>& missingRefs,
		int markerIdx,
		int typeIdx)
	{
		TypedStripResult result;

		for (size_t i = 0; i < parsed.blocks.size(); ++i) {
			if (i >= parsed.blockTypeIndex.size())
				break;
			if (parsed.blockTypeIndex[i] != static_cast<uint16_t>(typeIdx))
				continue;

			const auto& block = parsed.blocks[i];
			if (block.size() < nif::kNiStringExtraDataMinBlockSize)
				continue;

			size_t nameOffset          = kUnsetOffset;
			size_t firstXmlValueOffset  = kUnsetOffset;
			size_t firstAnyValueOffset  = kUnsetOffset;
			size_t firstMissingOffset   = kUnsetOffset;

			const size_t laneCount = block.size() / sizeof(uint32_t);
			for (size_t lane = 0; lane < laneCount; ++lane) {
				const size_t off = lane * sizeof(uint32_t);
				uint32_t v = 0;
				std::memcpy(&v, block.data() + off, sizeof(uint32_t));

				if (v == static_cast<uint32_t>(markerIdx) && nameOffset == kUnsetOffset)
					nameOffset = off;

				if (v >= parsed.strings.size())
					continue;

				const std::string norm = NormalizePathForComparison(parsed.strings[v]);
				const bool xmlLike = norm.find(".xml") != std::string::npos;
				const bool missing = missingRefs.count(norm) > 0;

				if (firstAnyValueOffset == kUnsetOffset) firstAnyValueOffset = off;
				if (xmlLike && firstXmlValueOffset == kUnsetOffset) firstXmlValueOffset = off;
				if (missing && firstMissingOffset  == kUnsetOffset) firstMissingOffset  = off;
			}

			const bool markerMatched = nameOffset != kUnsetOffset;
			const bool missingByPath = firstMissingOffset != kUnsetOffset;

			if (!markerMatched && !missingByPath)
				continue;

			// Prefer the XML value lane immediately after the name when marker is present.
			size_t chosenValueOffset = kUnsetOffset;
			if (markerMatched) {
				for (size_t lane = (nameOffset / sizeof(uint32_t)) + 1; lane < laneCount; ++lane) {
					const size_t off = lane * sizeof(uint32_t);
					uint32_t v = 0;
					std::memcpy(&v, block.data() + off, sizeof(uint32_t));
					if (v >= parsed.strings.size())
						continue;
					if (NormalizePathForComparison(parsed.strings[v]).find(".xml") != std::string::npos) {
						chosenValueOffset = off;
						break;
					}
				}
			}
			if (chosenValueOffset == kUnsetOffset)
				chosenValueOffset = firstXmlValueOffset != kUnsetOffset ? firstXmlValueOffset : firstAnyValueOffset;
			if (chosenValueOffset == kUnsetOffset)
				continue;

			uint32_t chosenValueIdx = 0;
			std::memcpy(&chosenValueIdx, block.data() + chosenValueOffset, sizeof(uint32_t));
			if (chosenValueIdx >= parsed.strings.size())
				continue;

			logger::trace(
				"[NIFImprover] Block {}: name@{} value@{} markerMatched={} raw='{}', normalized='{}'",
				i,
				markerMatched ? std::to_string(nameOffset) : std::string("none"),
				chosenValueOffset,
				markerMatched,
				parsed.strings[chosenValueIdx],
				NormalizePathForComparison(parsed.strings[chosenValueIdx]));

			if (!missingByPath) {
				logger::trace("[NIFImprover] Block {}: XML value not in missing set, skipping", i);
				continue;
			}

			if (markerMatched) {
				++result.markerMatchedBlocks;
				if (nameOffset == 4)
					++result.alternateLayoutMatches;
			} else {
				++result.valueMatchedFallbacks;
			}

			logger::trace("[NIFImprover] Block {}: queuing NiStringExtraData for removal", i);
			result.blocksToRemove.push_back(static_cast<int32_t>(i));
		}

		return result;
	}

	// Last-resort pass when no NiStringExtraData blocks were found via the type index.
	// Scans every block for any (marker, missing-xml-value) pair and queues it for removal.
	static std::vector<int32_t> stripXmlRefsRawFallback(
		const ParsedNif& parsed,
		const std::unordered_set<std::string>& missingRefs,
		int markerIdx)
	{
		std::vector<int32_t> result;

		for (size_t bi = 0; bi < parsed.blocks.size(); ++bi) {
			const auto& block = parsed.blocks[bi];
			if (block.size() < sizeof(uint32_t) * 2)
				continue;

			const size_t laneCount = block.size() / sizeof(uint32_t);
			size_t markerOffset       = kUnsetOffset;
			size_t missingValueOffset = kUnsetOffset;
			uint32_t missingValueIdx  = 0;

			for (size_t lane = 0; lane < laneCount; ++lane) {
				const size_t off = lane * sizeof(uint32_t);
				uint32_t v = 0;
				std::memcpy(&v, block.data() + off, sizeof(uint32_t));

				if (v == static_cast<uint32_t>(markerIdx) && markerOffset == kUnsetOffset)
					markerOffset = off;

				if (v >= parsed.strings.size())
					continue;

				const std::string norm = NormalizePathForComparison(parsed.strings[v]);
				if (missingRefs.count(norm) > 0 && missingValueOffset == kUnsetOffset) {
					missingValueOffset = off;
					missingValueIdx    = v;
				}
			}

			if (markerOffset == kUnsetOffset || missingValueOffset == kUnsetOffset)
				continue;

			logger::warn(
				"[NIFImprover] Raw-pair fallback: queuing block {} for removal (marker@{}, missingValueIdx={}@{})",
				bi, markerOffset, missingValueIdx, missingValueOffset);

			result.push_back(static_cast<int32_t>(bi));
		}

		if (!result.empty())
			logger::warn("[NIFImprover] Raw-pair fallback queued {} block(s) for removal", result.size());

		return result;
	}

	static bool removeMissingPhysicsXmlExtraData(
		ParsedNif& parsed,
		const std::unordered_set<std::string>* missingPhysicsXmlRefs)
	{
		if (!missingPhysicsXmlRefs || missingPhysicsXmlRefs->empty())
			return false;

		logger::trace("[NIFImprover] missingPhysicsXmlRefs ({} entries):", missingPhysicsXmlRefs->size());
		for (const auto& ref : *missingPhysicsXmlRefs)
			logger::trace("  - {}", ref);

		int markerIdx = -1;
		for (int i = 0; i < static_cast<int>(parsed.strings.size()); ++i) {
			if (parsed.strings[static_cast<size_t>(i)] == nif::kPhysicsMarker) {
				markerIdx = i;
				break;
			}
		}
		if (markerIdx < 0)
			return false;

		int typeIdx = -1;
		for (int i = 0; i < static_cast<int>(parsed.blockTypes.size()); ++i) {
			if (parsed.blockTypes[static_cast<size_t>(i)] == nif::kTypeNiStringExtraData) {
				typeIdx = i;
				break;
			}
		}
		if (typeIdx < 0)
			return false;

		auto r = stripXmlRefsFromTypedBlocks(parsed, *missingPhysicsXmlRefs, markerIdx, typeIdx);

		if (r.markerMatchedBlocks == 0) {
			logger::warn(
				"[NIFImprover] No NiStringExtraData blocks referenced the physics marker "
				"(markerIdx={}, typeIdx={}, blockTypes={})",
				markerIdx, typeIdx, parsed.blockTypes.size());
			if (r.valueMatchedFallbacks > 0)
				logger::warn("[NIFImprover] Value-path fallback queued {} block(s) for removal", r.valueMatchedFallbacks);

			auto fallback = stripXmlRefsRawFallback(parsed, *missingPhysicsXmlRefs, markerIdx);
			for (int32_t idx : fallback)
				r.blocksToRemove.push_back(idx);
		}

		if (r.alternateLayoutMatches > 0)
			logger::warn("[NIFImprover] Used alternate NiStringExtraData layout for {} block(s)", r.alternateLayoutMatches);

		if (r.blocksToRemove.empty())
			return false;

		std::sort(r.blocksToRemove.begin(), r.blocksToRemove.end());
		r.blocksToRemove.erase(
			std::unique(r.blocksToRemove.begin(), r.blocksToRemove.end()),
			r.blocksToRemove.end());
		removeBlocksAndRemap(parsed, r.blocksToRemove);
		return true;
	}

	// ── Public API ────────────────────────────────────────────────────────────

	bool GenerateImprovedNIF(
		const std::string& srcNIFPath,
		const std::string& outputDir,
		const NIFDecimationOptions& options,
		NIFImproverDiagnostics& outDiagnostics,
		const std::unordered_set<std::string>* missingPhysicsXmlRefs,
		bool copyOriginal)
	{
		namespace fs = std::filesystem;

		outDiagnostics = {};

		g_profNifCount.fetch_add(1);
		auto _t0 = now();
		std::ifstream in(std::filesystem::u8path(srcNIFPath), std::ios::binary | std::ios::ate);
		if (!in.is_open()) {
			logger::error("[NIFImprover] Failed to open NIF file '{}'", srcNIFPath);
			return false;
		}
		auto sz = in.tellg();
		if (sz <= 0 || sz > static_cast<std::streamoff>(nif::kMaxNifFileSize)) {
			logger::error("[NIFImprover] Invalid file size={} for '{}'", static_cast<long long>(sz), srcNIFPath);
			return false;
		}
		std::vector<uint8_t> data(static_cast<size_t>(sz));
		in.seekg(0);
		in.read(reinterpret_cast<char*>(data.data()), sz);
		if (in.gcount() != static_cast<std::streamsize>(sz)) {
			logger::error("[NIFImprover] Short read for '{}', expected={} actual={}",
				srcNIFPath, static_cast<long long>(sz), static_cast<long long>(in.gcount()));
			return false;
		}
		auto _t1 = now();
		g_profFileRead.fetch_add(elapsedNs(_t0, _t1));

		std::string parseError;
		auto parsedOpt = parseNif(data, &parseError);
		if (!parsedOpt.has_value()) {
			logger::error("[NIFImprover] parseNif failed for '{}': {}",
				srcNIFPath, parseError.empty() ? "unknown parse error" : parseError);
			return false;
		}
		auto _t2 = now();
		g_profParse.fetch_add(elapsedNs(_t1, _t2));
		auto& parsed = *parsedOpt;

		logger::trace("[NIFImprover] Parsed '{}': mode={} endian={} blocks={} strings={}",
			srcNIFPath,
			parsed.hasExplicitEndiannessByte ? "explicit-endian" : "legacy",
			parsed.endianness == 0 ? "big" : "little",
			parsed.blocks.size(),
			parsed.strings.size());

		if (!parsed.groups.empty()) {
			logger::error("[NIFImprover] Unsupported grouped NIF '{}' (group count={})",
				srcNIFPath, parsed.groups.size());
			return false;
		}

		bool changed = false;
		auto _t3 = now();
		if (options.enableStructuralFixes)
			changed |= removeBogusNiNodes(parsed);
		auto _t4 = now();
		g_profBogusNodes.fetch_add(elapsedNs(_t3, _t4));

		{
			int removed = 0;
			if (options.enableStructuralFixes) {
				removed = removeOrphanedSkinInstances(parsed);
				if (removed > 0) {
					changed = true;
					logger::info("[NIFImprover] Orphaned skin instances '{}': removed={}", srcNIFPath, removed);
				}
			}
			outDiagnostics.orphanedSkinInstancesRemoved = removed;
		}
		auto _t4b = now();
		g_profOrphanedSkin.fetch_add(elapsedNs(_t4, _t4b));

		{
			int repaired = 0;
			if (options.enableStructuralFixes) {
				repaired = repairNIFSkinMeshIssues(parsed);
				if (repaired > 0) {
					changed = true;
					logger::info("[NIFImprover] Skin mesh issues '{}': repaired={}", srcNIFPath, repaired);
				}
			}
			outDiagnostics.skinMeshIssuesFixed = repaired;
		}
		auto _t4c = now();
		g_profSkinMeshRepair.fetch_add(elapsedNs(_t4b, _t4c));

		if (options.enableStructuralFixes)
			changed |= removeMissingPhysicsXmlExtraData(parsed, missingPhysicsXmlRefs);
		auto _t5 = now();
		g_profXmlStrip.fetch_add(elapsedNs(_t4c, _t5));

		if (options.enableCollisionMeshDecimation) {
			auto d = runOfflineCollisionDecimationBridge(parsed, options, changed);
			outDiagnostics.decimationCandidatesDiscovered    = d.candidatesDiscovered;
			outDiagnostics.decimationCandidatesAttempted     = d.candidatesAttempted;
			outDiagnostics.decimationCandidatesApplied       = d.candidatesApplied;
			outDiagnostics.decimationCandidatesSkippedNoChange = d.candidatesSkippedNoChange;
			outDiagnostics.decimationCandidatesSkippedUnsafe = d.candidatesSkippedUnsafe;
			outDiagnostics.decimationSkipReasons             = d.skipReasons;
			if (d.candidatesAttempted > 0)
				logger::info("[NIFImprover] Decimation '{}': discovered={} attempted={} applied={} skipped-no-change={} skipped-unsafe={}",
					srcNIFPath, d.candidatesDiscovered, d.candidatesAttempted,
					d.candidatesApplied, d.candidatesSkippedNoChange, d.candidatesSkippedUnsafe);
		}

		if (!changed)
			return false;

		// Serialize once; reuse the buffer for both round-trip validation and the
		// file write to avoid serializing the same NIF twice.
		auto _ts0 = now();
		auto serialized = serializeNif(parsed);
		auto _ts1 = now();
		g_profSerialize.fetch_add(elapsedNs(_ts0, _ts1));
		auto roundTripError = validateNifRoundTripFromBytes(serialized, parsed);
		auto _ts2 = now();
		g_profValidate.fetch_add(elapsedNs(_ts1, _ts2));
		if (roundTripError.has_value()) {
			logger::error("[NIFImprover] Round-trip validation failed for '{}': {}", srcNIFPath, *roundTripError);
			outDiagnostics.validationError = *roundTripError;
			return false;
		}

		fs::path outPath = fs::path(outputDir) / stripDataPrefix(srcNIFPath);
		std::error_code ec;
		fs::create_directories(outPath.parent_path(), ec);
		if (ec) {
			logger::error("[NIFImprover] create_directories failed: {}", ec.message());
			return false;
		}

		auto _tw0 = now();
		if (!writeNifBytes(serialized, PathToUtf8(outPath))) {
			logger::error("[NIFImprover] Write failed for '{}'", PathToUtf8(outPath));
			return false;
		}
		g_profWrite.fetch_add(elapsedNs(_tw0, now()));
		logger::info("[NIFImprover] Wrote improved NIF: {}", PathToUtf8(outPath));

		if (copyOriginal) {
			fs::path originalOutPath = outPath;
			const std::string ext = originalOutPath.extension().string();
			originalOutPath.replace_filename(originalOutPath.stem().string() + "-original" + ext);
			fs::copy_file(fs::u8path(srcNIFPath), originalOutPath, fs::copy_options::overwrite_existing, ec);
			// Non-fatal: the improved NIF was already written; the original copy is for reference only.
		}

		return true;
	}

	bool CopyNIFToOutput(const std::string& srcNIFPath, const std::string& outputDir)
	{
		namespace fs = std::filesystem;
		fs::path outPath = fs::path(outputDir) / stripDataPrefix(srcNIFPath);
		std::error_code ec;
		fs::create_directories(outPath.parent_path(), ec);
		if (ec)
			return false;
		fs::copy_file(fs::u8path(srcNIFPath), outPath, fs::copy_options::overwrite_existing, ec);
		return !ec;
	}
}
