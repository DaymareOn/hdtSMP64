#include "hdtNIFBogusNodeImprover.h"

#include "hdtNIFBinaryIO.h"

#include "../Schema/hdtNifSchema.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
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

	// ── Remap helper ─────────────────────────────────────────────────────────
	static void remapAfterRemoval(ParsedNif& parsed, int32_t removed,
	                              const NifSchema& schema)
	{
		const int32_t numBlocks = static_cast<int32_t>(parsed.blocks.size());
		for (int32_t i = 0; i < numBlocks; ++i) {
			uint16_t tIdx = parsed.blockTypeIndex[static_cast<size_t>(i)];
			const std::string& typeName =
				(tIdx < parsed.blockTypes.size()) ? parsed.blockTypes[tIdx] : std::string{};

			auto& block = parsed.blocks[static_cast<size_t>(i)];

			auto detailOpt = walkBlockRefDetails(schema, typeName,
			                                     block.data(), block.size(),
			                                     numBlocks + 1);
			if (!detailOpt.has_value()) continue;

			// Collect array-entry offsets that pointed to `removed` and must be
			// deleted from their arrays (NifSkope removes array entries entirely).
			// Scalar refs that pointed to `removed` are set to -1 and kept.
			std::map<std::string, std::vector<size_t>> toDelete;  // arr1 → offsets

			for (const auto& rd : detailOpt->refs) {
				if (rd.offset + 4 > block.size()) continue;
				int32_t v = -1;
				std::memcpy(&v, block.data() + rd.offset, 4);

				if (v == removed) {
					if (rd.arr1.empty()) {
						// Scalar Ref → null it
						int32_t null32 = -1;
						std::memcpy(block.data() + rd.offset, &null32, 4);
					} else {
						// Array Ref → mark for deletion (NifSkope removes the entry)
						toDelete[rd.arr1].push_back(rd.offset);
					}
				} else if (v > removed) {
					--v;
					std::memcpy(block.data() + rd.offset, &v, 4);
				}
			}

			// Erase marked array entries from highest offset to lowest, then
			// decrement the count field for each affected array.
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

		parsed.blocks.erase(parsed.blocks.begin() + removed);
		parsed.blockTypeIndex.erase(parsed.blockTypeIndex.begin() + removed);
	}

	// ── Main algorithm ────────────────────────────────────────────────────────
	bool removeBogusNiNodes(ParsedNif& parsed)
	{
		const NifSchema& schema = globalNifSchema();
		bool anyChanged = false;
		bool removed;
		int  pass = 0;

		do {
			removed = false;
			++pass;
			const int32_t numBlocks = static_cast<int32_t>(parsed.blocks.size());
			// logger::info("[BogusNodes] === Pass {} ({} blocks) ===", pass, numBlocks);

			for (int32_t b = 0; b < numBlocks; ++b) {

				// ── IF: block is NiNode ───────────────────────────────────────
				uint16_t tIdx = parsed.blockTypeIndex[static_cast<size_t>(b)];
				const std::string& typeName =
					(tIdx < parsed.blockTypes.size()) ? parsed.blockTypes[tIdx] : std::string{};

				if (typeName != "NiNode") {
					// logger::info("[BogusNodes]   b={} type='{}' → NOT NiNode, skip", b, typeName);
					continue;
				}

				// Read name
				std::string nodeName;
				const auto& block = parsed.blocks[static_cast<size_t>(b)];
				if (block.size() >= 4) {
					int32_t nameIdx = -1;
					std::memcpy(&nameIdx, block.data(), 4);
					if (nameIdx >= 0 && nameIdx < static_cast<int32_t>(parsed.strings.size()))
						nodeName = parsed.strings[static_cast<size_t>(nameIdx)];
				}

				// logger::info("[BogusNodes]   b={} '{}' IS NiNode → checking pre-condition", b, nodeName);

				// ── IF: getChildLinks(b).isEmpty() && getParentLinks(b).isEmpty() ─
				const bool walkerLog = false;  // set to (pass == 1) to enable per-field Walker trace
				auto childOut  = getChildLinks (schema, typeName, block, numBlocks, walkerLog);
				auto parentOut = getParentLinks(schema, typeName, block, numBlocks, -1, walkerLog);

				// logger::info("[BogusNodes]     getChildLinks({})={} empty={}", b, setToStr(childOut), childOut.empty());
				// logger::info("[BogusNodes]     getParentLinks({})={} empty={}", b, setToStr(parentOut), parentOut.empty());

				if (!childOut.empty() || !parentOut.empty()) {
					// logger::info("[BogusNodes]     pre-condition FALSE (has outgoing links) → skip b={}", b);
					continue;
				}
				// logger::info("[BogusNodes]     pre-condition TRUE → continuing");

				// ── x = important check ───────────────────────────────────────
				bool impSubtype = isImportantSubtype(typeName);
				int x = impSubtype ? 1 : 0;
				// logger::info("[BogusNodes]     isImportantSubtype('{}')={} → x starts at {}", typeName, impSubtype, x);

				if (!x && parsed.bsVersion >= 14) {
					bool in = isImportantNodeName(nodeName, parsed.bsVersion);
					// logger::info("[BogusNodes]     bsVersion={} >= 14, isImportantNodeName('{}')={}", parsed.bsVersion, nodeName, in);
					if (in) x = 1;
				}
				// logger::info("[BogusNodes]     x after name/type check = {}", x);

				// ── FOR: each other block c ───────────────────────────────────
				for (int32_t c = 0; c < numBlocks && x < 2; ++c) {
					if (c == b) continue;

					uint16_t cTIdx = parsed.blockTypeIndex[static_cast<size_t>(c)];
					const std::string& cType =
						(cTIdx < parsed.blockTypes.size()) ? parsed.blockTypes[cTIdx] : std::string{};

					auto cl = getChildLinks (schema, cType, parsed.blocks[static_cast<size_t>(c)], numBlocks);
					bool clHas = cl.count(b) > 0;
					if (clHas) {
						// logger::info("[BogusNodes]     c={} '{}' getChildLinks contains b={} → x++ ({}→{})",
						//              c, cType, b, x, x + 1);
						++x;
					}

					if (x < 2) {
						auto pl = getParentLinks(schema, cType, parsed.blocks[static_cast<size_t>(c)], numBlocks);
						bool plHas = pl.count(b) > 0;
						if (plHas) {
							// logger::info("[BogusNodes]     c={} '{}' getParentLinks contains b={} → x=2 (DISQUALIFY)",
							//              c, cType, b);
							x = 2;
						}
					}
				}

				// logger::info("[BogusNodes]     final x={} → {}", x, (x < 2 ? "REMOVE" : "KEEP"));

				// ── IF: x < 2 → remove ────────────────────────────────────────
				if (x < 2) {
					// logger::info("[BogusNodes]   Removing block {} '{}' (type={}, x={})",
					//              b, nodeName, typeName, x);
					remapAfterRemoval(parsed, b, schema);
					removed    = true;
					anyChanged = true;
					break;
				}
			}

			// if (!removed)
			// 	logger::info("[BogusNodes] === Pass {} complete: no removals, done ===", pass);
			// else
			// 	logger::info("[BogusNodes] === Pass {} complete: one block removed, restarting ===", pass);

		} while (removed);

		// logger::info("[BogusNodes] removeBogusNiNodes finished: anyChanged={}", anyChanged);
		return anyChanged;
	}
}
