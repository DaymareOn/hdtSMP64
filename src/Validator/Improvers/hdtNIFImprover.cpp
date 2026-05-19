#include "hdtNIFImprover.h"

#include "hdtNIFBinaryIO.h"

#include "hdtNIFBogusNodeImprover.h"

#include "hdtNIFCollisionDecimationImprover.h"

#include "../Utils/hdtNIFBinaryUtils.h"
#include "../Utils/hdtStringUtils.h"

#include <cstring>
#include <filesystem>
#include <fstream>

namespace hdt
{
	static constexpr std::streamoff kMaxNifFileSizeBytes = 256 * 1024 * 1024;

	static std::string pathToUtf8(const std::filesystem::path& fp)
	{
		auto u8 = fp.generic_u8string();
		return { reinterpret_cast<const char*>(u8.data()), u8.size() };
	}

	static bool removeMissingPhysicsXmlExtraData(
		ParsedNif& parsed,
		const std::unordered_set<std::string>* missingPhysicsXmlRefs)
	{
		if (!missingPhysicsXmlRefs || missingPhysicsXmlRefs->empty())
			return false;

		logger::info("[NIFImprover] missingPhysicsXmlRefs set ({} entries):", missingPhysicsXmlRefs->size());
		for (const auto& ref : *missingPhysicsXmlRefs) {
		   logger::info("  - {}", ref);
		}

		int markerIdx = -1;
		for (int i = 0; i < static_cast<int>(parsed.strings.size()); ++i) {
			if (parsed.strings[static_cast<size_t>(i)] == nif::kPhysicsMarker) {
				markerIdx = i;
				break;
			}
		}
		if (markerIdx < 0)
			return false;

		int niStringExtraDataTypeIdx = -1;
		for (int i = 0; i < static_cast<int>(parsed.blockTypes.size()); ++i) {
			if (parsed.blockTypes[static_cast<size_t>(i)] == nif::kTypeNiStringExtraData) {
				niStringExtraDataTypeIdx = i;
				break;
			}
		}
		if (niStringExtraDataTypeIdx < 0)
			return false;

		bool changed = false;
		int markerMatchedBlocks = 0;
		int alternateLayoutMatches = 0;
		int valueMatchedFallbackBlocks = 0;
		constexpr size_t kUnsetOffset = static_cast<size_t>(-1);
		for (size_t i = 0; i < parsed.blocks.size(); ++i) {
			if (i >= parsed.blockTypeIndex.size())
				break;
			if (parsed.blockTypeIndex[i] != static_cast<uint16_t>(niStringExtraDataTypeIdx))
				continue;

			auto& block = parsed.blocks[i];
			if (block.size() < nif::kNiStringExtraDataMinBlockSize)
				continue;

			size_t nameOffset = kUnsetOffset;
			size_t firstXmlValueOffset = kUnsetOffset;
			size_t firstAnyValueOffset = kUnsetOffset;
			size_t firstMissingValueOffset = kUnsetOffset;

			const size_t laneCount = block.size() / sizeof(uint32_t);
			for (size_t lane = 0; lane < laneCount; ++lane) {
				const size_t off = lane * sizeof(uint32_t);
				uint32_t v = 0;
				std::memcpy(&v, block.data() + off, sizeof(uint32_t));

				if (v == static_cast<uint32_t>(markerIdx) && nameOffset == kUnsetOffset)
					nameOffset = off;

				if (v >= parsed.strings.size())
					continue;

				const std::string normalized = NormalizePathForComparison(parsed.strings[v]);
				const bool xmlLike = normalized.find(".xml") != std::string::npos;
				const bool missing = missingPhysicsXmlRefs->count(normalized) > 0;

				if (firstAnyValueOffset == kUnsetOffset)
					firstAnyValueOffset = off;
				if (xmlLike && firstXmlValueOffset == kUnsetOffset)
					firstXmlValueOffset = off;
				if (missing && firstMissingValueOffset == kUnsetOffset)
					firstMissingValueOffset = off;
			}

			const bool markerMatched = nameOffset != kUnsetOffset;
			size_t chosenValueOffset = kUnsetOffset;
			if (markerMatched) {
				// Prefer a plausible String Data lane after Name when marker is present.
				for (size_t lane = (nameOffset / sizeof(uint32_t)) + 1; lane < laneCount; ++lane) {
					const size_t off = lane * sizeof(uint32_t);
					uint32_t v = 0;
					std::memcpy(&v, block.data() + off, sizeof(uint32_t));
					if (v >= parsed.strings.size())
						continue;
					const std::string normalized = NormalizePathForComparison(parsed.strings[v]);
					if (normalized.find(".xml") != std::string::npos) {
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

			const std::string& xmlPathRaw = parsed.strings[chosenValueIdx];
			const std::string normalizedXmlPath = NormalizePathForComparison(xmlPathRaw);
			const bool missingByPath = firstMissingValueOffset != kUnsetOffset;

			if (markerMatched) {
				++markerMatchedBlocks;
				if (nameOffset == 4)
					++alternateLayoutMatches;
			} else if (missingByPath) {
				++valueMatchedFallbackBlocks;
			} else {
				continue;
			}

			logger::info(
				"[NIFImprover] Block {}: name@{} value@{} markerMatched={} raw='{}', normalized='{}'",
				i,
				markerMatched ? std::to_string(nameOffset) : std::string("none"),
				chosenValueOffset,
				markerMatched,
				xmlPathRaw,
				normalizedXmlPath);

			if (!missingByPath) {
				logger::info("[NIFImprover] Block {}: no missing XML value found in NiStringExtraData lanes, skipping", i);
				continue;
			}

			const size_t stripValueOffset = firstMissingValueOffset;
			uint32_t stripValueIdx = 0;
			std::memcpy(&stripValueIdx, block.data() + stripValueOffset, sizeof(uint32_t));
			logger::info("[NIFImprover] Block {}: STRIPPING missing XML value index {} at offset {}", i, stripValueIdx, stripValueOffset);

			const uint32_t kInvalidStringIndex = 0xFFFFFFFFu;
			if (markerMatched)
				std::memcpy(block.data() + nameOffset, &kInvalidStringIndex, sizeof(uint32_t));
			std::memcpy(block.data() + stripValueOffset, &kInvalidStringIndex, sizeof(uint32_t));
			changed = true;
		}

		if (markerMatchedBlocks == 0) {
			logger::warn(
				"[NIFImprover] No NiStringExtraData blocks referenced the physics marker (markerIdx={}, typeIdx={}, blockTypes={})",
				markerIdx,
				niStringExtraDataTypeIdx,
				parsed.blockTypes.size());
			if (valueMatchedFallbackBlocks > 0) {
				logger::warn("[NIFImprover] Applied value-path fallback stripping for {} block(s)", valueMatchedFallbackBlocks);
			}

			// Last-resort fallback: some files appear to carry valid Name/String Data pairs
			// that are not surfaced under NiStringExtraData in our parsed type index table.
			int rawPairFallbackBlocks = 0;
			for (size_t bi = 0; bi < parsed.blocks.size(); ++bi) {
				auto& block = parsed.blocks[bi];
				if (block.size() < sizeof(uint32_t) * 2)
					continue;

				const size_t laneCount = block.size() / sizeof(uint32_t);
				size_t markerOffset = kUnsetOffset;
				size_t missingValueOffset = kUnsetOffset;
				uint32_t missingValueIdx = 0;

				for (size_t lane = 0; lane < laneCount; ++lane) {
					const size_t off = lane * sizeof(uint32_t);
					uint32_t v = 0;
					std::memcpy(&v, block.data() + off, sizeof(uint32_t));

					if (v == static_cast<uint32_t>(markerIdx) && markerOffset == kUnsetOffset)
						markerOffset = off;

					if (v >= parsed.strings.size())
						continue;

					const std::string normalized = NormalizePathForComparison(parsed.strings[v]);
					if (missingPhysicsXmlRefs->count(normalized) > 0 && missingValueOffset == kUnsetOffset) {
						missingValueOffset = off;
						missingValueIdx = v;
					}
				}

				if (markerOffset == kUnsetOffset || missingValueOffset == kUnsetOffset)
					continue;

				logger::warn(
					"[NIFImprover] Raw-pair fallback stripping block {} (marker@{}, missingValueIdx={}@{})",
					bi,
					markerOffset,
					missingValueIdx,
					missingValueOffset);

				const uint32_t kInvalidStringIndex = 0xFFFFFFFFu;
				std::memcpy(block.data() + markerOffset, &kInvalidStringIndex, sizeof(uint32_t));
				std::memcpy(block.data() + missingValueOffset, &kInvalidStringIndex, sizeof(uint32_t));
				changed = true;
				++rawPairFallbackBlocks;
			}

			if (rawPairFallbackBlocks > 0) {
				logger::warn("[NIFImprover] Raw-pair fallback stripped {} block(s)", rawPairFallbackBlocks);
			}
		} else if (alternateLayoutMatches > 0) {
			logger::warn("[NIFImprover] Used alternate NiStringExtraData layout for {} block(s)", alternateLayoutMatches);
		}

		return changed;
	}

	bool GenerateImprovedNIF(
		const std::string& srcNIFPath,
		const std::string& outputDir,
		const NIFDecimationOptions& options,
		NIFImproverDiagnostics* outDiagnostics,
		const std::unordered_set<std::string>* missingPhysicsXmlRefs,
		bool copyOriginal)
	{
		logger::info("[NIFImprover] ENTRY: {} missingXMLrefs={} outputDir={}", srcNIFPath, missingPhysicsXmlRefs ? missingPhysicsXmlRefs->size() : 0, outputDir);
		namespace fs = std::filesystem;

		if (outDiagnostics)
			*outDiagnostics = {};

		std::ifstream in(std::filesystem::u8path(srcNIFPath), std::ios::binary | std::ios::ate);
		if (!in.is_open()) {
			logger::error("[NIFImprover] Early return: failed to open NIF file '{}'", srcNIFPath);
			return false;
		}
		auto sz = in.tellg();
		if (sz <= 0 || sz > kMaxNifFileSizeBytes) {
			logger::error("[NIFImprover] Early return: invalid file size={} for '{}'", static_cast<long long>(sz), srcNIFPath);
			return false;
		}
		std::vector<uint8_t> data(static_cast<size_t>(sz));
		in.seekg(0);
		in.read(reinterpret_cast<char*>(data.data()), sz);
		if (in.gcount() != static_cast<std::streamsize>(sz)) {
			logger::error(
				"[NIFImprover] Early return: short read for '{}', expected={} actual={}",
				srcNIFPath,
				static_cast<long long>(sz),
				static_cast<long long>(in.gcount()));
			return false;
		}

		std::string parseError;
		auto parsedOpt = parseNif(data, &parseError);
		if (!parsedOpt.has_value()) {
			logger::error(
				"[NIFImprover] Early return: parseNif failed for '{}': {}",
				srcNIFPath,
				parseError.empty() ? "unknown parse error" : parseError);
			return false;
		}
		auto& parsed = *parsedOpt;
		const char* layoutMode = parsed.hasExplicitEndiannessByte ? "explicit-endian-byte" : "legacy-no-endian-byte";
		const char* payloadEndian = parsed.endianness == 0 ? "big" : "little";
		logger::info(
			"[NIFImprover] Parse layout selected: mode={} payload-endian={} version=0x{:08X}",
			layoutMode,
			payloadEndian,
			parsed.version);
		logger::info(
			"[NIFImprover] Parsed NIF: blocks={}, strings={}, groups={}",
			parsed.blocks.size(),
			parsed.strings.size(),
			parsed.groups.size());

		if (!parsed.groups.empty()) {
			logger::error(
				"[NIFImprover] Early return: unsupported grouped NIF for '{}' (group count={})",
				srcNIFPath,
				parsed.groups.size());
			return false;
		}

		bool changed = false;
		bool bogusRemoved = removeBogusNiNodes(parsed);
		logger::info("[NIFImprover] removeBogusNiNodes returned: {}", bogusRemoved);
		changed |= bogusRemoved;
		
		bool xmlStripped = removeMissingPhysicsXmlExtraData(parsed, missingPhysicsXmlRefs);
		logger::info("[NIFImprover] removeMissingPhysicsXmlExtraData returned: {}", xmlStripped);
		changed |= xmlStripped;
		
		logger::info("[NIFImprover] changed={} after bogus/xml cleanup", changed);

		if (options.enableCollisionMeshDecimation) {
			auto d = runOfflineCollisionDecimationBridge(parsed, options, changed);
			if (outDiagnostics) {
				outDiagnostics->decimationCandidatesDiscovered = d.candidatesDiscovered;
				outDiagnostics->decimationCandidatesAttempted = d.candidatesAttempted;
				outDiagnostics->decimationCandidatesApplied = d.candidatesApplied;
				outDiagnostics->decimationCandidatesSkippedNoChange = d.candidatesSkippedNoChange;
				outDiagnostics->decimationCandidatesSkippedUnsafe = d.candidatesSkippedUnsafe;
				outDiagnostics->decimationSkipReasons = d.skipReasons;
			}

			if (d.candidatesAttempted > 0) {
				logger::info(
					"[Validator] Decimation bridge {}: discovered={}, attempted={}, applied={}, skipped-no-change={}, skipped-unsafe={}",
					srcNIFPath,
					d.candidatesDiscovered,
					d.candidatesAttempted,
					d.candidatesApplied,
					d.candidatesSkippedNoChange,
					d.candidatesSkippedUnsafe);
			}
		}

		if (!changed) {
			logger::info("[NIFImprover] changed=false, skipping file write and returning false");
			return false;
		}

		logger::info("[NIFImprover] changed=true, proceeding to round-trip validation");
		auto roundTripError = validateNifRoundTrip(parsed);
		if (roundTripError.has_value()) {
			logger::error("[NIFImprover] Round-trip validation FAILED: {}", *roundTripError);
			if (outDiagnostics)
				outDiagnostics->validationError = *roundTripError;
			return false;
		}
		logger::info("[NIFImprover] Round-trip validation passed");

		// Create output directory
		fs::path outPath = fs::path(outputDir) / stripDataPrefix(srcNIFPath);
		std::error_code ec;
		fs::create_directories(outPath.parent_path(), ec);
		if (ec) {
			logger::error("[NIFImprover] create_directories failed: {}", ec.message());
			return false;
		}

		logger::info("[NIFImprover] Writing NIF to: {}", pathToUtf8(outPath));
		if (!writeNifFile(parsed, pathToUtf8(outPath))) {
			logger::error("[NIFImprover] writeNifFile FAILED");
			return false;
		}
		logger::info("[NIFImprover] Write SUCCESS");

		if (copyOriginal) {
			fs::path originalOutPath = outPath;
			const std::string extension = originalOutPath.extension().string();
			originalOutPath.replace_filename(originalOutPath.stem().string() + "-original" + extension);
			fs::copy_file(fs::u8path(srcNIFPath), originalOutPath, fs::copy_options::overwrite_existing, ec);
			// Non-fatal: the improved NIF was already written; the original copy is for reference only.
		}

		return true;
	}

	bool CopyNIFToOutput(const std::string& srcNIFPath, const std::string& outputDir)
	{
		namespace fs = std::filesystem;
		std::string relative = stripDataPrefix(srcNIFPath);
		fs::path outPath = fs::path(outputDir) / relative;
		std::error_code ec;
		fs::create_directories(outPath.parent_path(), ec);
		if (ec)
			return false;
		fs::copy_file(fs::u8path(srcNIFPath), outPath, fs::copy_options::overwrite_existing, ec);
		return !ec;
	}
}
