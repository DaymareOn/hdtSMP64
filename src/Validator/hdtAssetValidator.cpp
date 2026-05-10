#include "hdtAssetValidator.h"

#include "ActorManager.h"
#include "NetImmerseUtils.h"
#include "Improvers/hdtNIFImprover.h"
#include "Validators/hdtNIFValidator.h"
#include "Validators/hdtSCHValidator.h"
#include "Utils/hdtStringUtils.h"
#include "Utils/hdtTimeUtils.h"
#include "Improvers/hdtXMLImprover.h"
#include "Validators/hdtXSDValidator.h"

#include <pugixml.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdt
{
	ValidationConfig g_validationConfig;

	static std::string PathToUtf8(const std::filesystem::path& fp)
	{
		auto u8 = fp.generic_u8string();
		return { reinterpret_cast<const char*>(u8.data()), u8.size() };
	}

	/// Constructs a NIFDecimationOptions struct from the current global validation config.
	/// Packages all mesh decimation settings for use by NIF improvers.
	static NIFDecimationOptions buildDecimationOptionsFromConfig()
	{
		NIFDecimationOptions o;
		o.enableCollisionMeshDecimation = g_validationConfig.decimateCollisionMeshesOffline;
		o.targetVertexRatio = g_validationConfig.decimationTargetVertexRatio;
		o.targetVertexCount = g_validationConfig.decimationTargetVertexCount;
		o.qemCostThreshold = g_validationConfig.decimationQemCostThreshold;
		o.shortEdgeRatio = g_validationConfig.decimationShortEdgeRatio;
		o.skinWeightPenalty = g_validationConfig.decimationSkinWeightPenalty;
		o.maxSkinWeightDrift = g_validationConfig.decimationMaxSkinWeightDrift;
		o.maxVolumeLossPercent = g_validationConfig.decimationMaxVolumeLossPercent;
		o.maxLocalVolumeChangePercent = g_validationConfig.decimationMaxLocalVolumeChangePercent;
		o.maxNormalDeviationDegrees = g_validationConfig.decimationMaxNormalDeviationDegrees;
		o.maxPointRemovals = g_validationConfig.decimationMaxPointRemovals;
		o.maxEdgeCollapses = g_validationConfig.decimationMaxEdgeCollapses;
		o.preserveBoundary = g_validationConfig.decimationPreserveBoundary;
		o.preserveFeatures = g_validationConfig.decimationPreserveFeatures;
		return o;
	}

	// ---- helpers ----

	/// Extracts the base stem from a NIF filename, stripping weight-variant suffixes.
	/// For files like "foo_0.nif" or "foo_1.nif", returns "foo".
	/// For unpaired NIFs like "foo.nif", returns "foo".
	/// Used to group _0/_1 NIF pairs for atomic processing.
	static std::string getNifPairBaseStem(const std::string& normPath)
	{
		if (normPath.size() > 6 &&
		    (normPath.substr(normPath.size() - 6) == "_0.nif" ||
		     normPath.substr(normPath.size() - 6) == "_1.nif"))
			return normPath.substr(0, normPath.size() - 6);
		return normPath.size() > 4 ? normPath.substr(0, normPath.size() - 4) : normPath;
	}

	/// Discovers TRI collision files related to a given NIF.
	/// TRI files are canonical and not weight-variant split:
	///   - foo.nif       -> foo.tri
	///   - foo_0.nif     -> foo.tri
	///   - foo_1.nif     -> foo.tri
	static std::vector<std::string> discoverRelatedTRIFiles(const std::string& nifPath)
	{
		namespace fs = std::filesystem;
		auto toStr = [](const fs::path& fp) -> std::string {
			auto u8 = fp.generic_u8string();
			return { reinterpret_cast<const char*>(u8.data()), u8.size() };
		};
		auto hasSuffix = [](const std::string& s, const char* suffix) {
			const size_t n = std::char_traits<char>::length(suffix);
			return s.size() >= n && s.compare(s.size() - n, n, suffix) == 0;
		};

		std::vector<std::string> result;
		std::unordered_set<std::string> seen;
		std::error_code ec;

		fs::path nifFsPath = nifPath;
		if (!nifFsPath.has_extension())
			return result;

		auto ext = toStr(nifFsPath.extension());
		std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
		if (ext != ".nif")
			return result;

		const fs::path parent = nifFsPath.parent_path();
		const std::string stem = toStr(nifFsPath.stem());

		std::string triStem = stem;
		if (stem.size() > 2 && (hasSuffix(stem, "_0") || hasSuffix(stem, "_1")))
			triStem = stem.substr(0, stem.size() - 2);

		std::vector<fs::path> candidates;
		candidates.push_back(parent / (triStem + ".tri"));

		for (const auto& candidate : candidates) {
			if (!fs::exists(candidate, ec) || ec || !fs::is_regular_file(candidate, ec) || ec)
				continue;

			auto pathStr = toStr(candidate);
			auto norm = NormalizePathForComparison(pathStr);
			if (seen.insert(norm).second)
				result.push_back(std::move(pathStr));
		}

		return result;
	}

	/// Resolves an XML path to a filesystem location, trying both prefixed and unprefixed variants.
	/// First attempts the path as "data/<rawPath>"; if not found, tries the raw path as-is.
	static std::pair<std::string, bool> resolveXMLPath(const std::string& rawPath)
	{
		if (rawPath.empty())
			return { {}, false };

		std::string xmlPath = rawPath;
		std::replace(xmlPath.begin(), xmlPath.end(), '\\', '/');

		namespace fs = std::filesystem;
		std::error_code ec;
		fs::path xmlFsPath = xmlPath;
		if (!fs::exists(xmlFsPath, ec))
			xmlFsPath = "data/" + xmlPath;

		return { PathToUtf8(xmlFsPath), fs::exists(xmlFsPath, ec) };
	}

	/// Distributes work across available CPU threads, splitting [0, n) into chunks.
	/// Each chunk is processed by a hardware-concurrency thread via std::async.
	/// Blocks until all chunks complete.
	template <typename F>
	static void parallelForChunks(size_t n, F chunkFn)
	{
		if (n == 0)
			return;
		const unsigned nThreads = std::max(1u, std::thread::hardware_concurrency());
		const size_t chunkSize = (n + nThreads - 1) / nThreads;
		std::vector<std::future<void>> futures;
		futures.reserve(nThreads);
		for (size_t i = 0; i < n; i += chunkSize) {
			size_t end = std::min(i + chunkSize, n);
			futures.push_back(std::async(std::launch::async, [chunkFn, i, end]() {
				chunkFn(i, end);
			}));
		}
		for (auto& f : futures)
			f.get();
	}

	using XMLValidationPair = std::pair<XSDValidationResult, SCHValidationResult>;

	static bool isIgnoredDisallowedChildTagViolation(const XSDViolation& v)
	{
		return v.message.find(" is not allowed inside <") != std::string::npos;
	}

	static bool isIgnoredInvalidSharedValueViolation(const XSDViolation& v)
	{
		return v.message.find("<shared> has invalid value '") != std::string::npos;
	}

	static bool isNonBlockingXsdViolation(const XSDViolation& v)
	{
		return isIgnoredDisallowedChildTagViolation(v) || isIgnoredInvalidSharedValueViolation(v);
	}

	static bool hasBlockingXsdErrors(const XSDValidationResult& xsd)
	{
		return std::any_of(xsd.violations.begin(), xsd.violations.end(), [](const XSDViolation& v) {
			return !isNonBlockingXsdViolation(v);
		});
	}

	/// Reports all XSD and SCH violations from a validated XML file to the report and output stream.
	/// Formats each violation with line number, element path, and message.
	/// Assumes a pass/fail header line has already been written before calling this.
	static void appendXmlViolationsToReport(const XMLValidationPair& pair, const std::string& xmlPath,
		AssetValidationResult& report, std::ostream& out)
	{
		const auto& [xsdResult, schResult] = pair;

		for (const auto& v : xsdResult.violations) {
			if (isIgnoredDisallowedChildTagViolation(v)) {
				std::string msg = xmlPath + ":" + std::to_string(v.line) + ": " +
				                  v.elementPath + " - " + v.message + " This tag will be ignored.";
				report.warnings.push_back(msg);
				report.hasWarnings = true;
				out << "    [WARNING] " << v.elementPath << " (line " << v.line << "): "
					<< v.message << "; this tag will be ignored." << "\n";
				continue;
			}

			if (isIgnoredInvalidSharedValueViolation(v)) {
				std::string msg = xmlPath + ":" + std::to_string(v.line) + ": " +
				                  v.elementPath + " - " + v.message +
				                  " This value will be ignored and replaced by the default value ('public').";
				report.warnings.push_back(msg);
				report.hasWarnings = true;
				out << "    [WARNING] " << v.elementPath << " (line " << v.line << "): "
					<< v.message << "; this value will be ignored and replaced by the default value ('public')." << "\n";
				continue;
			}

			std::string msg = xmlPath + ":" + std::to_string(v.line) + ": " +
			                  v.elementPath + " - " + v.message;
			report.errors.push_back(msg);
			out << "    [ERROR] " << v.elementPath << " (line " << v.line << "): "
				<< v.message << "\n";
		}

		for (const auto& v : schResult.violations) {
			std::string msg = xmlPath + ":" + std::to_string(v.line) + ": " +
			                  v.location + " - " + v.message;
			if (v.role == SCHRole::Error) {
				report.errors.push_back(msg);
				out << "    [ERROR] " << v.location << " (line " << v.line << "): "
					<< v.message << "\n";
			} else {
				report.warnings.push_back(msg);
				report.hasWarnings = true;
				out << "    [WARNING] " << v.location << " (line " << v.line << "): "
					<< v.message << "\n";
			}
		}
	}

	// ---- Parallel XML validation helpers ----

	/// Validates multiple XML files in parallel, running both XSD and SCH validators on each.
	/// Both validators use std::once_flag-protected schema loading, making this thread-safe.
	/// Results are returned in the same order as input paths.
	static std::vector<XMLValidationPair> parallelValidateXMLs(const std::vector<std::string>& paths)
	{
		std::vector<XMLValidationPair> results(paths.size());
		parallelForChunks(paths.size(), [&](size_t begin, size_t end) {
			for (size_t j = begin; j < end; ++j)
				results[j] = { ValidatePhysicsXMLWithXSD(paths[j]), ValidatePhysicsXMLWithSchematron(paths[j]) };
		});
		return results;
	}

	// ---- Phase 0: DefaultBBP XML discovery ----

	struct DefaultBBPEntry
	{
		std::string shape;    // shape name from <map shape="...">
		std::string xmlPath;  // resolved filesystem path (data/...)
		bool xmlExists = false;
	};

	// Parse defaultBBPs.xml and resolve the file path of each <map> entry.
	static std::vector<DefaultBBPEntry> discoverDefaultBBPXMLs()
	{
		std::vector<DefaultBBPEntry> result;
		namespace fs = std::filesystem;

		fs::path bbpFile = "data/SKSE/Plugins/hdtSkinnedMeshConfigs/defaultBBPs.xml";
		std::error_code ec;
		if (!fs::exists(bbpFile, ec)) {
			logger::info("[Validator] defaultBBPs.xml not found at {}, skipping Phase 0",
				PathToUtf8(bbpFile));
			return result;
		}

		pugi::xml_document doc;
		const std::string bbpPathUtf8 = PathToUtf8(bbpFile);
		std::string bbpBytes = readAllFile2(bbpPathUtf8.c_str());
		auto parseResult = doc.load_buffer(bbpBytes.data(), bbpBytes.size());
		if (!parseResult) {
			logger::warn("[Validator] Failed to parse defaultBBPs.xml: {}",
				parseResult.description());
			return result;
		}

		for (auto& map : doc.child("default-bbps").children("map")) {
			std::string shape = map.attribute("shape").as_string();
			std::string rawFile = map.attribute("file").as_string();
			if (shape.empty() || rawFile.empty())
				continue;

			// Normalise path separators
			std::replace(rawFile.begin(), rawFile.end(), '\\', '/');

			// Build candidate paths: try "data/<path>" first, then as-is
			DefaultBBPEntry entry;
			entry.shape = shape;

			fs::path candidate = "data/" + rawFile;
			if (fs::exists(candidate, ec)) {
				entry.xmlPath = PathToUtf8(candidate);
				entry.xmlExists = true;
			} else {
				// Fall back to path as-is (in case it's already absolute or differently rooted)
				candidate = rawFile;
				entry.xmlPath = PathToUtf8(candidate);
				entry.xmlExists = fs::exists(candidate, ec);
			}

			result.push_back(std::move(entry));
		}

		return result;
	}

	// ---- Physics asset discovery ----

	/// Discovers physics-enabled assets from either filesystem or runtime.
	/// When equippedOnly=false: Scans data/meshes for all NIF files with physics data.
	///   - Step 1 (serial): Recursive directory walk collects all .nif paths.
	///   - Step 2 (parallel): Binary scan detects physics data in each NIF.
	/// When equippedOnly=true: Iterates live actor skeletons to collect equipped armor and headparts.
	static std::vector<PhysicsAsset> discoverPhysicsAssets(bool equippedOnly = false,
		std::vector<std::string>* outNifScanViolations = nullptr,
		int* outFilesystemNifFilesDiscovered = nullptr,
		int* outEquippedNifsDiscovered = nullptr)
	{
		if (outNifScanViolations)
			outNifScanViolations->clear();
		if (outFilesystemNifFilesDiscovered)
			*outFilesystemNifFilesDiscovered = 0;
		if (outEquippedNifsDiscovered)
			*outEquippedNifsDiscovered = 0;

		if (equippedOnly) {
			// ---- Runtime-only: Equipped-gear discovery ----
			std::vector<PhysicsAsset> result;
			int equippedCount = 0;

			auto* actorManager = ActorManager::instance();
			auto lock = actorManager->lockGuard();
			auto& skeletons = actorManager->getSkeletons();

			for (auto& skeleton : skeletons) {
				for (const auto& armor : skeleton.getArmors()) {
					if (!armor.armorWorn || !armor.armorWorn->parent)
						continue;
					if (armor.physicsFile.first.empty())
						continue;

					auto [xmlPath, xmlExists] = resolveXMLPath(armor.physicsFile.first);
					std::string armorName = armor.armorWorn->name.size() ? armor.armorWorn->name.c_str() : "<unnamed>";
					PhysicsAsset asset;
					// For equipped-gear validation we don't have stable NIF file paths at runtime,
					// so nifPath is used as a human-readable item identifier in the report output.
					asset.nifPath = skeleton.name() + " [armor:" + armorName + "]";
					asset.nifExists = true;
					asset.xmlPath = std::move(xmlPath);
					asset.xmlExists = xmlExists;
					asset.allPhysicsXmlPaths.push_back(armor.physicsFile.first);
					++equippedCount;
					if (outNifScanViolations) {
						if (auto* armorRoot = castNiNode(armor.armorWorn.get())) {
							auto structural = validateNIFStructure(armorRoot, asset.nifPath);
							for (const auto& err : structural.errors)
								outNifScanViolations->push_back(err);
							for (const auto& warn : structural.warnings)
								logger::warn("[Validator] Equipped NIF warning: {}", warn);
						} else {
							outNifScanViolations->push_back(asset.nifPath + ": equipped armor node is not a NiNode");
						}
					}
					result.push_back(std::move(asset));
				}

				if (!skeleton.head.headNode)
					continue;

				for (const auto& headPart : skeleton.head.headParts) {
					if (!headPart.headPart || !headPart.headPart->parent)
						continue;
					if (headPart.physicsFile.first.empty())
						continue;

					auto [xmlPath, xmlExists] = resolveXMLPath(headPart.physicsFile.first);
					std::string headPartName = headPart.headPart->name.size() ? headPart.headPart->name.c_str() : "<unnamed>";
					PhysicsAsset asset;
					asset.nifPath = skeleton.name() + " [headpart:" + headPartName + "]";
					asset.nifExists = true;
					asset.xmlPath = std::move(xmlPath);
					asset.xmlExists = xmlExists;
					asset.allPhysicsXmlPaths.push_back(headPart.physicsFile.first);
					++equippedCount;
					if (outNifScanViolations) {
						if (auto* headRoot = castNiNode(headPart.headPart.get())) {
							auto structural = validateNIFStructure(headRoot, asset.nifPath);
							for (const auto& err : structural.errors)
								outNifScanViolations->push_back(err);
							for (const auto& warn : structural.warnings)
								logger::warn("[Validator] Equipped NIF warning: {}", warn);
						} else {
							outNifScanViolations->push_back(asset.nifPath + ": equipped headpart node is not a NiNode");
						}
					}
					result.push_back(std::move(asset));
				}
			}
			if (outEquippedNifsDiscovered)
				*outEquippedNifsDiscovered = equippedCount;

			return result;
		} else {
			// ---- Filesystem: NIF discovery ----
			// Two-step: serial directory walk to collect paths, then parallel binary scan.
			namespace fs = std::filesystem;
			auto toStr = [](const fs::path& fp) -> std::string {
				auto u8 = fp.generic_u8string();
				return { reinterpret_cast<const char*>(u8.data()), u8.size() };
			};
			fs::path meshDir = "data/meshes";

			std::error_code ec;
			if (!fs::exists(meshDir, ec) || !fs::is_directory(meshDir, ec)) {
				logger::warn("[Validator] Meshes directory not found: {}", PathToUtf8(meshDir));
				return {};
			}

			// Step 1: serial directory walk — collect all .nif paths.
			// Filesystem iteration is not required to be thread-safe, so keep it serial.
			std::vector<std::string> nifPaths;
			for (auto& entry : fs::recursive_directory_iterator(meshDir, ec)) {
				if (ec) {
					logger::warn("[Validator] Mesh directory iteration error: {}", ec.message());
					break;
				}
				if (!entry.is_regular_file(ec) || ec)
					continue;
				auto& p = entry.path();
				auto ext = toStr(p.extension());
				std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
				if (ext != ".nif")
					continue;
				try {
					nifPaths.push_back(toStr(p));
				} catch (const std::exception& e) {
					logger::warn("[Validator] Skipping unrepresentable path: {}", e.what());
				}
			}

			logger::info("[Validator] Found {} NIF files, scanning for physics data...", nifPaths.size());
			if (outFilesystemNifFilesDiscovered)
				*outFilesystemNifFilesDiscovered = static_cast<int>(nifPaths.size());

			// Step 2: parallel binary scan — each thread scans an independent chunk.
			// ExtractPhysicsXmlRefsFromNIFs opens its own file handle and uses only local state → thread-safe.
			const size_t n = nifPaths.size();
			std::vector<std::optional<PhysicsAsset>> scanResults(n);
			std::vector<std::vector<std::string>> scanViolations(n);

			parallelForChunks(n, [&](size_t begin, size_t end) {
				for (size_t j = begin; j < end; ++j) {
					const auto& pathStr = nifPaths[j];
					try {
						auto scanRes = ExtractPhysicsXmlRefsFromNIFs(pathStr);
						for (const auto& err : scanRes.errors)
							scanViolations[j].push_back(err);

						if (!scanRes.hasPhysicsData)
							continue;

						PhysicsAsset asset;
						asset.nifPath = pathStr;
						asset.nifExists = true;
						asset.relatedTRIPaths = discoverRelatedTRIFiles(pathStr);
						asset.allPhysicsXmlPaths = scanRes.allPhysicsXmlPaths;

						if (!scanRes.physicsXmlPath.empty()) {
							std::string xmlPath = scanRes.physicsXmlPath;
							std::replace(xmlPath.begin(), xmlPath.end(), '\\', '/');

							std::error_code ec2;
							namespace fs2 = std::filesystem;
							fs2::path xmlFsPath = xmlPath;
							if (!fs2::exists(xmlFsPath, ec2))
								xmlFsPath = "data/" + xmlPath;
							asset.xmlPath = toStr(xmlFsPath);
							asset.xmlExists = fs2::exists(xmlFsPath, ec2);
						}

						scanResults[j] = std::move(asset);
					} catch (const std::exception& e) {
						scanViolations[j].push_back("Exception while scanning NIF '" + pathStr + "': " + e.what());
						logger::warn("[Validator] Error scanning NIF {}: {}", pathStr, e.what());
					} catch (...) {
						scanViolations[j].push_back("Unknown exception while scanning NIF: " + pathStr);
						logger::warn("[Validator] Unknown error scanning NIF {}", pathStr);
					}
				}
			});

			// Collect physics-enabled NIFs, preserving discovery order.
			std::vector<PhysicsAsset> result;
			result.reserve(n);
			for (auto& opt : scanResults) {
				if (opt)
					result.push_back(std::move(*opt));
			}
			if (outNifScanViolations) {
				for (auto& errs : scanViolations)
					for (auto& err : errs)
						outNifScanViolations->push_back(std::move(err));
			}

			logger::info("[Validator] Scanned {} NIF files, found {} physics-enabled NIFs",
				nifPaths.size(), result.size());
			return result;
		}
	}

	/// Collects unique physics XML paths from both DefaultBBP entries and discovered assets.
	/// Deduplicates by normalized path to avoid processing the same file multiple times.
	/// Only includes XMLs that exist on disk.
	static std::vector<std::string> collectPhysicsXMLPaths(bool equippedOnly = false)
	{
		std::unordered_set<std::string> seen;
		std::vector<std::string> queue;

		// Inline dedup: enqueue unique paths only
		auto enqueueUnique = [&](const std::string& path) {
			if (path.empty())
				return;
			auto norm = NormalizePathForComparison(path);
			if (seen.insert(norm).second)
				queue.push_back(path);
		};

		// Enqueue DefaultBBP XMLs
		auto bbpEntries = discoverDefaultBBPXMLs();
		for (const auto& entry : bbpEntries)
			if (entry.xmlExists)
				enqueueUnique(entry.xmlPath);

		// Enqueue physics asset XMLs (NIFs or equipped items)
		auto physicsAssets = discoverPhysicsAssets(equippedOnly);
		for (const auto& asset : physicsAssets)
			if (asset.xmlExists)
				enqueueUnique(asset.xmlPath);

		return queue;
	}

	/// Generates improved versions of physics XML files in the output directory.
	/// Removes unknown/misplaced elements and corrects formatting.
	static XMLImproveResult improveXMLPaths(const std::vector<std::string>& xmlPaths,
		const std::string& outputDir)
	{
		XMLImproveResult result;
		if (outputDir.empty()) {
			result.errors.push_back("Output directory is empty");
			return result;
		}

		result.totalXMLsFound = static_cast<int>(xmlPaths.size());
		if (xmlPaths.empty())
			return result;

		for (const auto& xmlPath : xmlPaths) {
			try {
				if (GenerateImprovedXML(xmlPath, outputDir))
					++result.xmlImprovedCount;
			} catch (const std::exception& e) {
				result.errors.push_back("Failed to improve XML " + xmlPath + ": " + e.what());
			} catch (...) {
				result.errors.push_back("Failed to improve XML " + xmlPath + ": unknown error");
			}
		}

		return result;
	}

	// ---- Phase 0: DefaultBBP XML validation ----

	// ---- Phase 2.5: NIF _0/_1 pair consistency check ----

	/// Validates that _0.nif and _1.nif NIF pairs reference the same physics XML at the same block positions.
	/// For every _0.nif, checks that the matching _1.nif exists and references identical physics data.
	/// Emits errors if pairs are missing or mismatched.
	static void validateNifPairConsistency(const std::vector<PhysicsAsset>& nifAssets,
		AssetValidationResult& report, std::ostream& out)
	{
		// Build a fast lookup: normalised nif path -> index in nifAssets
		std::unordered_map<std::string, size_t> nifByNormPath;
		nifByNormPath.reserve(nifAssets.size());
		for (size_t i = 0; i < nifAssets.size(); ++i)
			nifByNormPath[NormalizePathForComparison(nifAssets[i].nifPath)] = i;

		// Track _0.nif paths we've already checked (avoid reporting the same pair twice)
		std::unordered_set<std::string> checked;

		for (size_t i = 0; i < nifAssets.size(); ++i) {
			const auto& asset = nifAssets[i];

			// We only initiate checks from the _0.nif side
			auto normPath = NormalizePathForComparison(asset.nifPath);
			if (!normPath.ends_with("_0.nif"))
				continue;
			if (checked.count(normPath))
				continue;
			checked.insert(normPath);

			// Derive the expected _1.nif path
			std::string norm1 = normPath.substr(0, normPath.size() - 6) + "_1.nif";

			auto it1 = nifByNormPath.find(norm1);
			if (it1 == nifByNormPath.end()) {
				// _1.nif has no physics data or does not exist — only warn if _0 has physics
				if (!asset.xmlPath.empty()) {
					std::string msg = asset.nifPath + ": _0.nif has physics data but the matching _1.nif (" + norm1 + ") was not found or has no physics reference.";
					report.errors.push_back(msg);
					report.hasErrors = true;
					out << "  [PAIR-ERROR] " << asset.nifPath << "\n";
					out << "    Matching _1.nif not found or has no physics reference: " << norm1 << "\n";
				}
				continue;
			}

			const auto& asset1 = nifAssets[it1->second];

			// 1. Both must reference the same XML (normalised)
			auto normXml0 = NormalizePathForComparison(asset.xmlPath);
			auto normXml1 = NormalizePathForComparison(asset1.xmlPath);
			if (normXml0 != normXml1) {
				std::string msg = asset.nifPath + " and " + asset1.nifPath +
				                  ": _0/_1 NIF pair reference different physics XMLs: '" +
				                  asset.xmlPath + "' vs '" + asset1.xmlPath + "'.";
				report.errors.push_back(msg);
				report.hasErrors = true;
				out << "  [PAIR-ERROR] " << asset.nifPath << "\n";
				out << "    _0.nif XML: " << (asset.xmlPath.empty() ? "(none)" : asset.xmlPath) << "\n";
				out << "    _1.nif XML: " << (asset1.xmlPath.empty() ? "(none)" : asset1.xmlPath) << "\n";
				out << "    _0/_1 NIF pair reference different physics XMLs.\n";
			}

			// 2. Both must have the same number of physics blocks at the same positions
			const auto& paths0 = asset.allPhysicsXmlPaths;
			const auto& paths1 = asset1.allPhysicsXmlPaths;
			if (paths0.size() != paths1.size()) {
				std::string msg = asset.nifPath + " and " + asset1.nifPath +
				                  ": _0/_1 NIF pair have a different number of physics XML blocks (" +
				                  std::to_string(paths0.size()) + " vs " + std::to_string(paths1.size()) + ").";
				report.errors.push_back(msg);
				report.hasErrors = true;
				out << "  [PAIR-ERROR] " << asset.nifPath << " vs " << asset1.nifPath << "\n";
				out << "    Block count mismatch: _0.nif has " << paths0.size()
					<< " block(s), _1.nif has " << paths1.size() << " block(s).\n";
			} else {
				for (size_t k = 0; k < paths0.size(); ++k) {
					if (NormalizePathForComparison(paths0[k]) != NormalizePathForComparison(paths1[k])) {
						std::string msg = asset.nifPath + " and " + asset1.nifPath +
						                  ": _0/_1 NIF pair have different physics XML at block index " +
						                  std::to_string(k) + ": '" + paths0[k] + "' vs '" + paths1[k] + "'.";
						report.errors.push_back(msg);
						report.hasErrors = true;
						out << "  [PAIR-ERROR] " << asset.nifPath << " vs " << asset1.nifPath << "\n";
						out << "    Block " << k << " mismatch:\n";
						out << "      _0.nif: " << paths0[k] << "\n";
						out << "      _1.nif: " << paths1[k] << "\n";
					}
				}
			}
		}
	}

	// ---- Phase 3: NIF-based validation ----

	/// Validates physics XMLs referenced by NIF assets with per-NIF error context.
	/// Validates each unique XML, warns if NIFs reference missing XMLs or have multiple physics blocks,
	/// and deduplicates validation results across NIFs sharing the same XML file.
	static void validateNIFAssets(const std::vector<PhysicsAsset>& nifAssets,
		AssetValidationResult& report, std::ostream& out)
	{
		// Pre-collect unique XML paths from all NIF assets (serial dedup).
		// xmlToIdx maps normalised path → index in batch.
		std::unordered_map<std::string, size_t> xmlToIdx;
		std::vector<std::string> batch;
		for (const auto& asset : nifAssets) {
			if (asset.xmlPath.empty() || !asset.xmlExists)
				continue;
			auto norm = NormalizePathForComparison(asset.xmlPath);
			if (!xmlToIdx.count(norm)) {
				xmlToIdx[norm] = batch.size();
				batch.push_back(asset.xmlPath);
			}
		}

		// Parallel validate all unique XMLs.
		auto batchResults = parallelValidateXMLs(batch);

		// Report per-NIF in original order (serial).
		// reportedXMLs tracks which XMLs have already been reported within Phase 3
		// (multiple NIFs often share the same physics XML).
		std::unordered_set<std::string> reportedXMLs;

		for (const auto& asset : nifAssets) {
			out << "  [NIF]  " << asset.nifPath << "\n";

			// Warn if multiple "HDT Skinned Mesh Physics Object" blocks found
			if (asset.allPhysicsXmlPaths.size() > 1) {
				std::string msg = asset.nifPath + ": has " +
				                  std::to_string(asset.allPhysicsXmlPaths.size()) +
				                  " \"HDT Skinned Mesh Physics Object\" blocks; only the first is used by the runtime.";
				report.warnings.push_back(msg);
				report.hasWarnings = true;
				out << "    [WARNING] Multiple \"HDT Skinned Mesh Physics Object\" blocks found ("
					<< asset.allPhysicsXmlPaths.size() << "); only the first is used:\n";
				for (const auto& p : asset.allPhysicsXmlPaths)
					out << "      - " << p << "\n";
			}

			if (!asset.xmlExists && !asset.xmlPath.empty()) {
				std::string err = "NIF " + asset.nifPath +
				                  " references missing XML: " + asset.xmlPath;
				report.errors.push_back(err);
				report.hasErrors = true;
				out << "    [ERROR] Referenced XML not found: " << asset.xmlPath << "\n";
			} else if (!asset.xmlPath.empty()) {
				out << "    -> " << asset.xmlPath << "\n";

				auto norm = NormalizePathForComparison(asset.xmlPath);
				if (reportedXMLs.count(norm)) {
					out << "    (already validated)\n";
					continue;
				}
				reportedXMLs.insert(norm);
				++report.totalXMLsFound;

				const auto& pair = batchResults[xmlToIdx[norm]];
				bool xmlHasErrors = hasBlockingXsdErrors(pair.first) || pair.second.hasErrors;

				if (xmlHasErrors) {
					++report.xmlErrorCount;
					report.hasErrors = true;
				} else {
					++report.xmlPassCount;
				}

				appendXmlViolationsToReport(pair, asset.xmlPath, report, out);
			} else {
				out << "    [WARN] Could not determine XML path from NIF\n";
			}
		}
	}

	// ---- report writing ----

	/// Writes validation report content to a timestamped file in the SKSE log directory.
	static std::string writeValidationReportFile(const std::string& reportContent,
		const std::string& timestamp)
	{
		auto logDir = logger::log_directory();
		if (!logDir) {
			logger::warn("[Validator] Could not determine log directory for report");
			return {};
		}

		auto reportPath = *logDir / ("hdtSMP64_validation_" + timestamp + ".log");

		std::ofstream out(reportPath, std::ios::out | std::ios::trunc);
		if (!out.is_open()) {
			logger::warn("[Validator] Could not open report file: {}", PathToUtf8(reportPath));
			return {};
		}

		out << reportContent;
		logger::info("[Validator] Validation report written to: {}", PathToUtf8(reportPath));
		return PathToUtf8(reportPath);
	}

	// ---- core validation ----

	/// Executes the complete validation pipeline (full or equipped-only) and generates a report.
	/// Full pipeline (equippedOnly=false):
	///   - Phase 0: Validates DefaultBBP XML entries.
	///   - Phase 2: Discovers NIFs and validates _0/_1 pair consistency.
	///   - Phase 3: Validates NIF-referenced XMLs with NIF context.
	///   - Phase 4-5: Generates improved XMLs and NIFs if configured.
	/// Equipped-only pipeline (equippedOnly=true):
	///   - Discovers equipped armor and headparts.
	///   - Validates their physics XMLs.
	static std::string runValidationCore(AssetValidationResult& report, const std::string& timestamp, bool equippedOnly = false)
	{
		auto wallStart = std::chrono::steady_clock::now();
		std::ostringstream bodyStream;

		if (equippedOnly) {
			// ---- Simplified: Equipped-only validation ----
			bodyStream << "== Phase 1: Equipped Gear Discovery ==\n";
			std::vector<std::string> nifScanViolations;
			int equippedNifsDiscovered = 0;
			auto equippedAssets = discoverPhysicsAssets(true, &nifScanViolations, nullptr, &equippedNifsDiscovered);
			report.equippedNifsDiscovered = equippedNifsDiscovered;
			report.filesystemNifFilesDiscovered = 0;
			report.nifScanViolationCount = static_cast<int>(nifScanViolations.size());
			report.totalNIFsScanned = report.equippedNifsDiscovered;
			bodyStream << "  Found " << equippedAssets.size() << " equipped physics item(s).\n";
			bodyStream << "  NIF discovery metrics: filesystem=0, equipped=" << report.equippedNifsDiscovered
				<< ", scan violations=" << report.nifScanViolationCount << "\n";

			if (!nifScanViolations.empty()) {
				bodyStream << "\n  -- NIF Scan Violations --\n";
				bodyStream << "  Count: " << nifScanViolations.size() << "\n";
				for (const auto& violation : nifScanViolations) {
					report.errors.push_back(violation);
					report.hasErrors = true;
					bodyStream << "    [ERROR] " << violation << "\n";
				}
			}

			if (!equippedAssets.empty()) {
				bodyStream << "\n== Phase 2: Equipped Gear XML Validation ==\n";
				validateNIFAssets(equippedAssets, report, bodyStream);
			}
		} else {
			// ---- Full: Comprehensive validation pipeline ----
			// Shared dedup set for this run's XML validations.
			std::unordered_set<std::string> globalValidatedXMLs;

			// Phase 0: DefaultBBP XML validation
			auto bbpEntries = discoverDefaultBBPXMLs();
			if (!bbpEntries.empty()) {
				bodyStream << "== Phase 0: DefaultBBP XML Validation ==\n";
				bodyStream << "  Found " << bbpEntries.size() << " map entries in defaultBBPs.xml.\n";

				std::vector<size_t> validBatchIdx(bbpEntries.size(), SIZE_MAX);
				std::vector<std::string> batch;
				for (size_t i = 0; i < bbpEntries.size(); ++i) {
					const auto& entry = bbpEntries[i];
					if (entry.xmlPath.empty() || !entry.xmlExists)
						continue;

					auto norm = NormalizePathForComparison(entry.xmlPath);
					if (globalValidatedXMLs.count(norm))
						continue;

					globalValidatedXMLs.insert(norm);
					validBatchIdx[i] = batch.size();
					batch.push_back(entry.xmlPath);
				}

				auto batchResults = parallelValidateXMLs(batch);

				for (size_t i = 0; i < bbpEntries.size(); ++i) {
					const auto& entry = bbpEntries[i];
					size_t batchIdx = validBatchIdx[i];

					bodyStream << "  [BBP]  shape=" << entry.shape << " -> " << entry.xmlPath << "\n";

					if (!entry.xmlExists) {
						std::string err = "defaultBBPs.xml: shape '" + entry.shape + "' references missing XML: " + entry.xmlPath;
						report.errors.push_back(err);
						report.hasErrors = true;
						bodyStream << "    [ERROR] XML file not found\n";
						continue;
					}

					if (batchIdx == SIZE_MAX) {
						bodyStream << "    (already validated)\n";
						continue;
					}

					++report.totalXMLsFound;
					const auto& pair = batchResults[batchIdx];
					bool fileHasErrors = !pair.first.isValid || pair.second.hasErrors;

					if (fileHasErrors) {
						++report.xmlErrorCount;
						report.hasErrors = true;
						bodyStream << "    [FAIL]\n";
					} else {
						++report.xmlPassCount;
						bodyStream << "    [OK]\n";
					}

					appendXmlViolationsToReport(pair, entry.xmlPath, report, bodyStream);
				}
				bodyStream << "\n";
			}

			// Phase 2: NIF discovery
			bodyStream << "\n== Phase 2: NIF File Discovery ==\n";
			std::vector<std::string> nifScanViolations;
			int filesystemNifFilesDiscovered = 0;
			auto nifAssets = discoverPhysicsAssets(false, &nifScanViolations, &filesystemNifFilesDiscovered, nullptr);
			report.filesystemNifFilesDiscovered = filesystemNifFilesDiscovered;
			report.equippedNifsDiscovered = 0;
			report.nifScanViolationCount = static_cast<int>(nifScanViolations.size());
			report.totalNIFsScanned = report.filesystemNifFilesDiscovered;
			std::unordered_set<std::string> relatedTRINorm;
			for (const auto& a : nifAssets)
				for (const auto& tri : a.relatedTRIPaths)
					relatedTRINorm.insert(NormalizePathForComparison(tri));
			bodyStream << "  Scanned " << filesystemNifFilesDiscovered << " NIF file(s) in data/meshes.\n";
			bodyStream << "  Found " << nifAssets.size() << " NIF file(s) referencing physics configs.\n";
			bodyStream << "  Identified " << relatedTRINorm.size() << " related TRI file(s).\n";

			if (!nifScanViolations.empty()) {
				bodyStream << "\n  -- NIF Scan Violations --\n";
				bodyStream << "  Count: " << nifScanViolations.size() << "\n";
				for (const auto& violation : nifScanViolations) {
					report.errors.push_back(violation);
					report.hasErrors = true;
					bodyStream << "    [ERROR] " << violation << "\n";
				}
			}

			// Phase 2.5: NIF _0/_1 pair consistency check
			bodyStream << "\n== Phase 2.5: NIF Pair Consistency Check ==\n";
			validateNifPairConsistency(nifAssets, report, bodyStream);

			// Phase 3: NIF-referenced XML validation
			if (!nifAssets.empty()) {
				bodyStream << "\n== Phase 3: NIF-Referenced XML Validation ==\n";
				validateNIFAssets(nifAssets, report, bodyStream);
			}

			// Phase 4: Improved XML generation
			// Collect all unique XML paths validated across all phases, then generate
			// improved copies (unknown / misplaced elements removed) for each.
			if (!g_validationConfig.outputDir.empty()) {
				bodyStream << "\n== Phase 4: Improved XML Generation ==\n";
				bodyStream << "  Output directory: " << g_validationConfig.outputDir << "\n";

				std::unordered_set<std::string> improveNorm;
				std::vector<std::string> improveQueue;

				auto enqueue = [&](const std::string& path) {
					if (path.empty())
						return;
					auto norm = NormalizePathForComparison(path);
					if (improveNorm.insert(norm).second)
						improveQueue.push_back(path);
				};

				for (const auto& e : bbpEntries)
					if (e.xmlExists)
						enqueue(e.xmlPath);
				for (const auto& a : nifAssets)
					if (a.xmlExists)
						enqueue(a.xmlPath);

				for (const auto& xmlPath : improveQueue) {
					if (GenerateImprovedXML(xmlPath, g_validationConfig.outputDir)) {
						++report.xmlImprovedCount;
						bodyStream << "  [IMPROVED] " << xmlPath << "\n";
					}
				}

				bodyStream << "  " << report.xmlImprovedCount << " improved file(s) written.\n";
			}

			// Phase 5: Improved NIF generation
			if (!g_validationConfig.outputDir.empty() && g_validationConfig.improveNIFs && !nifAssets.empty()) {
				bodyStream << "\n== Phase 5: Improved NIF Generation ==\n";
				bodyStream << "  Output directory: " << g_validationConfig.outputDir << "\n";
				if (g_validationConfig.decimateCollisionMeshesOffline) {
					bodyStream << "  Collision mesh decimation: enabled (offline)\n";
					bodyStream << "  Target vertex ratio: " << g_validationConfig.decimationTargetVertexRatio << "\n";
					bodyStream << "  Target vertex count: " << g_validationConfig.decimationTargetVertexCount << "\n";
					bodyStream << "  Skin weight penalty: " << g_validationConfig.decimationSkinWeightPenalty << "\n";
					bodyStream << "  Max skin weight drift: " << g_validationConfig.decimationMaxSkinWeightDrift << "\n";
					bodyStream << "  Max volume loss (%): " << g_validationConfig.decimationMaxVolumeLossPercent << "\n";
				}

				auto decimationOptions = buildDecimationOptionsFromConfig();
				std::atomic<int> improvedCount{ 0 };
				std::atomic<int> decCandidatesDiscovered{ 0 };
				std::atomic<int> decCandidatesAttempted{ 0 };
				std::atomic<int> decCandidatesApplied{ 0 };
				std::atomic<int> decCandidatesSkippedNoChange{ 0 };
				std::atomic<int> decCandidatesSkippedUnsafe{ 0 };
				std::vector<std::string> improvedPaths;
				std::map<std::string, int> decimationReasonCounts;
				std::mutex improvedLock;

				// Group assets by base stem so _0/_1 pairs are processed atomically:
				// if one member of a pair is improved, the other is also written to the
				// output directory (as a plain copy if it needed no changes) so the game
				// never sees a half-updated pair.
				std::map<std::string, std::vector<size_t>> stemGroups;
				for (size_t i = 0; i < nifAssets.size(); ++i)
					stemGroups[getNifPairBaseStem(NormalizePathForComparison(nifAssets[i].nifPath))].push_back(i);

				std::vector<std::vector<size_t>> groups;
				groups.reserve(stemGroups.size());
				for (auto& [stem, indices] : stemGroups)
					groups.push_back(std::move(indices));

				auto improveGroup = [&](const std::vector<size_t>& indices) {
					std::vector<bool> results;
					results.reserve(indices.size());
					for (size_t idx : indices) {
						NIFImproverDiagnostics d;
						results.push_back(GenerateImprovedNIF(nifAssets[idx].nifPath, g_validationConfig.outputDir, decimationOptions, &d));
						decCandidatesDiscovered.fetch_add(d.decimationCandidatesDiscovered);
						decCandidatesAttempted.fetch_add(d.decimationCandidatesAttempted);
						decCandidatesApplied.fetch_add(d.decimationCandidatesApplied);
						decCandidatesSkippedNoChange.fetch_add(d.decimationCandidatesSkippedNoChange);
						decCandidatesSkippedUnsafe.fetch_add(d.decimationCandidatesSkippedUnsafe);
						if (!d.decimationSkipReasons.empty()) {
							std::lock_guard<std::mutex> l(improvedLock);
							for (const auto& rc : d.decimationSkipReasons)
								decimationReasonCounts[rc.first] += rc.second;
						}
					}

					if (!std::any_of(results.begin(), results.end(), [](bool r) { return r; }))
						return;

					for (size_t k = 0; k < indices.size(); ++k) {
						const std::string& path = nifAssets[indices[k]].nifPath;
						if (results[k]) {
							improvedCount.fetch_add(1);
							std::lock_guard<std::mutex> l(improvedLock);
							improvedPaths.push_back(path);
						} else {
							// Sibling was improved; copy this member unchanged so the
							// pair is complete in the output directory.
							if (!CopyNIFToOutput(path, g_validationConfig.outputDir))
								logger::warn("[Validator] Failed to copy unchanged NIF sibling to output: {}", path);
						}
					}
				};

				if (g_validationConfig.parallelNIFImprovement && groups.size() > 1) {
					tbb::parallel_for_each(groups.begin(), groups.end(), improveGroup);
				} else {
					for (const auto& g : groups)
						improveGroup(g);
				}

				report.nifImprovedCount += improvedCount.load();
				for (const auto& p : improvedPaths) {
					bodyStream << "  [IMPROVED] " << p << "\n";
				}
				bodyStream << "  " << report.nifImprovedCount << " improved NIF file(s) written.\n";

				if (g_validationConfig.decimateCollisionMeshesOffline) {
					bodyStream << "  Decimation bridge candidates discovered: " << decCandidatesDiscovered.load() << "\n";
					bodyStream << "  Decimation bridge candidates attempted: " << decCandidatesAttempted.load() << "\n";
					bodyStream << "  Decimation bridge candidates applied: " << decCandidatesApplied.load() << "\n";
					bodyStream << "  Decimation bridge candidates skipped (no change): " << decCandidatesSkippedNoChange.load() << "\n";
					bodyStream << "  Decimation bridge candidates skipped (unsafe): " << decCandidatesSkippedUnsafe.load() << "\n";
					for (const auto& [reason, count] : decimationReasonCounts)
						bodyStream << "    [SKIP-REASON] " << reason << ": " << count << "\n";
				}
			}
		}

		// Stop timer
		auto wallEnd = std::chrono::steady_clock::now();
		double elapsedSec = std::chrono::duration<double>(wallEnd - wallStart).count();
		report.elapsedSeconds = elapsedSec;

		// Errors and warnings index (appended after body for quick reference)
		std::ostringstream tailStream;
		if (!report.errors.empty()) {
			tailStream << "== Errors ==\n";
			for (const auto& e : report.errors)
				tailStream << "  [ERROR] " << e << "\n";
			tailStream << "\n";
		}
		if (!report.warnings.empty()) {
			tailStream << "== Warnings ==\n";
			for (const auto& w : report.warnings)
				tailStream << "  [WARN] " << w << "\n";
			tailStream << "\n";
		}

		// Assemble final report: header + summary + body + tail
		std::ostringstream reportStream;
		reportStream << "========================================\n";
		if (equippedOnly) {
			reportStream << "FSMP Equipped Gear Validation Report\n";
		} else {
			reportStream << "FSMP Asset Validation Report\n";
		}
		reportStream << "Generated: " << timestamp << "\n";
		reportStream << "========================================\n\n";

		reportStream << "== Summary ==\n";
		reportStream << "  Duration:      " << std::fixed << std::setprecision(2) << elapsedSec << "s\n";
		reportStream << "  XMLs found:    " << report.totalXMLsFound << "\n";
		reportStream << "  XMLs passed:   " << report.xmlPassCount << "\n";
		reportStream << "  XMLs failed:   " << report.xmlErrorCount << "\n";
		reportStream << "  NIF discovery: filesystem=" << report.filesystemNifFilesDiscovered
			<< ", equipped=" << report.equippedNifsDiscovered
			<< ", scan violations=" << report.nifScanViolationCount << "\n";
		reportStream << "  Warnings:      " << report.warnings.size() << "\n";
		reportStream << "  Errors:        " << report.errors.size() << "\n";
		if (!equippedOnly) {
			if (!g_validationConfig.outputDir.empty())
				reportStream << "  XMLs improved: " << report.xmlImprovedCount << "\n";
			if (!g_validationConfig.outputDir.empty() && g_validationConfig.improveNIFs)
				reportStream << "  NIFs improved: " << report.nifImprovedCount << "\n";
		}
		reportStream << "\n";

		reportStream << bodyStream.str();
		reportStream << "\n";
		reportStream << tailStream.str();
		reportStream << "========================================\n";
		return reportStream.str();
	}

	// ---- public entry points ----

	static std::string buildErrorsOnlyReport(
		const AssetValidationResult& report,
		const std::string& timestamp,
		bool equippedOnly)
	{
		std::ostringstream reportStream;
		reportStream << "========================================\n";
		if (equippedOnly) {
			reportStream << "FSMP Equipped Gear Validation Report (Errors Only)\n";
		} else {
			reportStream << "FSMP Asset Validation Report (Errors Only)\n";
		}
		reportStream << "Generated: " << timestamp << "\n";
		reportStream << "========================================\n\n";

		reportStream << "== Summary ==\n";
		reportStream << "  Duration:      " << std::fixed << std::setprecision(2) << report.elapsedSeconds << "s\n";
		reportStream << "  XMLs found:    " << report.totalXMLsFound << "\n";
		reportStream << "  XMLs failed:   " << report.xmlErrorCount << "\n";
		reportStream << "  Errors:        " << report.errors.size() << "\n\n";

		reportStream << "== Errors ==\n";
		if (report.errors.empty()) {
			reportStream << "  [OK] No errors found.\n";
		} else {
			for (const auto& e : report.errors)
				reportStream << "  [ERROR] " << e << "\n";
		}

		reportStream << "\n========================================\n";
		return reportStream.str();
	}

	/// Validates all physics assets (NIFs and XMLs) and writes a detailed report to disk.
	/// Runs either the full pipeline (all NIFs) or equipped-only pipeline (equipped items only).
	AssetValidationResult ValidatePhysicsAssets(
		std::string& outReportPath,
		bool equippedOnly,
		ValidationReportMode reportMode)
	{
		logger::info("[Validator] Starting {} on-demand validation...",
			equippedOnly ? "equipped gear" : "FSMP asset");

		AssetValidationResult report;
		std::string timestamp = BuildTimestampStringForFilenames();
		std::string fullReportContent = runValidationCore(report, timestamp, equippedOnly);

		std::string reportContent;
		if (reportMode == ValidationReportMode::ErrorsOnly) {
			reportContent = buildErrorsOnlyReport(report, timestamp, equippedOnly);
		} else {
			reportContent = std::move(fullReportContent);
		}

		// Always write the file for on-demand runs
		outReportPath = writeValidationReportFile(reportContent, timestamp);

		if (report.hasErrors) {
			logger::warn(
				"[Validator] {} validation in {:.2f}s: {} error(s), {} warning(s).",
				equippedOnly ? "Equipped gear" : "On-demand",
				report.elapsedSeconds, report.errors.size(), report.warnings.size());
		} else if (report.hasWarnings) {
			logger::info(
				"[Validator] {} validation in {:.2f}s: no errors, {} warning(s).",
				equippedOnly ? "Equipped gear" : "On-demand",
				report.elapsedSeconds, report.warnings.size());
		} else {
			if (equippedOnly) {
				logger::info(
					"[Validator] Equipped gear validation in {:.2f}s: all equipped physics assets OK ({} XML file(s)).",
					report.elapsedSeconds, report.totalXMLsFound);
			} else {
				logger::info(
					"[Validator] On-demand validation in {:.2f}s: all physics assets OK ({} XML file(s)).",
					report.elapsedSeconds, report.totalXMLsFound);
			}
		}

		return report;
	}

	/// Generates improved versions of all physics XML files in the specified output directory.
	/// Improves XMLs from all discovered assets (or equipped items only).
	XMLImproveResult ImprovePhysicsXMLs(const std::string& outputDir, bool equippedOnly)
	{
		logger::info("[Validator] Starting {} XML cleanup...",
			equippedOnly ? "equipped gear" : "on-demand FSMP");
		auto result = improveXMLPaths(
			collectPhysicsXMLPaths(equippedOnly),
			outputDir);
		logger::info("[Validator] {} XML cleanup complete: {} XML(s) found, {} improved, {} error(s).",
			equippedOnly ? "Equipped gear" : "On-demand",
			result.totalXMLsFound, result.xmlImprovedCount, result.errors.size());
		return result;
	}

	/// Generates improved versions of all physics NIF files in the specified output directory.
	/// Applies decimation (if configured) and generates clean versions of NIFs.
	/// Preserves _0/_1 NIF pair atomicity by copying unchanged siblings when one is improved.
	NIFImproveResult ImprovePhysicsNIFs(const std::string& outputDir, bool equippedOnly)
	{
		NIFImproveResult result;
		if (outputDir.empty()) {
			result.errors.push_back("Output directory is empty");
			return result;
		}

		if (equippedOnly) {
			std::unordered_set<std::string> equippedXMLs;
			for (const auto& xmlPath : collectPhysicsXMLPaths(true))
				equippedXMLs.insert(NormalizePathForComparison(xmlPath));

			auto nifAssetsEquipped = discoverPhysicsAssets();
			for (const auto& asset : nifAssetsEquipped) {
				if (!asset.xmlExists)
					continue;
				if (equippedXMLs.count(NormalizePathForComparison(asset.xmlPath)) == 0)
					continue;
				++result.totalNIFsFound;
				if (GenerateImprovedNIF(asset.nifPath, outputDir))
					++result.nifImprovedCount;
			}
			return result;
		}

		auto nifAssets = discoverPhysicsAssets();
		result.totalNIFsFound = static_cast<int>(nifAssets.size());
		std::unordered_set<std::string> relatedTRINorm;
		for (const auto& a : nifAssets)
			for (const auto& tri : a.relatedTRIPaths)
				relatedTRINorm.insert(NormalizePathForComparison(tri));
		result.totalTRIFilesFound = static_cast<int>(relatedTRINorm.size());
		auto decimationOptions = buildDecimationOptionsFromConfig();

		// Group by base stem so _0/_1 pairs are processed atomically (same as Phase 5).
		std::map<std::string, std::vector<size_t>> stemGroups;
		for (size_t i = 0; i < nifAssets.size(); ++i)
			stemGroups[getNifPairBaseStem(NormalizePathForComparison(nifAssets[i].nifPath))].push_back(i);

		std::vector<std::vector<size_t>> groups;
		groups.reserve(stemGroups.size());
		for (auto& [stem, indices] : stemGroups)
			groups.push_back(std::move(indices));

		std::atomic<int> improvedCount{ 0 };
		std::atomic<int> decCandidatesDiscovered{ 0 };
		std::atomic<int> decCandidatesAttempted{ 0 };
		std::atomic<int> decCandidatesApplied{ 0 };
		std::atomic<int> decCandidatesSkippedNoChange{ 0 };
		std::atomic<int> decCandidatesSkippedUnsafe{ 0 };
		std::mutex decimationReasonLock;
		std::map<std::string, int> decimationReasonCounts;
		auto improveGroup = [&](const std::vector<size_t>& indices) {
			std::vector<bool> results;
			results.reserve(indices.size());
			for (size_t idx : indices) {
				NIFImproverDiagnostics d;
				results.push_back(GenerateImprovedNIF(nifAssets[idx].nifPath, outputDir, decimationOptions, &d));
				decCandidatesDiscovered.fetch_add(d.decimationCandidatesDiscovered);
				decCandidatesAttempted.fetch_add(d.decimationCandidatesAttempted);
				decCandidatesApplied.fetch_add(d.decimationCandidatesApplied);
				decCandidatesSkippedNoChange.fetch_add(d.decimationCandidatesSkippedNoChange);
				decCandidatesSkippedUnsafe.fetch_add(d.decimationCandidatesSkippedUnsafe);
				if (!d.decimationSkipReasons.empty()) {
					std::lock_guard<std::mutex> l(decimationReasonLock);
					for (const auto& rc : d.decimationSkipReasons)
						decimationReasonCounts[rc.first] += rc.second;
				}
			}

			if (!std::any_of(results.begin(), results.end(), [](bool r) { return r; }))
				return;

			improvedCount.fetch_add(static_cast<int>(std::count(results.begin(), results.end(), true)));
			for (size_t k = 0; k < indices.size(); ++k) {
				if (!results[k]) {
					if (!CopyNIFToOutput(nifAssets[indices[k]].nifPath, outputDir))
						logger::warn("[Validator] Failed to copy unchanged NIF sibling to output: {}", nifAssets[indices[k]].nifPath);
				}
			}
		};

		if (g_validationConfig.parallelNIFImprovement && groups.size() > 1)
			tbb::parallel_for_each(groups.begin(), groups.end(), improveGroup);
		else
			for (const auto& g : groups)
				improveGroup(g);

		result.nifImprovedCount = improvedCount.load();
		result.decimationCandidatesDiscovered = decCandidatesDiscovered.load();
		result.decimationCandidatesAttempted = decCandidatesAttempted.load();
		result.decimationCandidatesApplied = decCandidatesApplied.load();
		result.decimationCandidatesSkippedNoChange = decCandidatesSkippedNoChange.load();
		result.decimationCandidatesSkippedUnsafe = decCandidatesSkippedUnsafe.load();
		for (const auto& [reason, count] : decimationReasonCounts)
			result.decimationSkipReasonHistogram.push_back(reason + "=" + std::to_string(count));

		return result;
	}

}  // namespace hdt
