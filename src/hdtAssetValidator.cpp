#include "hdtAssetValidator.h"

#include "NetImmerseUtils.h"
#include "hdtNIFValidator.h"
#include "hdtSCHValidator.h"
#include "hdtXSDValidator.h"
#include "hdtXMLImprover.h"

#include <pugixml.hpp>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdt
{
	ValidationConfig g_validationConfig;

	// ---- helpers ----

	static std::string timestampString()
	{
		auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		std::tm tmBuf{};
		localtime_s(&tmBuf, &t);
		std::ostringstream ss;
		ss << std::put_time(&tmBuf, "%Y%m%d_%H%M%S");
		return ss.str();
	}

	// Normalise a path to lowercase and forward slashes for comparisons
	static std::string normalisePath(std::string p)
	{
		std::transform(p.begin(), p.end(), p.begin(), [](unsigned char c) {
			return c == '\\' ? '/' : (char)std::tolower(c);
		});
		return p;
	}

	// XML file stems to skip – not physics config files
	static const std::unordered_set<std::string> kSkippedXMLStems = {
		"configs",      // main configuration file
		"defaultbbps",  // shape-to-XML mapping (not a physics config)
		"hdtsmp64",     // hdtSMP64.xsd physics schema (FSMP-Validator)
	};

	// ---- Parallel XML validation helpers ----

	using XMLValidationPair = std::pair<XSDValidationResult, SCHValidationResult>;

	// Validate a list of XML file paths in parallel, returning results in the same order.
	// Both ValidatePhysicsXML and ValidatePhysicsXMLWithSCH are thread-safe after
	// their one-time schema loading (guarded by std::once_flag internally).
	static std::vector<XMLValidationPair> parallelValidateXMLs(const std::vector<std::string>& paths)
	{
		std::vector<XMLValidationPair> results(paths.size());
		if (paths.empty())
			return results;

		const unsigned nThreads = std::max(1u, std::thread::hardware_concurrency());
		const size_t n = paths.size();
		const size_t chunkSize = (n + nThreads - 1) / nThreads;

		std::vector<std::future<void>> futures;
		futures.reserve(nThreads);

		for (size_t i = 0; i < n; i += chunkSize) {
			size_t end = std::min(i + chunkSize, n);
			futures.push_back(std::async(std::launch::async, [&paths, &results, i, end]() {
				for (size_t j = i; j < end; ++j) {
					results[j] = { ValidatePhysicsXML(paths[j]), ValidatePhysicsXMLWithSCH(paths[j]) };
				}
			}));
		}

		for (auto& f : futures)
			f.get();

		return results;
	}

	// ---- Phase 0: DefaultBBP XML discovery ----

	struct DefaultBBPEntry {
		std::string shape;   // shape name from <map shape="...">
		std::string xmlPath; // resolved filesystem path (data/...)
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
				bbpFile.string());
			return result;
		}

		pugi::xml_document doc;
		auto parseResult = doc.load_file(bbpFile.string().c_str());
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
				entry.xmlPath = candidate.string();
				entry.xmlExists = true;
			} else {
				// Fall back to path as-is (in case it's already absolute or differently rooted)
				candidate = rawFile;
				entry.xmlPath = candidate.string();
				entry.xmlExists = fs::exists(candidate, ec);
			}

			result.push_back(std::move(entry));
		}

		return result;
	}

	// ---- Phase 1: XML discovery ----

	// Scan the hdtSkinnedMeshConfigs directory for all XML files.
	static std::vector<std::string> discoverXMLFiles()
	{
		std::vector<std::string> result;

		namespace fs = std::filesystem;
		fs::path configDir = "data/skse/plugins/hdtSkinnedMeshConfigs";

		std::error_code ec;
		if (!fs::exists(configDir, ec) || !fs::is_directory(configDir, ec)) {
			logger::warn("[Validator] Config directory not found: {}", configDir.string());
			return result;
		}

		for (auto& entry : fs::recursive_directory_iterator(configDir, ec)) {
			if (ec) {
				logger::warn("[Validator] Directory iteration error: {}", ec.message());
				break;
			}
			if (entry.is_regular_file(ec) && !ec) {
				auto& p = entry.path();
				auto ext = p.extension().string();
				// Lowercase extension comparison
				std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
				if (ext == ".xml") {
					// Skip the configsPresets subfolder — those are preset files, not physics configs.
					bool inPresetsDir = false;
					for (const auto& component : p) {
						auto c = component.string();
						std::transform(c.begin(), c.end(), c.begin(), ::tolower);
						if (c == "configspresets") {
							inPresetsDir = true;
							break;
						}
					}
					if (inPresetsDir)
						continue;

					// Skip non-physics-config files using the static exclusion list
					auto stem = p.stem().string();
					std::transform(stem.begin(), stem.end(), stem.begin(), ::tolower);
					if (!kSkippedXMLStems.count(stem)) {
						result.push_back(p.string());
					}
				}
			}
		}

		return result;
	}

	// ---- Phase 2: NIF discovery ----

	// Scan the game data directory for NIF files that contain physics data.
	// Two-step: serial directory walk to collect paths, then parallel binary scan.
	static std::vector<PhysicsAsset> discoverPhysicsNIFs()
	{
		namespace fs = std::filesystem;
		fs::path meshDir = "data/meshes";

		std::error_code ec;
		if (!fs::exists(meshDir, ec) || !fs::is_directory(meshDir, ec)) {
			logger::warn("[Validator] Meshes directory not found: {}", meshDir.string());
			return {};
		}

		auto toStr = [](const fs::path& fp) -> std::string {
			auto u8 = fp.generic_u8string();
			return { reinterpret_cast<const char*>(u8.data()), u8.size() };
		};

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

		// Step 2: parallel binary scan — each thread scans an independent chunk.
		// ScanNIFBinary opens its own file handle and uses only local state → thread-safe.
		const unsigned nThreads = std::max(1u, std::thread::hardware_concurrency());
		const size_t n = nifPaths.size();
		const size_t chunkSize = (n + nThreads - 1) / nThreads;

		std::vector<std::optional<PhysicsAsset>> scanResults(n);
		std::vector<std::future<void>> futures;
		futures.reserve(nThreads);

		for (size_t i = 0; i < n; i += chunkSize) {
			size_t end = std::min(i + chunkSize, n);
			futures.push_back(std::async(std::launch::async,
				[&nifPaths, &scanResults, i, end, &toStr]() {
					for (size_t j = i; j < end; ++j) {
						const auto& pathStr = nifPaths[j];
						try {
							auto scanRes = ScanNIFBinary(pathStr);
							if (!scanRes.hasPhysicsData)
								continue;

							PhysicsAsset asset;
							asset.nifPath = pathStr;
							asset.nifExists = true;
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
							logger::warn("[Validator] Error scanning NIF {}: {}", pathStr, e.what());
						} catch (...) {
							logger::warn("[Validator] Unknown error scanning NIF {}", pathStr);
						}
					}
				}));
		}

		for (auto& f : futures)
			f.get();

		// Collect physics-enabled NIFs, preserving discovery order.
		std::vector<PhysicsAsset> result;
		result.reserve(n);
		for (auto& opt : scanResults) {
			if (opt)
				result.push_back(std::move(*opt));
		}

		logger::info("[Validator] Scanned {} NIF files, found {} physics-enabled NIFs",
			nifPaths.size(), result.size());
		return result;
	}

	// ---- Phase 0: DefaultBBP XML validation ----

	// Validate each XML referenced in defaultBBPs.xml.
	// Files already present in validatedXMLs are skipped (cross-phase dedup).
	static void validateDefaultBBPXMLFiles(const std::vector<DefaultBBPEntry>& entries,
		AssetValidationResult& report, std::ostream& out,
		std::unordered_set<std::string>& validatedXMLs)
	{
		// Pre-filter: determine which entries need validation (serial, cheap).
		// validBatchIdx[i] = index into batch, or SIZE_MAX if skipped/missing.
		std::vector<size_t> validBatchIdx(entries.size(), SIZE_MAX);
		std::vector<std::string> batch;
		for (size_t i = 0; i < entries.size(); ++i) {
			if (!entries[i].xmlExists)
				continue;
			auto norm = normalisePath(entries[i].xmlPath);
			if (validatedXMLs.count(norm))
				continue;
			validatedXMLs.insert(norm);
			validBatchIdx[i] = batch.size();
			batch.push_back(entries[i].xmlPath);
		}

		// Parallel validate the batch.
		auto batchResults = parallelValidateXMLs(batch);

		// Report results in original order (serial).
		for (size_t i = 0; i < entries.size(); ++i) {
			const auto& entry = entries[i];
			out << "  [BBP]  shape=" << entry.shape << " -> " << entry.xmlPath << "\n";

			if (!entry.xmlExists) {
				std::string err = "defaultBBPs.xml: shape '" + entry.shape +
					"' references missing XML: " + entry.xmlPath;
				report.errors.push_back(err);
				report.hasErrors = true;
				out << "    [ERROR] XML file not found\n";
				continue;
			}

			if (validBatchIdx[i] == SIZE_MAX) {
				out << "    (already validated)\n";
				continue;
			}

			++report.totalXMLsFound;
			const auto& [xsdResult, schResult] = batchResults[validBatchIdx[i]];
			bool fileHasErrors = !xsdResult.isValid || schResult.hasErrors;

			if (fileHasErrors) {
				++report.xmlErrorCount;
				report.hasErrors = true;
				out << "    [FAIL]\n";
			} else {
				++report.xmlPassCount;
				out << "    [OK]\n";
			}

			for (const auto& v : xsdResult.violations) {
				std::string msg = entry.xmlPath + ":" + std::to_string(v.line) + ": " +
				                  v.elementPath + " - " + v.message;
				report.errors.push_back(msg);
				out << "    [ERROR] " << v.elementPath << " (line " << v.line << "): "
					<< v.message << "\n";
			}

			for (const auto& v : schResult.violations) {
				std::string msg = entry.xmlPath + ":" + std::to_string(v.line) + ": " +
				                  v.location + " - " + v.message;
				if (v.role == "error") {
					report.errors.push_back(msg);
					out << "    [SCH-ERROR] " << v.location << " (line " << v.line << "): "
						<< v.message << "\n";
				} else {
					report.warnings.push_back(msg);
					report.hasWarnings = true;
					out << "    [WARNING] " << v.location << " (line " << v.line << "): "
						<< v.message << "\n";
				}
			}
		}
	}

	// ---- Phase 1: XML validation ----

	// Validate all discovered XML files and build the report.
	// Files already present in validatedXMLs are skipped (cross-phase dedup).
	static void validateXMLFiles(const std::vector<std::string>& xmlPaths,
		AssetValidationResult& report, std::ostream& out,
		std::unordered_set<std::string>& validatedXMLs)
	{
		// Pre-filter: determine which files need validation (serial, cheap).
		// validBatchIdx[i] = index into batch, or SIZE_MAX if already done.
		std::vector<size_t> validBatchIdx(xmlPaths.size(), SIZE_MAX);
		std::vector<std::string> batch;
		for (size_t i = 0; i < xmlPaths.size(); ++i) {
			auto norm = normalisePath(xmlPaths[i]);
			if (validatedXMLs.count(norm))
				continue;
			validatedXMLs.insert(norm);
			validBatchIdx[i] = batch.size();
			batch.push_back(xmlPaths[i]);
		}

		// Parallel validate the batch.
		auto batchResults = parallelValidateXMLs(batch);

		// Report results in original order (serial).
		for (size_t i = 0; i < xmlPaths.size(); ++i) {
			const auto& xmlPath = xmlPaths[i];

			if (validBatchIdx[i] == SIZE_MAX) {
				out << "  [SKIP] " << xmlPath << " (already validated)\n";
				continue;
			}

			const auto& [xsdResult, schResult] = batchResults[validBatchIdx[i]];
			++report.totalXMLsFound;

			bool fileHasErrors = !xsdResult.isValid || schResult.hasErrors;

			if (fileHasErrors) {
				++report.xmlErrorCount;
				report.hasErrors = true;
				out << "  [FAIL] " << xmlPath << "\n";
			} else {
				++report.xmlPassCount;
				out << "  [OK]   " << xmlPath << "\n";
			}

			// XSD violations
			for (const auto& v : xsdResult.violations) {
				std::string msg = xmlPath + ":" + std::to_string(v.line) + ": " +
				                  v.elementPath + " - " + v.message;
				report.errors.push_back(msg);
				out << "    [ERROR] " << v.elementPath << " (line " << v.line << "): "
					<< v.message << "\n";
			}

			// SCH violations
			for (const auto& v : schResult.violations) {
				std::string msg = xmlPath + ":" + std::to_string(v.line) + ": " + v.location + " - " + v.message;
				if (v.role == "error") {
					report.errors.push_back(msg);
					out << "    [SCH-ERROR] " << v.location << " (line " << v.line << "): " << v.message << "\n";
				} else {
					report.warnings.push_back(msg);
					report.hasWarnings = true;
					out << "    [WARNING] " << v.location << " (line " << v.line << "): " << v.message << "\n";
				}
			}
		}
	}

	// ---- Phase 2.5: NIF _0/_1 pair consistency check ----

	// For every NIF whose filename ends in _0.nif, check that the matching _1.nif:
	//   1. exists in the same directory
	//   2. references the same physics XML (same normalised path, or both missing)
	//   3. references it at the same block position (same index in allPhysicsXmlPaths)
	static void checkNIFPairs(const std::vector<PhysicsAsset>& nifAssets,
		AssetValidationResult& report, std::ostream& out)
	{
		// Build a fast lookup: normalised nif path -> index in nifAssets
		std::unordered_map<std::string, size_t> nifByNormPath;
		nifByNormPath.reserve(nifAssets.size());
		for (size_t i = 0; i < nifAssets.size(); ++i)
			nifByNormPath[normalisePath(nifAssets[i].nifPath)] = i;

		// Track _0.nif paths we've already checked (avoid reporting the same pair twice)
		std::unordered_set<std::string> checked;

		for (size_t i = 0; i < nifAssets.size(); ++i) {
			const auto& asset = nifAssets[i];

			// We only initiate checks from the _0.nif side
			auto normPath = normalisePath(asset.nifPath);
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
			auto normXml0 = normalisePath(asset.xmlPath);
			auto normXml1 = normalisePath(asset1.xmlPath);
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
					if (normalisePath(paths0[k]) != normalisePath(paths1[k])) {
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
			auto norm = normalisePath(asset.xmlPath);
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

				auto norm = normalisePath(asset.xmlPath);
				if (reportedXMLs.count(norm)) {
					out << "    (already validated)\n";
					continue;
				}
				reportedXMLs.insert(norm);
				++report.totalXMLsFound;

				const auto& [xsdResult, schResult] = batchResults[xmlToIdx[norm]];
				bool xmlHasErrors = !xsdResult.isValid || schResult.hasErrors;

				if (xmlHasErrors) {
					++report.xmlErrorCount;
					report.hasErrors = true;
					for (const auto& v : xsdResult.violations) {
						std::string msg = asset.xmlPath + ":" + std::to_string(v.line) +
						                  ": " + v.elementPath + " - " + v.message;
						report.errors.push_back(msg);
						out << "    [ERROR] " << v.elementPath << " (line " << v.line
							<< "): " << v.message << "\n";
					}
					for (const auto& v : schResult.violations) {
						std::string msg = asset.xmlPath + ":" + std::to_string(v.line) + ": " + v.location + " - " + v.message;
						if (v.role == "error") {
							report.errors.push_back(msg);
							out << "    [SCH-ERROR] " << v.location << " (line " << v.line << "): " << v.message << "\n";
						} else {
							report.warnings.push_back(msg);
							report.hasWarnings = true;
							out << "    [WARNING] " << v.location << " (line " << v.line << "): " << v.message << "\n";
						}
					}
				} else {
					++report.xmlPassCount;

					// SCH warnings on a passing file
					for (const auto& v : schResult.violations) {
						std::string msg = asset.xmlPath + ":" + std::to_string(v.line) + ": " + v.location + " - " + v.message;
						report.warnings.push_back(msg);
						report.hasWarnings = true;
						out << "    [WARNING] " << v.location << " (line " << v.line << "): " << v.message << "\n";
					}
				}
			} else {
				out << "    [WARN] Could not determine XML path from NIF\n";
			}
		}
	}

	// ---- report writing ----

	// Write report content to a timestamped file in the log directory.
	// Returns the full path to the written file, or empty string on failure.
	static std::string writeReport(const std::string& reportContent,
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
			logger::warn("[Validator] Could not open report file: {}", reportPath.string());
			return {};
		}

		out << reportContent;
		logger::info("[Validator] Validation report written to: {}", reportPath.string());
		return reportPath.string();
	}

	// ---- core validation ----

	// Runs all validation phases; populates report.
	static std::string runValidationCore(AssetValidationResult& report, const std::string& timestamp)
	{
		auto wallStart = std::chrono::steady_clock::now();

		std::ostringstream bodyStream;

		// Shared dedup set: tracks normalised paths already validated across phases 0 and 1
		// (prevents double-counting and redundant XSD/SCH runs for files in both DefaultBBP and the config dir)
		std::unordered_set<std::string> globalValidatedXMLs;

		// Phase 0: DefaultBBP XML validation
		auto bbpEntries = discoverDefaultBBPXMLs();
		if (!bbpEntries.empty()) {
			bodyStream << "== Phase 0: DefaultBBP XML Validation ==\n";
			bodyStream << "  Found " << bbpEntries.size() << " map entries in defaultBBPs.xml.\n";
			validateDefaultBBPXMLFiles(bbpEntries, report, bodyStream, globalValidatedXMLs);
			bodyStream << "\n";
		}

		// Phase 1: direct XML validation
		bodyStream << "== Phase 1: XML Configuration Validation ==\n";
		auto xmlFiles = discoverXMLFiles();
		logger::info("[Validator] Found {} physics XML files in config directory",
			xmlFiles.size());

		validateXMLFiles(xmlFiles, report, bodyStream, globalValidatedXMLs);

		// Phase 2: NIF discovery
		bodyStream << "\n== Phase 2: NIF File Discovery ==\n";
		auto nifAssets = discoverPhysicsNIFs();
		report.totalNIFsScanned = (int)nifAssets.size();
		bodyStream << "  Found " << nifAssets.size() << " NIF file(s) referencing physics configs.\n";

		// Phase 2.5: NIF _0/_1 pair consistency check
		bodyStream << "\n== Phase 2.5: NIF Pair Consistency Check ==\n";
		checkNIFPairs(nifAssets, report, bodyStream);

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
				if (path.empty()) return;
				auto norm = normalisePath(path);
				if (improveNorm.insert(norm).second)
					improveQueue.push_back(path);
			};

			for (const auto& e : bbpEntries)
				if (e.xmlExists) enqueue(e.xmlPath);
			for (const auto& p : xmlFiles)
				enqueue(p);
			for (const auto& a : nifAssets)
				if (a.xmlExists) enqueue(a.xmlPath);

			for (const auto& xmlPath : improveQueue) {
				if (GenerateImprovedXML(xmlPath, g_validationConfig.outputDir)) {
					++report.xmlImprovedCount;
					bodyStream << "  [IMPROVED] " << xmlPath << "\n";
				}
			}

			bodyStream << "  " << report.xmlImprovedCount << " improved file(s) written.\n";
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
		reportStream << "FSMP Asset Validation Report\n";
		reportStream << "Generated: " << timestamp << "\n";
		reportStream << "========================================\n\n";

		reportStream << "== Summary ==\n";
		reportStream << "  Duration:      " << std::fixed << std::setprecision(2) << elapsedSec << "s\n";
		reportStream << "  XMLs found:    " << report.totalXMLsFound << "\n";
		reportStream << "  XMLs passed:   " << report.xmlPassCount << "\n";
		reportStream << "  XMLs failed:   " << report.xmlErrorCount << "\n";
		reportStream << "  NIFs scanned:  " << report.totalNIFsScanned << "\n";
		reportStream << "  Warnings:      " << report.warnings.size() << "\n";
		reportStream << "  Errors:        " << report.errors.size() << "\n";
		if (!g_validationConfig.outputDir.empty())
			reportStream << "  XMLs improved: " << report.xmlImprovedCount << "\n";
		reportStream << "\n";

		reportStream << bodyStream.str();
		reportStream << "\n";
		reportStream << tailStream.str();
		reportStream << "========================================\n";
		return reportStream.str();
	}

	// ---- public entry points ----

	bool ValidateAllPhysicsAssets()
	{
		if (!g_validationConfig.enabled) {
			logger::info("[Validator] Asset validation disabled by config");
			return true;
		}

		logger::info("[Validator] Starting FSMP asset validation...");

		AssetValidationResult report;
		std::string timestamp = timestampString();
		std::string reportContent = runValidationCore(report, timestamp);

		if (g_validationConfig.reportFileEnabled) {
			writeReport(reportContent, timestamp);
		}

		if (report.hasErrors) {
			logger::warn(
				"[Validator] Validation complete in {:.2f}s: {} error(s), {} warning(s).",
				report.elapsedSeconds, report.errors.size(), report.warnings.size());
		} else if (report.hasWarnings) {
			logger::info(
				"[Validator] Validation complete in {:.2f}s: no errors, {} warning(s).",
				report.elapsedSeconds, report.warnings.size());
		} else {
			logger::info("[Validator] Validation complete in {:.2f}s: all physics assets OK ({} XML file(s)).",
				report.elapsedSeconds, report.totalXMLsFound);
		}

		if (g_validationConfig.strictMode && report.hasErrors) {
			return false;
		}

		return !report.hasErrors;
	}

	AssetValidationResult ValidateAllPhysicsAssetsOnDemand(std::string& outReportPath)
	{
		logger::info("[Validator] Starting on-demand FSMP asset validation...");

		AssetValidationResult report;
		std::string timestamp = timestampString();
		std::string reportContent = runValidationCore(report, timestamp);

		// Always write the file for on-demand runs
		outReportPath = writeReport(reportContent, timestamp);

		if (report.hasErrors) {
			logger::warn(
				"[Validator] On-demand validation in {:.2f}s: {} error(s), {} warning(s).",
				report.elapsedSeconds, report.errors.size(), report.warnings.size());
		} else if (report.hasWarnings) {
			logger::info(
				"[Validator] On-demand validation in {:.2f}s: no errors, {} warning(s).",
				report.elapsedSeconds, report.warnings.size());
		} else {
			logger::info(
				"[Validator] On-demand validation in {:.2f}s: all physics assets OK ({} XML file(s)).",
				report.elapsedSeconds, report.totalXMLsFound);
		}

		return report;
	}

}  // namespace hdt
