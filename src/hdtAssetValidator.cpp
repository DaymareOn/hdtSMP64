#include "hdtAssetValidator.h"

#include "NetImmerseUtils.h"
#include "hdtNIFValidator.h"
#include "hdtSCHValidator.h"
#include "hdtXSDValidator.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
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

	// ---- Phase 1b: NIF discovery (optional) ----

	// Scan the game data directory for NIF files that contain physics data.
	static std::vector<PhysicsAsset> discoverPhysicsNIFs()
	{
		std::vector<PhysicsAsset> result;

		namespace fs = std::filesystem;
		fs::path meshDir = "data/meshes";

		std::error_code ec;
		if (!fs::exists(meshDir, ec) || !fs::is_directory(meshDir, ec)) {
			logger::warn("[Validator] Meshes directory not found: {}", meshDir.string());
			return result;
		}

		int scanned = 0;
		for (auto& entry : fs::recursive_directory_iterator(meshDir, ec)) {
			if (ec) {
				logger::warn("[Validator] Mesh directory iteration error: {}", ec.message());
				break;
			}
			if (!entry.is_regular_file(ec) || ec) {
				continue;
			}

			auto& p = entry.path();
			auto ext = p.extension().string();
			std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
			if (ext != ".nif") {
				continue;
			}

			++scanned;
			auto scanRes = ScanNIFBinary(p.string());
			if (scanRes.hasPhysicsData) {
				PhysicsAsset asset;
				asset.nifPath = p.string();
				asset.nifExists = true;

				if (!scanRes.physicsXmlPath.empty()) {
					// Normalise path separator and make relative
					std::string xmlPath = scanRes.physicsXmlPath;
					std::replace(xmlPath.begin(), xmlPath.end(), '\\', '/');

					// FSMP XML paths in NIFs are typically relative to the game root
					// e.g. "SKSE/Plugins/hdtSkinnedMeshConfigs/foo.xml"
					// Check both as-is and under data/
					auto normXml = normalisePath(xmlPath);

					fs::path xmlFsPath = xmlPath;
					if (!fs::exists(xmlFsPath, ec)) {
						// Try with data/ prefix
						xmlFsPath = "data/" + xmlPath;
					}
					asset.xmlPath = xmlFsPath.string();
					asset.xmlExists = fs::exists(xmlFsPath, ec);
				}

				result.push_back(std::move(asset));
			}
		}

		logger::info("[Validator] Scanned {} NIF files, found {} physics-enabled NIFs",
			scanned, result.size());
		return result;
	}

	// ---- Phase 2 + 3: XML validation ----

	// Validate all discovered XML files and build the report.
	static void validateXMLFiles(const std::vector<std::string>& xmlPaths,
		AssetValidationResult& report, std::ostream& out)
	{
		for (const auto& xmlPath : xmlPaths) {
			auto xsdResult = ValidatePhysicsXML(xmlPath);
			auto schResult = ValidatePhysicsXMLWithSCH(xmlPath);

			PhysicsAsset asset;
			asset.xmlPath = xmlPath;
			asset.xmlExists = true;
			report.assets.push_back(asset);
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
				std::string msg = xmlPath + ": " + v.location + " - " + v.message;
				if (v.role == "error") {
					report.errors.push_back(msg);
					out << "    [SCH-ERROR] " << v.location << ": " << v.message << "\n";
				} else {
					report.warnings.push_back(msg);
					report.hasWarnings = true;
					out << "    [SCH-WARN] " << v.location << ": " << v.message << "\n";
				}
			}
		}
	}

	// ---- Phase 3b: NIF-based validation ----

	static void validateNIFAssets(const std::vector<PhysicsAsset>& nifAssets,
		AssetValidationResult& report, std::ostream& out)
	{
		for (const auto& asset : nifAssets) {
			out << "  [NIF]  " << asset.nifPath << "\n";

			if (!asset.xmlExists && !asset.xmlPath.empty()) {
				std::string err = "NIF " + asset.nifPath +
				                  " references missing XML: " + asset.xmlPath;
				report.errors.push_back(err);
				report.hasErrors = true;
				out << "    [ERROR] Referenced XML not found: " << asset.xmlPath << "\n";
			} else if (!asset.xmlPath.empty()) {
				out << "    -> " << asset.xmlPath << "\n";

				// Validate the referenced XML
				auto xsdResult = ValidatePhysicsXML(asset.xmlPath);
				auto schResult = ValidatePhysicsXMLWithSCH(asset.xmlPath);

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
						std::string msg = asset.xmlPath + ": " + v.location + " - " + v.message;
						if (v.role == "error") {
							report.errors.push_back(msg);
							out << "    [SCH-ERROR] " << v.location << ": " << v.message << "\n";
						} else {
							report.warnings.push_back(msg);
							report.hasWarnings = true;
							out << "    [SCH-WARN] " << v.location << ": " << v.message << "\n";
						}
					}
				} else {
					++report.xmlPassCount;
					++report.totalXMLsFound;

					// SCH warnings on a passing file
					for (const auto& v : schResult.violations) {
						std::string msg = asset.xmlPath + ": " + v.location + " - " + v.message;
						report.warnings.push_back(msg);
						report.hasWarnings = true;
						out << "    [SCH-WARN] " << v.location << ": " << v.message << "\n";
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
	// forceNIFScan: if true, always scans data/meshes/ for NIF-referenced XMLs (safe when
	// called in-game since BSAs are mounted; unsafe at plugin-load time).
	static std::string runValidationCore(AssetValidationResult& report, const std::string& timestamp,
		bool forceNIFScan = false)
	{
		std::ostringstream reportStream;

		reportStream << "========================================\n";
		reportStream << "FSMP Asset Validation Report\n";
		reportStream << "Generated: " << timestamp << "\n";
		reportStream << "========================================\n\n";

		// Phase 1: XML discovery
		reportStream << "== Phase 1-3: XML Configuration Validation ==\n";
		auto xmlFiles = discoverXMLFiles();
		logger::info("[Validator] Found {} physics XML files in config directory",
			xmlFiles.size());

		validateXMLFiles(xmlFiles, report, reportStream);

		// Phase 1b + 3b: NIF discovery and validation (optional at startup, always on for on-demand)
		if (g_validationConfig.scanDataFolder || forceNIFScan) {
			reportStream << "\n== Phase 1b: NIF File Discovery ==\n";
			auto nifAssets = discoverPhysicsNIFs();
			report.totalNIFsScanned = (int)nifAssets.size();

			if (!nifAssets.empty()) {
				reportStream << "\n== Phase 3b: NIF-Referenced XML Validation ==\n";
				validateNIFAssets(nifAssets, report, reportStream);
			}
		}

		// Summary
		reportStream << "\n== Summary ==\n";
		reportStream << "  XMLs found:    " << report.totalXMLsFound << "\n";
		reportStream << "  XMLs passed:   " << report.xmlPassCount << "\n";
		reportStream << "  XMLs failed:   " << report.xmlErrorCount << "\n";
		if (g_validationConfig.scanDataFolder || forceNIFScan) {
			reportStream << "  NIFs scanned:  " << report.totalNIFsScanned << "\n";
		}
		reportStream << "  Warnings:      " << report.warnings.size() << "\n";
		reportStream << "  Errors:        " << report.errors.size() << "\n";
		reportStream << "\n";

		if (!report.errors.empty()) {
			reportStream << "== Errors ==\n";
			for (const auto& e : report.errors) {
				reportStream << "  [ERROR] " << e << "\n";
			}
			reportStream << "\n";
		}

		if (!report.warnings.empty()) {
			reportStream << "== Warnings ==\n";
			for (const auto& w : report.warnings) {
				reportStream << "  [WARN] " << w << "\n";
			}
			reportStream << "\n";
		}

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
				"[Validator] Validation complete: {} error(s), {} warning(s).",
				report.errors.size(), report.warnings.size());
		} else if (report.hasWarnings) {
			logger::info(
				"[Validator] Validation complete: no errors, {} warning(s).", report.warnings.size());
		} else {
			logger::info("[Validator] Validation complete: all physics assets OK ({} XML file(s)).",
				report.totalXMLsFound);
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

		// Always run NIF scan for on-demand calls: invoked from the in-game console, so all
		// BSAs are mounted and NIF scanning is safe. This finds XMLs in data/meshes/ and
		// other locations outside hdtSkinnedMeshConfigs/.
		std::string reportContent = runValidationCore(report, timestamp, /*forceNIFScan=*/true);

		// Always write the file for on-demand runs
		outReportPath = writeReport(reportContent, timestamp);

		if (report.hasErrors) {
			logger::warn(
				"[Validator] On-demand validation: {} error(s), {} warning(s).",
				report.errors.size(), report.warnings.size());
		} else if (report.hasWarnings) {
			logger::info(
				"[Validator] On-demand validation: no errors, {} warning(s).", report.warnings.size());
		} else {
			logger::info(
				"[Validator] On-demand validation: all physics assets OK ({} XML file(s)).",
				report.totalXMLsFound);
		}

		return report;
	}

}  // namespace hdt
