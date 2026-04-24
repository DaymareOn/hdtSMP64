#include "hdtAssetValidator.h"

#include "hdtNIFValidator.h"
#include "hdtXSDValidator.h"
#include "NetImmerseUtils.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
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
					// Skip the XSD file itself and configs.xml (not physics configs)
					auto stem = p.stem().string();
					std::transform(stem.begin(), stem.end(), stem.begin(), ::tolower);
					if (stem != "configs" && stem != "defaultbbps" &&
						stem != "hdtsmp64" && stem != "configs.xsd") {
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

			PhysicsAsset asset;
			asset.xmlPath = xmlPath;
			asset.xmlExists = true;  // We found it on disk
			report.assets.push_back(asset);
			++report.totalXMLsFound;

			if (xsdResult.isValid) {
				++report.xmlPassCount;
				out << "  [OK]   " << xmlPath << "\n";

				// Phase 5: content warnings
				if (!xsdResult.hasWeightThreshold) {
					std::string warn = xmlPath + ": no <weight-threshold> defined (may impact performance)";
					report.warnings.push_back(warn);
					report.hasWarnings = true;
					out << "    [WARN] No weight-threshold definitions\n";
				}
			} else {
				++report.xmlErrorCount;
				report.hasErrors = true;

				out << "  [FAIL] " << xmlPath << "\n";
				for (const auto& v : xsdResult.violations) {
					std::string msg = xmlPath + ":" + std::to_string(v.line) + ": " +
						v.elementPath + " - " + v.message;
					report.errors.push_back(msg);
					out << "    [ERROR] " << v.elementPath << " (line " << v.line << "): "
						<< v.message << "\n";
				}
			}

			// Phase 5: warn on missing weight thresholds even for valid files
			if (!xsdResult.hasWeightThreshold && xsdResult.isValid) {
				out << "    [WARN] No <weight-threshold> elements found\n";
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
				if (!xsdResult.isValid) {
					++report.xmlErrorCount;
					report.hasErrors = true;
					for (const auto& v : xsdResult.violations) {
						std::string msg = asset.xmlPath + ":" + std::to_string(v.line) +
							": " + v.elementPath + " - " + v.message;
						report.errors.push_back(msg);
						out << "    [ERROR] " << v.elementPath << " (line " << v.line
							<< "): " << v.message << "\n";
					}
				} else {
					++report.xmlPassCount;
					++report.totalXMLsFound;
					if (!xsdResult.hasWeightThreshold) {
						std::string warn = asset.xmlPath + ": no <weight-threshold> defined";
						report.warnings.push_back(warn);
						report.hasWarnings = true;
						out << "    [WARN] No <weight-threshold> definitions\n";
					}
				}
			} else {
				out << "    [WARN] Could not determine XML path from NIF\n";
			}
		}
	}

	// ---- report writing ----

	static void writeReport(const AssetValidationResult& report, const std::string& reportContent,
		const std::string& timestamp)
	{
		if (!g_validationConfig.reportFileEnabled) {
			return;
		}

		auto logDir = logger::log_directory();
		if (!logDir) {
			logger::warn("[Validator] Could not determine log directory for report");
			return;
		}

		auto reportPath = *logDir / ("hdtSMP64_validation_" + timestamp + ".log");

		std::ofstream out(reportPath, std::ios::out | std::ios::trunc);
		if (!out.is_open()) {
			logger::warn("[Validator] Could not open report file: {}", reportPath.string());
			return;
		}

		out << reportContent;
		logger::info("[Validator] Validation report written to: {}", reportPath.string());
	}

	// ---- public entry point ----

	bool ValidateAllPhysicsAssets()
	{
		if (!g_validationConfig.enabled) {
			logger::info("[Validator] Asset validation disabled by config");
			return true;
		}

		logger::info("[Validator] Starting FSMP asset validation...");

		std::ostringstream reportStream;
		std::string timestamp = timestampString();

		reportStream << "========================================\n";
		reportStream << "FSMP Asset Validation Report\n";
		reportStream << "Generated: " << timestamp << "\n";
		reportStream << "========================================\n\n";

		AssetValidationResult report;

		// Phase 1: XML discovery
		reportStream << "== Phase 1-3: XML Configuration Validation ==\n";
		auto xmlFiles = discoverXMLFiles();
		logger::info("[Validator] Found {} physics XML files in config directory",
			xmlFiles.size());

		validateXMLFiles(xmlFiles, report, reportStream);

		// Phase 1b + 3b: NIF discovery and validation (optional)
		if (g_validationConfig.scanDataFolder) {
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
		if (g_validationConfig.scanDataFolder) {
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

		// Write report to file
		std::string reportContent = reportStream.str();
		writeReport(report, reportContent, timestamp);

		// Log summary to main log
		if (report.hasErrors) {
			logger::warn("[Validator] Validation complete: {} error(s), {} warning(s). "
						 "See hdtSMP64_validation_{}.log for details.",
				report.errors.size(), report.warnings.size(), timestamp);
		} else if (report.hasWarnings) {
			logger::info(
				"[Validator] Validation complete: no errors, {} warning(s).", report.warnings.size());
		} else {
			logger::info("[Validator] Validation complete: all physics assets OK ({} XML file(s)).",
				report.totalXMLsFound);
		}

		// In strict mode, errors cause a return of false (logged, not fatal)
		if (g_validationConfig.strictMode && report.hasErrors) {
			return false;
		}

		return !report.hasErrors;
	}

}  // namespace hdt
