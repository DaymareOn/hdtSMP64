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

			// path::string() throws std::system_error if the path contains characters
			// that cannot be encoded in the system code page (e.g. non-ASCII mod names).
			// generic_u8string() is always safe but returns u8string (char8_t) in C++20;
			// reinterpret the bytes into a plain std::string (valid since char8_t is unsigned char).
			auto toStr = [](const std::filesystem::path& fp) -> std::string {
				auto u8 = fp.generic_u8string();
				return { reinterpret_cast<const char*>(u8.data()), u8.size() };
			};

			std::string pathStr;
			try {
				pathStr = toStr(p);
			} catch (const std::exception& e) {
				logger::warn("[Validator] Skipping unrepresentable path: {}", e.what());
				continue;
			}

			{
				auto ext = toStr(p.extension());
				std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
				if (ext != ".nif") {
					continue;
				}
			}

			++scanned;
			try {
				auto scanRes = ScanNIFBinary(pathStr);
				if (scanRes.hasPhysicsData) {
					PhysicsAsset asset;
					asset.nifPath = pathStr;
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
						asset.xmlPath = toStr(xmlFsPath);
						asset.xmlExists = fs::exists(xmlFsPath, ec);
					}

					result.push_back(std::move(asset));
				}
			} catch (const std::exception& e) {
				logger::warn("[Validator] Error scanning NIF {}: {}", pathStr, e.what());
			} catch (...) {
				logger::warn("[Validator] Unknown error scanning NIF {}", pathStr);
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

	// ---- Phase 3b: NIF-based validation ----

	static void validateNIFAssets(const std::vector<PhysicsAsset>& nifAssets,
		AssetValidationResult& report, std::ostream& out)
	{
		// Track already-validated XML paths to avoid re-running XSD/SCH on the same
		// file multiple times when several NIFs share a single physics XML.
		std::unordered_set<std::string> validatedXMLs;

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

				if (validatedXMLs.count(asset.xmlPath)) {
					out << "    (already validated)\n";
					continue;
				}
				validatedXMLs.insert(asset.xmlPath);

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
					++report.totalXMLsFound;

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

		// Phase 1b + 3b: NIF discovery and validation (optional)
		reportStream << "\n== Phase 1b: NIF File Discovery ==\n";
		auto nifAssets = discoverPhysicsNIFs();
		report.totalNIFsScanned = (int)nifAssets.size();

		if (!nifAssets.empty()) {
			reportStream << "\n== Phase 3b: NIF-Referenced XML Validation ==\n";
			validateNIFAssets(nifAssets, report, reportStream);
		}

		// Summary
		auto wallEnd = std::chrono::steady_clock::now();
		double elapsedSec = std::chrono::duration<double>(wallEnd - wallStart).count();
		report.elapsedSeconds = elapsedSec;

		reportStream << "\n== Summary ==\n";
		reportStream << "  Duration:      " << std::fixed << std::setprecision(2) << elapsedSec << "s\n";
		reportStream << "  XMLs found:    " << report.totalXMLsFound << "\n";
		reportStream << "  XMLs passed:   " << report.xmlPassCount << "\n";
		reportStream << "  XMLs failed:   " << report.xmlErrorCount << "\n";
		reportStream << "  NIFs scanned:  " << report.totalNIFsScanned << "\n";
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
