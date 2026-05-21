#include "hdtAssetValidator.h"

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#endif
#include "ActorManager.h"
#include "NetImmerseUtils.h"
#include "Improvers/hdtNIFImprover.h"
#include "Improvers/hdtNIFBogusNodeImprover.h"
#include "Improvers/hdtNIFOrphanedSkinImprover.h"
#include "Improvers/hdtNIFSkinMeshValidator.h"
#include "Improvers/hdtNIFBinaryIO.h"
#include "Validators/hdtNIFValidator.h"
#include "Validators/hdtSCHValidator.h"
#include "Config/hdtValidatorPaths.h"
#include "Utils/hdtConcurrencyUtils.h"
#include "Utils/hdtStringUtils.h"
#include "Utils/hdtXMLUtils.h"
#include "Utils/hdtTemplateDefaults.h"
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

// ═══════════════════════════════════════════════════════════════════════════════
// §1  Global state
// ═══════════════════════════════════════════════════════════════════════════════

	ValidationConfig g_validationConfig;

// ═══════════════════════════════════════════════════════════════════════════════
// §2  Violation classifiers
//     Pure predicates on XSD/SCH violation objects — no I/O, no side effects.
//     Order: XSD helpers, then SCH helpers, each group ending with the
//     aggregate hasBlocking* check that the pipeline uses as a gate.
// ═══════════════════════════════════════════════════════════════════════════════

	// Lazy-loads (once) the set of XSD element names typed "factor".
	// Used to decide whether an out-of-range SCH violation is a silent runtime
	// clamp rather than a genuine error — only [0,1]-bounded factor elements qualify.
	static const std::unordered_set<std::string>& getUnitFactorElementNamesFromXsd()
	{
		static std::once_flag once;
		static std::unordered_set<std::string> names;

		std::call_once(once, []() {
			pugi::xml_document doc;
			auto loadResult = doc.load_file(kPhysicsXSDPath);
			if (!loadResult) {
				logger::warn("[Validator] Could not load XSD from '{}': {}; [0,1] factor detection disabled.",
					kPhysicsXSDPath, loadResult.description());
				return;
			}

			auto schema = doc.child("xsd:schema");
			if (!schema)
				schema = doc.child("schema");
			if (!schema) {
				logger::warn("[Validator] Could not find schema root in '{}'; [0,1] factor detection disabled.",
					kPhysicsXSDPath);
				return;
			}

			for (auto element : schema.children()) {
				if (StripXmlNamespacePrefix(element.name()) != "element")
					continue;

				auto typeName = StripXmlNamespacePrefix(element.attribute("type").as_string());
				if (typeName != "factor")
					continue;

				std::string elementName = element.attribute("name").as_string();
				if (!elementName.empty())
					names.insert(std::move(elementName));
			}

			logger::info("[Validator] Loaded {} unit-factor element name(s) from XSD.", names.size());
		});

		return names;
	}

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

	static bool isOutOfRangeUnitFactorSchViolation(const SCHViolation& v)
	{
		// Check if message indicates [0,1] range violation
		const bool isUnitRangeMessage =
			v.message.find("value '") != std::string::npos &&
			v.message.find("is out of range: must be in [0, 1].") != std::string::npos;
		if (!isUnitRangeMessage)
			return false;

		auto elementName = ExtractElementNameFromSchLocation(v.location);
		if (elementName.empty())
			return false;

		const auto& factorNames = getUnitFactorElementNamesFromXsd();
		return factorNames.find(elementName) != factorNames.end();
	}

	static bool isRedundantDefaultValueSchWarning(const SCHViolation& v)
	{
		return v.role == SCHRole::Warning &&
		       v.message.find("is set to its default value") != std::string::npos;
	}

	static bool hasBlockingSchErrors(const SCHValidationResult& sch)
	{
		return std::any_of(sch.violations.begin(), sch.violations.end(), [](const SCHViolation& v) {
			return v.role == SCHRole::Error && !isOutOfRangeUnitFactorSchViolation(v);
		});
	}

// ═══════════════════════════════════════════════════════════════════════════════
// §3  Report helpers
//     Build and write report content.  Order: per-XML violation formatter →
//     parallel batch validator → file writer → errors-only formatter.
// ═══════════════════════════════════════════════════════════════════════════════

	using XMLValidationPair = std::pair<XSDValidationResult, SCHValidationResult>;

	// Load xmlPath from disk and collect per-element redundancy info.
	// Called by appendXmlViolationsToReport to cross-reference SCH default-value
	// warnings against actual runtime-effective template inheritance — a warning is
	// suppressed when the tag is not redundant relative to the inherited template.
	static std::vector<TemplateRedundantChildInfo> collectTemplateRedundantChildrenForXml(const std::string& xmlPath)
	{
		std::vector<TemplateRedundantChildInfo> result;

		std::string bytes = readAllFile2(xmlPath.c_str());
		if (bytes.empty())
			return result;

		pugi::xml_document doc;
		auto parseResult = doc.load_buffer(bytes.data(), bytes.size());
		if (!parseResult)
			return result;

		return CollectTemplateRedundantChildrenInfo(doc, &bytes);
	}

	/// Appends XSD violations, SCH violations, and template-redundant warnings for one XML
	/// file to both the structured report and the text stream.
	/// Callers must have already written a context line (e.g. "[OK]" or "-> xmlPath") to out.
	static void appendXmlViolationsToReport(const XMLValidationPair& pair, const std::string& xmlPath,
		AssetValidationResult& report, std::ostream& out)
	{
		const auto& [xsdResult, schResult] = pair;
		const auto templateRedundantChildren = collectTemplateRedundantChildrenForXml(xmlPath);
		std::unordered_map<std::string, TemplateRedundantChildInfo> templateRedundantByLocation;
		for (const auto& info : templateRedundantChildren)
			templateRedundantByLocation[info.location] = info;
		std::unordered_set<std::string> emittedTemplateRedundantLocations;

		auto emitTemplateRedundantWarning = [&](const TemplateRedundantChildInfo& info) {
			if (info.shadowedByLaterFrameTag) {
				std::string msg = xmlPath + ":" + std::to_string(info.line) + ": " + info.location +
				                  " - " + info.tagName + " is shadowed by later " + info.shadowingTagName +
				                  " in the same constraint/default block and has no effect. This tag is unnecessary and can be removed.";
				report.warnings.push_back(msg);
				report.hasWarnings = true;
				out << "    [WARNING] " << info.location << " (line " << info.line << "): "
				    << info.tagName << " is shadowed by later " << info.shadowingTagName
				    << " in the same constraint/default block and has no effect; this tag can be removed.\n";
				return;
			}

			std::string msg = xmlPath + ":" + std::to_string(info.line) + ": " + info.location +
			                  " - " + info.tagName +
			                  " is set to the effective inherited default value. This tag is unnecessary and can be removed.";
			report.warnings.push_back(msg);
			report.hasWarnings = true;
			out << "    [WARNING] " << info.location << " (line " << info.line << "): "
			    << info.tagName
			    << " is set to the effective inherited default value. This tag is unnecessary and can be removed.\n";
		};

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
			if (isOutOfRangeUnitFactorSchViolation(v)) {
				auto clampTarget = ExtractOutOfRangeClampTarget(v.message);
				std::string warningMsg = xmlPath + ":" + std::to_string(v.line) + ": " +
				                         v.location + " - " + v.message +
				                         " Runtime clamps this value to [0, 1]; effective value will be '" +
				                         clampTarget + "'.";
				report.warnings.push_back(warningMsg);
				report.hasWarnings = true;
				out << "    [WARNING] " << v.location << " (line " << v.line << "): "
					<< v.message << "; runtime clamps this value to [0, 1], so the effective value will be '"
					<< clampTarget << "'.\n";
				continue;
			}

			if (isRedundantDefaultValueSchWarning(v) &&
			    templateRedundantByLocation.find(v.location) == templateRedundantByLocation.end()) {
				// This warning was matched against theoretical XSD defaults, but the tag is
				// not redundant relative to the effective inherited template at runtime.
				continue;
			}
			if (isRedundantDefaultValueSchWarning(v))
				emittedTemplateRedundantLocations.insert(v.location);

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

		for (const auto& [location, info] : templateRedundantByLocation) {
			if (emittedTemplateRedundantLocations.find(location) != emittedTemplateRedundantLocations.end())
				continue;

			emitTemplateRedundantWarning(info);
		}
	}

	/// Validates multiple XML files in parallel, running both XSD and SCH validators on each.
	/// Both validators use std::once_flag-protected schema loading, making this thread-safe.
	/// Results are returned in the same order as input paths.
	static std::vector<XMLValidationPair> parallelValidateXMLs(const std::vector<std::string>& paths)
	{
		std::vector<XMLValidationPair> results(paths.size());
		ParallelForChunks(paths.size(), [&](size_t begin, size_t end) {
			for (size_t j = begin; j < end; ++j)
				results[j] = { ValidatePhysicsXMLWithXSD(paths[j]), ValidatePhysicsXMLWithSchematron(paths[j]) };
		});
		return results;
	}

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

// ═══════════════════════════════════════════════════════════════════════════════
// §4  Discovery
//     Locate assets on disk or at runtime.  Order: file-system leaf helpers →
//     BBP discovery → Win32 NIF scanner (private to discoverPhysicsAssets) →
//     discoverPhysicsAssets → path collector.
// ═══════════════════════════════════════════════════════════════════════════════

	/// Discovers TRI collision files related to a given NIF.
	/// TRI files are canonical and not weight-variant split:
	///   - foo.nif       -> foo.tri
	///   - foo_0.nif     -> foo.tri
	///   - foo_1.nif     -> foo.tri
	static std::vector<std::string> discoverRelatedTRIFiles(const std::string& nifPath)
	{
		namespace fs = std::filesystem;
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

		auto ext = PathToUtf8(nifFsPath.extension());
		std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
		if (ext != ".nif")
			return result;

		const fs::path parent = nifFsPath.parent_path();
		const std::string stem = PathToUtf8(nifFsPath.stem());

		std::string triStem = stem;
		if (stem.size() > 2 && (hasSuffix(stem, "_0") || hasSuffix(stem, "_1")))
			triStem = stem.substr(0, stem.size() - 2);

		std::vector<fs::path> candidates;
		candidates.push_back(parent / (triStem + ".tri"));

		for (const auto& candidate : candidates) {
			if (!fs::exists(candidate, ec) || ec || !fs::is_regular_file(candidate, ec) || ec)
				continue;

			auto pathStr = PathToUtf8(candidate);
			auto norm = NormalizePathForComparison(pathStr);
			if (seen.insert(norm).second)
				result.push_back(std::move(pathStr));
		}

		return result;
	}

	// Heuristic: derive sibling NIF paths from a resolved XML path (foo.nif, foo_0.nif, foo_1.nif).
	// Returns only paths that exist on disk. Avoids full mesh scans; used only to
	// produce human-readable NIF paths in violation messages, not for validation logic.
	static std::vector<std::string> getCandidateNifDiskPathsForXml(const std::string& resolvedXmlPath)
	{
		namespace fs = std::filesystem;
		std::vector<std::string> matches;
		std::unordered_set<std::string> seen;
		std::error_code ec;

		fs::path xmlFs = resolvedXmlPath;
		if (xmlFs.empty())
			return matches;

		// Fast heuristic: most XML paths map to sibling .nif / _0.nif / _1.nif.
		// Avoid expensive full mesh scans during equipped validation.
		xmlFs.replace_extension(".nif");
		for (const auto& candidate : {
			xmlFs,
			xmlFs.parent_path() / (xmlFs.stem().string() + "_0.nif"),
			xmlFs.parent_path() / (xmlFs.stem().string() + "_1.nif")
		}) {
			if (candidate.empty())
				continue;
			if (!fs::exists(candidate, ec) || ec || !fs::is_regular_file(candidate, ec) || ec)
				continue;

			auto pathStr = PathToUtf8(candidate);
			auto norm = NormalizePathForComparison(pathStr);
			if (seen.insert(norm).second)
				matches.push_back(std::move(pathStr));
		}

		return matches;
	}

	static std::string formatNifDiskPathForViolation(const std::string& resolvedXmlPath)
	{
		const auto& matches = getCandidateNifDiskPathsForXml(resolvedXmlPath);
		if (matches.empty())
			return {};
		if (matches.size() == 1)
			return matches.front();
		return matches.front() + " (+" + std::to_string(matches.size() - 1) + " more matching NIFs)";
	}

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

	// ── Win32 NIF scanner (native NTFS, no VFS) ──────────────────────────────
	// Two passes per directory:
	//   Pass 1  *.nif + FindExSearchNameMatch        → NTFS returns NIF files only
	//   Pass 2  *     + FindExSearchLimitToDirectories → NTFS returns dirs only
	//
	// This is effective on the native filesystem where NTFS applies the filter at
	// the driver level. It is ineffective through MO2's VFS because the VFS hook
	// enumerates all entries before applying the pattern.
	static void findNifsNative(const std::filesystem::path& dir,
	                            std::vector<std::string>& out)
	{
		// Single pass: enumerate all entries, collect NIFs and recurse into dirs.
		// Previously used two passes (*.nif + FindExSearchLimitToDirectories) but
		// FindExSearchLimitToDirectories is advisory and ignored by NTFS — Pass 2
		// enumerated all files anyway, doubling the I/O cost on animation mods.
		// One pass with FIND_FIRST_EX_LARGE_FETCH is faster overall.
		const std::wstring dirW = dir.wstring();
		WIN32_FIND_DATAW fd;

		HANDLE h = FindFirstFileExW((dirW + L"\\*").c_str(),
		                             FindExInfoBasic, &fd,
		                             FindExSearchNameMatch,
		                             nullptr, FIND_FIRST_EX_LARGE_FETCH);
		if (h == INVALID_HANDLE_VALUE) return;

		do {
			if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
				if (fd.cFileName[0] != L'.')
					findNifsNative(dir / fd.cFileName, out);
			} else {
				// Check for .nif extension (case-insensitive)
				const wchar_t* name = fd.cFileName;
				const wchar_t* dot  = wcsrchr(name, L'.');
				if (dot && _wcsicmp(dot, L".nif") == 0) {
					try {
						out.push_back(PathToUtf8(dir / name));
					} catch (...) {}
				}
			}
		} while (FindNextFileW(h, &fd));

		FindClose(h);
	}

	/// Discovers physics-enabled assets from either filesystem or runtime.
	/// When equippedOnly=false: Scans data/ (or the configured physical mods dir) for NIF files.
	///   Phase 1 (serial + parallel): directory walk to collect .nif paths.
	///   Phase 2 (parallel): binary scan to detect physics-marker blocks in each NIF.
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

					auto [xmlPath, xmlExists] = ResolveXMLPath(armor.physicsFile.first);
					std::string armorName = armor.armorWorn->name.size() ? armor.armorWorn->name.c_str() : "<unnamed>";
					PhysicsAsset asset;
					// For equipped-gear validation we don't have stable NIF file paths at runtime,
					// so nifPath is used as a human-readable item identifier in the report output.
					asset.nifPath = skeleton.name() + " [armor:" + armorName + "]";
					asset.nifExists = true;
					asset.xmlPath = xmlPath;
					asset.xmlExists = xmlExists;
					asset.allPhysicsXmlPaths.push_back(armor.physicsFile.first);
					++equippedCount;
					if (outNifScanViolations) {
						if (auto* armorRoot = castNiNode(armor.armorWorn.get())) {
							auto structural = ValidateNIFStructure(armorRoot, asset.nifPath);
							for (const auto& err : structural.errors)
								outNifScanViolations->push_back(err);
							for (const auto& warn : structural.warnings)
								logger::warn("[Validator] Equipped NIF warning: {}", warn);
						} else {
							const auto nifDiskPath = formatNifDiskPathForViolation(asset.xmlPath);
							if (!nifDiskPath.empty())
								outNifScanViolations->push_back(nifDiskPath + ": equipped armor node is not a NiNode (physics XML: " + asset.xmlPath + ")");
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

					auto [xmlPath, xmlExists] = ResolveXMLPath(headPart.physicsFile.first);
					std::string headPartName = headPart.headPart->name.size() ? headPart.headPart->name.c_str() : "<unnamed>";
					PhysicsAsset asset;
					asset.nifPath = skeleton.name() + " [headpart:" + headPartName + "]";
					asset.nifExists = true;
					asset.xmlPath = xmlPath;
					asset.xmlExists = xmlExists;
					asset.allPhysicsXmlPaths.push_back(headPart.physicsFile.first);
					++equippedCount;
					if (outNifScanViolations) {
						if (auto* headRoot = castNiNode(headPart.headPart.get())) {
							auto structural = ValidateNIFStructure(headRoot, asset.nifPath);
							for (const auto& err : structural.errors)
								outNifScanViolations->push_back(err);
							for (const auto& warn : structural.warnings)
								logger::warn("[Validator] Equipped NIF warning: {}", warn);
						} else {
							const auto nifDiskPath = formatNifDiskPathForViolation(asset.xmlPath);
							if (!nifDiskPath.empty())
								outNifScanViolations->push_back(nifDiskPath + ": equipped headpart node is not a NiNode (physics XML: " + asset.xmlPath + ")");
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
			// Temporary debug mode: scan a single hardcoded NIF directly.
			namespace fs = std::filesystem;
			using Clock = std::chrono::high_resolution_clock;
			auto msElapsed = [](Clock::time_point a, Clock::time_point b) {
				return std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
			};
			std::error_code ec;

			fs::path scanRoot = "data";
			if (!fs::exists(scanRoot, ec) || !fs::is_directory(scanRoot, ec)) {
				logger::warn("[Validator] Data directory not found: {}", PathToUtf8(scanRoot));
				return {};
			}

			// ── Phase 1a: serial first-level scan ────────────────────────────────
			// Single non-recursive pass over data/ to collect top-level subdirs.
			// Timing each dir reveals which ones are slow to open (VFS overhead).
			auto tp1a = Clock::now();
			std::vector<std::string> nifPaths;
			std::vector<fs::path>    scanTasks; // dirs for parallel scan

			// Physical mods directory bypass: enumerate each mod's directory
			// directly on NTFS, avoiding MO2 VFS hook overhead entirely.
			// Driven by <mods-dir> in configs.xml; falls back to the VFS scan of
			// data/ when modsDir is empty (Vortex users, or unconfigured installs).
			const fs::path kPhysModsDir = !g_validationConfig.modsDir.empty()
			    ? fs::path(g_validationConfig.modsDir) : fs::path{};
			const bool physScan = !kPhysModsDir.empty()
			    && fs::exists(kPhysModsDir, ec) && fs::is_directory(kPhysModsDir, ec);
			ec.clear();

			if (physScan) {
				logger::info("[Validator][PROF] Phase 1: using physical mods dir: {}", PathToUtf8(kPhysModsDir));
				for (auto& entry : fs::directory_iterator(kPhysModsDir, ec)) {
					if (ec) { ec.clear(); continue; }
					if (entry.is_directory(ec) && !ec)
						scanTasks.push_back(entry.path());
				}
			} else {
				// VFS scan of data/ (original path)
				for (auto& entry : fs::directory_iterator(scanRoot, ec)) {
					if (ec) { ec.clear(); continue; }
					if (entry.is_directory(ec) && !ec) {
						scanTasks.push_back(entry.path());
					} else if (entry.is_regular_file(ec) && !ec) {
						auto ext = PathToUtf8(entry.path().extension());
						std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
						if (ext == ".nif") {
							try { nifPaths.push_back(PathToUtf8(entry.path())); } catch (...) {}
						}
					}
				}
			}

			auto tp1b = Clock::now();

			// ── Phase 1b: parallel recursive scan of all top-level subdirs ─────
			// One future per top-level dir so heavy dirs (meshes/, textures/) and
			// light dirs (sounds/, scripts/) overlap naturally.
			// Note: FindFirstFileExW with *.nif filter was tested but proved slower —
			// MO2's VFS hook processes all entries before applying the filter, so
			// a two-pass approach pays the VFS cost twice. Single-pass
			// recursive_directory_iterator is optimal in this environment.
			// Per-task wall time recorded for the slowest-dir report.
			std::vector<std::vector<std::string>> perDirNifs(scanTasks.size());
			std::vector<long long> taskMs(scanTasks.size(), 0);
			{
				const size_t n = scanTasks.size();
				std::vector<std::future<void>> futures;
				futures.reserve(n);
				for (size_t j = 0; j < n; ++j) {
					futures.push_back(std::async(std::launch::async, [&, j]() {
						auto tTask = Clock::now();
						if (physScan) {
							// Native NTFS: Win32 two-pass scan (*.nif + dirs only).
							// NTFS applies the filter at the driver level so we never
							// pay the per-entry cost for non-NIF files.
							findNifsNative(scanTasks[j], perDirNifs[j]);
						} else {
							// VFS fallback: single-pass recursive_directory_iterator.
							std::error_code ec2;
							for (auto& entry : fs::recursive_directory_iterator(
									scanTasks[j],
									fs::directory_options::skip_permission_denied,
									ec2)) {
								if (ec2) { ec2.clear(); continue; }
								if (!entry.is_regular_file(ec2) || ec2) { ec2.clear(); continue; }
								auto ext = PathToUtf8(entry.path().extension());
								std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
								if (ext != ".nif") continue;
								try { perDirNifs[j].push_back(PathToUtf8(entry.path())); } catch (...) {}
							}
						}
						taskMs[j] = msElapsed(tTask, Clock::now());
					}));
				}
				for (auto& f : futures) f.get();
			}
			auto tp1c = Clock::now();

			// ── Phase 1c: merge ───────────────────────────────────────────────────
			if (physScan) {
				// Physical scan: convert absolute paths to data/-relative virtual paths
				// and deduplicate — multiple mods may supply the same NIF (priority
				// overrides).  We keep the first occurrence; Phase 2+3 open paths
				// through the VFS so they always get the highest-priority version.
				std::unordered_set<std::string> seen;
				for (size_t j = 0; j < scanTasks.size(); ++j) {
					// Prefix = "c:/Modlists/JOJ/mods/<ModName>/" (forward slashes, trailing /)
					std::string prefix = PathToUtf8(scanTasks[j]);
					if (!prefix.empty() && prefix.back() != '/') prefix += '/';

					for (auto& physPath : perDirNifs[j]) {
						if (physPath.size() <= prefix.size()) continue;
						// Case-insensitive prefix strip (Windows paths may differ in case)
						bool prefixMatch = _strnicmp(physPath.c_str(), prefix.c_str(), prefix.size()) == 0;
						if (!prefixMatch) continue;
						std::string rel = physPath.substr(prefix.size());
						std::string virt = "data/" + rel;
						if (seen.insert(virt).second)
							nifPaths.push_back(std::move(virt));
					}
				}
			} else {
				size_t total = nifPaths.size();
				for (auto& v : perDirNifs) total += v.size();
				nifPaths.reserve(total);
				for (auto& v : perDirNifs)
					for (auto& p : v) nifPaths.push_back(std::move(p));
			}
			auto tp1d = Clock::now();

			logger::info("[Validator][PROF] Phase 1 breakdown ({} tasks, {} NIFs found, mode={}):",
				scanTasks.size(), nifPaths.size(), physScan ? "physical-mods" : "vfs-data");
			logger::info("[Validator][PROF]   1a serial scan        {:>6} ms", msElapsed(tp1a, tp1b));
			logger::info("[Validator][PROF]   1b parallel scan      {:>6} ms  (wall, {} tasks)", msElapsed(tp1b, tp1c), scanTasks.size());
			logger::info("[Validator][PROF]   1c merge              {:>6} ms", msElapsed(tp1c, tp1d));
			logger::info("[Validator][PROF]   Phase 1 total         {:>6} ms", msElapsed(tp1a, tp1d));

			// Per-task parallel scan timings (top 15 slowest) — shows which
			// second-level dirs dominate the parallel scan wall time.
			struct TaskStat { std::string path; long long ms; size_t nifs; };
			std::vector<TaskStat> taskStats;
			taskStats.reserve(scanTasks.size());
			for (size_t j = 0; j < scanTasks.size(); ++j)
				taskStats.push_back({ PathToUtf8(scanTasks[j]), taskMs[j], perDirNifs[j].size() });
			std::sort(taskStats.begin(), taskStats.end(),
				[](const auto& a, const auto& b) { return a.ms > b.ms; });
			logger::info("[Validator][PROF] Slowest scan tasks (top 15):");
			for (size_t i = 0; i < std::min<size_t>(15, taskStats.size()); ++i)
				logger::info("[Validator][PROF]   {:>6} ms  {:>6} nifs  {}",
					taskStats[i].ms, taskStats[i].nifs, taskStats[i].path);

			logger::info("[Validator] Found {} NIF files, scanning for physics data...", nifPaths.size());
			if (outFilesystemNifFilesDiscovered)
				*outFilesystemNifFilesDiscovered = static_cast<int>(nifPaths.size());

			// ── Phase 2: parallel physics marker scan ─────────────────────────────
			auto tp2a = Clock::now();
			const size_t n = nifPaths.size();
			std::vector<std::optional<PhysicsAsset>> scanResults(n);
			std::vector<std::vector<std::string>> scanViolations(n);

			ParallelForChunks(n, [&](size_t begin, size_t end) {
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
							auto [xmlPath, xmlExists] = ResolveXMLPath(scanRes.physicsXmlPath);
							asset.xmlPath = xmlPath;
							asset.xmlExists = xmlExists;
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
			auto tp2b = Clock::now();

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
			auto tp2c = Clock::now();

			logger::info("[Validator][PROF] Phase 2 breakdown ({} NIFs scanned, {} physics):",
				n, result.size());
			logger::info("[Validator][PROF]   2a parallel scan     {:>6} ms  (wall)", msElapsed(tp2a, tp2b));
			logger::info("[Validator][PROF]   2b collect results   {:>6} ms", msElapsed(tp2b, tp2c));
			logger::info("[Validator][PROF]   Phase 2 total        {:>6} ms", msElapsed(tp2a, tp2c));

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

// ═══════════════════════════════════════════════════════════════════════════════
// §5  Validation pipeline
//     Phase-ordered validators called by runValidationCore.
// ═══════════════════════════════════════════════════════════════════════════════

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
					out << "  [ERROR] " << asset.nifPath << "\n";
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
				out << "  [ERROR] " << asset.nifPath << "\n";
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
				out << "  [ERROR] " << asset.nifPath << " vs " << asset1.nifPath << "\n";
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
						out << "  [ERROR] " << asset.nifPath << " vs " << asset1.nifPath << "\n";
						out << "    Block " << k << " mismatch:\n";
						out << "      _0.nif: " << paths0[k] << "\n";
						out << "      _1.nif: " << paths1[k] << "\n";
					}
				}
			}
		}
	}

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
				bool xmlHasErrors = hasBlockingXsdErrors(pair.first) || hasBlockingSchErrors(pair.second);

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

// ═══════════════════════════════════════════════════════════════════════════════
// §6  Improvement
//     Generate corrected copies of physics assets.
// ═══════════════════════════════════════════════════════════════════════════════

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

	/// Writes cleaned-up copies of physics XML files to outputDir, removing unknown
	/// and misplaced elements. Only files where at least one element was removed are written.
	static XMLImproveResult improveXMLPaths(const std::vector<std::string>& xmlPaths,
		const std::string& outputDir,
		bool copyOriginal,
		bool stateless)
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
				if (GenerateImprovedXML(xmlPath, outputDir, copyOriginal, stateless))
					++result.xmlImprovedCount;
			} catch (const std::exception& e) {
				result.errors.push_back("Failed to improve XML " + xmlPath + ": " + e.what());
			} catch (...) {
				result.errors.push_back("Failed to improve XML " + xmlPath + ": unknown error");
			}
		}

		return result;
	}

// ═══════════════════════════════════════════════════════════════════════════════
// §7  Orchestration
//     runValidationCore drives the full pipeline in phase order.
// ═══════════════════════════════════════════════════════════════════════════════

	/// Executes the complete validation pipeline (full or equipped-only) and generates a report.
	/// Full pipeline (equippedOnly=false):
	///   - Phase 0: Validates DefaultBBP XML entries.
	///   - Phase 2: Discovers NIFs via filesystem or mods-dir scan.
	///   - Phase 2.5: Validates _0/_1 NIF pair consistency.
	///   - Phase 3: Validates NIF-referenced XMLs with NIF context.
	/// Equipped-only pipeline (equippedOnly=true):
	///   - Discovers equipped armor and headparts.
	///   - Validates their physics XMLs.
	/// Parse each NIF binary and run two structural checks: orphaned NiSkinInstance
	/// blocks (no NiSkinPartition child — runtime crash) and the full set of skin mesh
	/// integrity issues from steps 4–11 of decimateCandidateFailClosed.
	static void validateNIFStructure(const std::vector<PhysicsAsset>& nifAssets,
		AssetValidationResult& report, std::ostream& out)
	{
		for (const auto& asset : nifAssets) {
			if (!asset.nifExists) continue;

			std::ifstream in(asset.nifPath, std::ios::binary | std::ios::ate);
			if (!in.is_open()) continue;
			auto sz = in.tellg();
			if (sz <= 0 || sz > static_cast<std::streamoff>(nif::kMaxNifFileSize)) continue;
			std::vector<uint8_t> data(static_cast<size_t>(sz));
			in.seekg(0);
			in.read(reinterpret_cast<char*>(data.data()), sz);
			if (in.gcount() != static_cast<std::streamsize>(sz)) continue;

			auto parsedOpt = parseNif(data);
			if (!parsedOpt) continue;

			int count = countOrphanedSkinInstances(*parsedOpt);
			if (count > 0) {
				std::string err = asset.nifPath + ": " + std::to_string(count) +
					" NiSkinInstance block(s) with no NiSkinPartition ref"
					" — would crash the physics runtime. Run 'smp fix nif' to fix.";
				report.errors.push_back(err);
				report.hasErrors = true;
				out << "  [NIF]  " << asset.nifPath << "\n";
				out << "    [ERROR] " << count << " NiSkinInstance block(s) missing NiSkinPartition"
					" — would crash the physics runtime. Run 'smp fix nif' to fix.\n";
			}

			auto skinIssues = detectNIFSkinMeshIssues(*parsedOpt);
			if (!skinIssues.empty()) {
				out << "  [NIF]  " << asset.nifPath << "\n";
				for (const auto& issue : skinIssues) {
					report.skinMeshIssuesFound += 1;

					std::string msg;
					bool isError = true;
					if (issue.reasonCode == "unsupported-trishape-layout") {
						msg = "BSTriShape[" + std::to_string(issue.triShapeBlockIndex) +
						      "] (" + issue.shapeType + "): vertex format not recognised by this tool.";
						isError = false;
					} else if (issue.reasonCode == "unsupported-skin-partition-layout") {
						msg = "BSTriShape[" + std::to_string(issue.triShapeBlockIndex) +
						      "] (" + issue.shapeType + "): NiSkinPartition format not recognised by this tool.";
						isError = false;
					} else if (issue.reasonCode == "shape-partition-count-mismatch") {
						msg = "BSTriShape[" + std::to_string(issue.triShapeBlockIndex) +
						      "] (" + issue.shapeType + "): BSTriShape and NiSkinPartition report different"
						      " vertex or triangle counts — data corruption.";
					} else if (issue.reasonCode == "unsupported-non-permutation-vertex-map") {
						msg = "BSTriShape[" + std::to_string(issue.triShapeBlockIndex) +
						      "] (" + issue.shapeType + "): NiSkinPartition vertex map is not a bijection"
						      " — malformed data.";
					} else if (issue.reasonCode == "shape-partition-vertexdata-mismatch") {
						msg = "BSTriShape[" + std::to_string(issue.triShapeBlockIndex) +
						      "] (" + issue.shapeType + "): BSTriShape and NiSkinPartition vertex data"
						      " are inconsistent — data corruption.";
					} else if (issue.reasonCode == "partition-triangle-copy-mismatch") {
						msg = "BSTriShape[" + std::to_string(issue.triShapeBlockIndex) +
						      "] (" + issue.shapeType + "): NiSkinPartition has two inconsistent triangle"
						      " arrays. Run 'smp fix nif' to fix.";
					} else if (issue.reasonCode == "shape-partition-triangle-mismatch") {
						msg = "BSTriShape[" + std::to_string(issue.triShapeBlockIndex) +
						      "] (" + issue.shapeType + "): BSTriShape and NiSkinPartition triangle lists"
						      " are inconsistent — data corruption.";
					} else if (issue.reasonCode == "triangle-index-out-of-range") {
						msg = "BSTriShape[" + std::to_string(issue.triShapeBlockIndex) +
						      "] (" + issue.shapeType + "): NiSkinPartition triangle references a"
						      " non-existent vertex — data corruption.";
					} else {
						msg = "BSTriShape[" + std::to_string(issue.triShapeBlockIndex) +
						      "] (" + issue.shapeType + "): " + issue.reasonCode + ".";
					}

					const std::string fullMsg = asset.nifPath + ": " + msg;
					if (isError) {
						report.errors.push_back(fullMsg);
						report.hasErrors = true;
						out << "    [ERROR] " << msg << "\n";
					} else {
						report.warnings.push_back(fullMsg);
						report.hasWarnings = true;
						out << "    [WARNING] " << msg << "\n";
					}
				}
			}
		}
	}

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

				bodyStream << "\n== Phase 2.5: NIF Structural Validation ==\n";
				validateNIFStructure(equippedAssets, report, bodyStream);
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
					bool fileHasErrors = hasBlockingXsdErrors(pair.first) || hasBlockingSchErrors(pair.second);

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

				bodyStream << "\n== Phase 3.5: NIF Structural Validation ==\n";
				validateNIFStructure(nifAssets, report, bodyStream);
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
		reportStream << "\n";

		reportStream << bodyStream.str();
		reportStream << "\n";
		reportStream << tailStream.str();
		reportStream << "========================================\n";
		return reportStream.str();
	}

// ═══════════════════════════════════════════════════════════════════════════════
// §8  Public API
// ═══════════════════════════════════════════════════════════════════════════════

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

	/// Generates improved versions of all discovered physics XML files (or equipped items only) in the specified output directory.
	XMLImproveResult ImprovePhysicsXMLs(const std::string& outputDir, bool equippedOnly, bool copyOriginal, bool stateless)
	{
		logger::info("[Validator] Starting {} XML cleanup...",
			equippedOnly ? "equipped gear" : "on-demand FSMP");
		auto result = improveXMLPaths(
			collectPhysicsXMLPaths(equippedOnly),
			outputDir,
			copyOriginal,
			stateless);
		logger::info("[Validator] {} XML cleanup complete: {} XML(s) found, {} improved, {} error(s).",
			equippedOnly ? "Equipped gear" : "On-demand",
			result.totalXMLsFound, result.xmlImprovedCount, result.errors.size());
		return result;
	}

	/// Generates improved versions of all physics NIF files in the specified output directory.
	/// Applies decimation (if configured) and generates clean versions of NIFs.
	/// Preserves _0/_1 NIF pair atomicity by copying unchanged siblings when one is improved.
	NIFImproveResult ImprovePhysicsNIFs(const std::string& outputDir, bool equippedOnly, bool copyOriginal)
	{
		auto collectMissingPhysicsXmlRefs = [](const PhysicsAsset& asset) {
			std::unordered_set<std::string> missingRefs;
			for (const auto& rawPath : asset.allPhysicsXmlPaths) {
				auto [resolvedPath, xmlExists] = ResolveXMLPath(rawPath);
				if (xmlExists)
					continue;
				missingRefs.insert(NormalizePathForComparison(rawPath));
				if (!resolvedPath.empty())
					missingRefs.insert(NormalizePathForComparison(resolvedPath));
			}
			return missingRefs;
		};

		NIFImproveResult result;
		if (outputDir.empty()) {
			result.errors.push_back("Output directory is empty");
			return result;
		}

		auto decimationOptions = buildDecimationOptionsFromConfig();

		if (equippedOnly) {
			auto nifAssetsEquipped = discoverPhysicsAssets(true);
			result.totalNIFsFound = static_cast<int>(nifAssetsEquipped.size());

			std::map<std::string, std::vector<size_t>> stemGroups;
			for (size_t i = 0; i < nifAssetsEquipped.size(); ++i)
				stemGroups[GetNifPairBaseStem(NormalizePathForComparison(nifAssetsEquipped[i].nifPath))].push_back(i);

			for (auto& [stem, indices] : stemGroups) {
				std::vector<bool> results;
				results.reserve(indices.size());
				for (size_t idx : indices) {
					auto missingRefs = collectMissingPhysicsXmlRefs(nifAssetsEquipped[idx]);
					results.push_back(GenerateImprovedNIF(
						nifAssetsEquipped[idx].nifPath,
						outputDir,
						decimationOptions,
						nullptr,
						&missingRefs,
						copyOriginal));
				}

				if (!std::any_of(results.begin(), results.end(), [](bool r) { return r; }))
					continue;

				result.nifImprovedCount += static_cast<int>(std::count(results.begin(), results.end(), true));
				for (size_t k = 0; k < indices.size(); ++k) {
					if (!results[k]) {
						if (!CopyNIFToOutput(nifAssetsEquipped[indices[k]].nifPath, outputDir))
							logger::warn("[Validator] Failed to copy unchanged NIF sibling to output: {}", nifAssetsEquipped[indices[k]].nifPath);
					}
				}
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

		// Group by base stem so _0/_1 pairs are processed atomically (same as Phase 5).
		std::map<std::string, std::vector<size_t>> stemGroups;
		for (size_t i = 0; i < nifAssets.size(); ++i)
			stemGroups[GetNifPairBaseStem(NormalizePathForComparison(nifAssets[i].nifPath))].push_back(i);

		std::vector<std::vector<size_t>> groups;
		groups.reserve(stemGroups.size());
		for (auto& [stem, indices] : stemGroups)
			groups.push_back(std::move(indices));

		std::atomic<int> improvedCount{ 0 };
		std::atomic<int> orphanedSkinCount{ 0 };
		std::atomic<int> skinMeshFixedCount{ 0 };
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
				auto missingRefs = collectMissingPhysicsXmlRefs(nifAssets[idx]);
				results.push_back(GenerateImprovedNIF(
					nifAssets[idx].nifPath,
					outputDir,
					decimationOptions,
					&d,
					&missingRefs,
					copyOriginal));
				orphanedSkinCount.fetch_add(d.orphanedSkinInstancesRemoved);
				skinMeshFixedCount.fetch_add(d.skinMeshIssuesFixed);
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
				if (!d.validationError.empty())
					logger::error("[Validator] NIF round-trip validation failed for '{}': {}", nifAssets[idx].nifPath, d.validationError);
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

		resetNIFImproverTimings();
		resetBogusNodeTimings();
		if (groups.size() > 1)
			tbb::parallel_for_each(groups.begin(), groups.end(), improveGroup);
		else
			for (const auto& g : groups)
				improveGroup(g);

		logNIFImproverTimings();
		result.nifImprovedCount = improvedCount.load();
		result.orphanedSkinInstancesRemoved = orphanedSkinCount.load();
		result.skinMeshIssuesFixed = skinMeshFixedCount.load();
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
