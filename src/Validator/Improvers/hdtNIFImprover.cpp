#include "hdtNIFImprover.h"

#include "hdtNIFBinaryIO.h"

#include "hdtNIFBogusNodeImprover.h"

#include "hdtNIFCollisionDecimationImprover.h"

#include "../Utils/hdtStringUtils.h"

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

	bool GenerateImprovedNIF(
		const std::string& srcNIFPath,
		const std::string& outputDir,
		const NIFDecimationOptions& options,
		NIFImproverDiagnostics* outDiagnostics)
	{
		namespace fs = std::filesystem;

		if (outDiagnostics)
			*outDiagnostics = {};

		std::ifstream in(std::filesystem::u8path(srcNIFPath), std::ios::binary | std::ios::ate);
		if (!in.is_open())
			return false;
		auto sz = in.tellg();
		if (sz <= 0 || sz > kMaxNifFileSizeBytes)
			return false;
		std::vector<uint8_t> data(static_cast<size_t>(sz));
		in.seekg(0);
		in.read(reinterpret_cast<char*>(data.data()), sz);
		if (!in.good() && !in.eof())
			return false;

		auto parsedOpt = parseNif(data);
		if (!parsedOpt.has_value())
			return false;
		auto& parsed = *parsedOpt;

		if (!parsed.groups.empty())
			return false;

		bool changed = removeBogusTailNodes(parsed);

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

		if (!changed)
			return false;

		std::string relative = stripDataPrefix(srcNIFPath);
		fs::path outPath = fs::path(outputDir) / relative;
		std::error_code ec;
		fs::create_directories(outPath.parent_path(), ec);
		if (ec)
			return false;

		return writeNifFile(parsed, pathToUtf8(outPath));
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
