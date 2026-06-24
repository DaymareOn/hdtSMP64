#include "hdtPatternLibrary.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

namespace hdt
{
	namespace
	{
		// Folder, relative to the game working directory, scanned for shared pattern-definition files.
		// Sibling of the deployed hdtSMP64.xsd/.sch, mirroring the defaultBBPs.xml data-file convention.
		constexpr const char* kPatternsDir = "data/skse/plugins/hdtSkinnedMeshConfigs/patterns";

		std::vector<PatternLibrary> g_libraries;
		std::once_flag g_once;

		std::string readWholeFile(const std::filesystem::path& path)
		{
			std::ifstream f(path, std::ios::binary);
			if (!f)
				return {};
			std::ostringstream ss;
			ss << f.rdbuf();
			return ss.str();
		}

		// Reads every *.xml in the patterns folder in filename order, so a name clash across mods resolves
		// deterministically (later filename wins -- authors should still namespace with author= to avoid
		// relying on order). A missing folder simply yields no libraries.
		void loadOnce()
		{
			namespace fs = std::filesystem;
			std::error_code ec;
			const fs::path dir(kPatternsDir);
			if (!fs::is_directory(dir, ec))
				return;

			std::vector<fs::path> files;
			for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec))
			{
				if (!it->is_regular_file(ec))
					continue;
				std::string ext = it->path().extension().string();
				std::transform(ext.begin(), ext.end(), ext.begin(),
					[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
				if (ext == ".xml")
					files.push_back(it->path());
			}
			std::sort(files.begin(), files.end());

			for (const fs::path& p : files)
			{
				std::string xml = readWholeFile(p);
				if (!xml.empty())
					g_libraries.push_back({ std::move(xml), p.filename().string() });
			}
		}
	}  // namespace

	const std::vector<PatternLibrary>& getGlobalPatternLibraries()
	{
		std::call_once(g_once, loadOnce);
		return g_libraries;
	}
}
