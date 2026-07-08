#include "hdtDefaultBBP.h"

#include "NetImmerseUtils.h"
#include "XmlReader.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

namespace hdt
{
	DefaultBBP* DefaultBBP::instance()
	{
		static DefaultBBP s;
		return &s;
	}

	std::optional<DefaultBBP::PhysicsFile_t> DefaultBBP::scanEmbeddedBBP(RE::NiNode* scan)
	{
		for (int i = 0; i < scan->extraDataSize; ++i) {
			auto stringData = netimmerse_cast<RE::NiStringExtraData*>(scan->extra[i]);
			if (stringData && stringData->name == "HDT Skinned Mesh Physics Object" && stringData->value) {
				return PhysicsFile_t{ { std::string(stringData->value) }, defaultNameMap(scan) };
			}
		}

		return std::nullopt;
	}

	DefaultBBP::PhysicsFile_t DefaultBBP::scanBBP(RE::NiNode* scan)
	{
		// A present marker is authoritative even when its path is empty (malformed content): it must
		// yield no physics rather than fall through to a defaultBBPs name-matching the author never
		// asked for. Only markerless meshes consult the defaultBBPs mappings.
		if (auto embedded = scanEmbeddedBBP(scan)) {
			return *embedded;
		}

		return scanDefaultBBP(scan);
	}

	std::string DefaultBBP::getCreatureDefaultFile(const char* skeletonPath) const
	{
		if (!skeletonPath || !*skeletonPath) {
			return "";
		}

		std::string key(skeletonPath);
		std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

		auto it = creatureFileList.find(key);
		return it == creatureFileList.end() ? "" : it->second;
	}

	DefaultBBP::DefaultBBP()
	{
		loadDefaultBBPs();
	}

	namespace
	{
		// Read a loose file straight off disk. The config folder holds loose files, which the mod manager's
		// virtual file system makes visible to a normal file read. Returns "" if missing or unreadable.
		std::string readLooseFile(const std::filesystem::path& path)
		{
			std::ifstream f(path, std::ios::binary);
			if (!f)
				return {};
			std::ostringstream ss;
			ss << f.rdbuf();
			return ss.str();
		}
	}

	void DefaultBBP::loadDefaultBBPs()
	{
		// Two sources feed the same tables: the single legacy defaultBBPs.xml, and -- new -- every *.xml
		// dropped into the sibling defaultBBPs/ folder. The folder lets many mods each ship their own
		// mappings (for example one <creature> file per creature race) without the long-standing
		// single-file conflict, where only one defaultBBPs.xml survives the mod manager's virtual file
		// system and the rest are hidden.
		//
		// Precedence: the single defaultBBPs.xml is read FIRST, and every insert below keeps the FIRST value
		// seen for a shape/skeleton key, so the legacy file stays authoritative and the folder is purely
		// additive -- a folder file can only add mappings the single file does not already define, never
		// silently change an existing one. Among folder files the alphabetically-earlier name wins (prefix a
		// file with "00-" to raise its priority over other folder files). The folder is empty until a mod
		// ships one, so no existing load order changes behavior.
		auto single = readAllFile("SKSE/Plugins/hdtSkinnedMeshConfigs/defaultBBPs.xml");
		parseDefaultBBPsDocument(single);

		// Raw filesystem so we can list the directory; the VFS makes each mod's loose files visible here.
		namespace fs = std::filesystem;
		const fs::path folder = "data/skse/plugins/hdtSkinnedMeshConfigs/defaultBBPs";
		std::error_code ec;
		if (!fs::is_directory(folder, ec))
			return;

		std::vector<fs::path> files;
		for (fs::directory_iterator it(folder, ec), end; !ec && it != end; it.increment(ec)) {
			if (!it->is_regular_file(ec))
				continue;
			std::string ext = it->path().extension().string();
			std::transform(ext.begin(), ext.end(), ext.begin(),
				[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			if (ext == ".xml")
				files.push_back(it->path());
		}
		std::sort(files.begin(), files.end());

		for (const fs::path& p : files) {
			auto xml = readLooseFile(p);
			parseDefaultBBPsDocument(xml);
		}
	}

	// Load one "default-bbps" XML file by path and merge it into the tables. Public entry point so optional
	// extra mappings (e.g. obstructionBBPs.xml, used by the experimental world-collision feature) can be
	// merged on top of the defaults. Reads the file, then hands it to parseDefaultBBPsDocument -- a missing or
	// empty file parses to nothing, so callers can reference optional files without guarding first.
	void DefaultBBP::loadBBP(const char* path)
	{
		auto loaded = readAllFile(path);
		parseDefaultBBPsDocument(loaded);
	}

	// Parse one already-loaded <default-bbps> document into the tables. Shared by the single defaultBBPs.xml
	// and each drop-in in the defaultBBPs/ folder. Every insert is first-wins, so the caller's feed order
	// decides precedence between documents.
	void DefaultBBP::parseDefaultBBPsDocument(std::string& loaded)
	{
		if (loaded.empty())
			return;

		XMLReader reader((uint8_t*)loaded.data(), loaded.size());

		reader.nextStartElement();
		if (reader.GetName() != "default-bbps")
			return;

		while (reader.Inspect()) {
			if (reader.GetInspected() == Xml::Inspected::StartTag) {
				if (reader.GetName() == "map") {
					try {
						auto shape = reader.getAttribute("shape");
						auto file = reader.getAttribute("file");
						bbpFileList.insert(std::make_pair(shape, file));
					} catch (...) {
						logger::warn("defaultBBP({},{}) : invalid map", reader.GetRow(), reader.GetColumn());
					}
					reader.skipCurrentElement();
				} else if (reader.GetName() == "creature") {
					// <creature skeleton="Actors\...\skeleton.nif" file="physics.xml"/>: a per-race default,
					// keyed on the race's skeleton NIF path. Stored lowercased so lookup is case-insensitive.
					try {
						auto skeleton = reader.getAttribute("skeleton");
						auto file = reader.getAttribute("file");
						std::transform(skeleton.begin(), skeleton.end(), skeleton.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
						logger::debug("creature physics: defaultBBPs entry skeleton '{}' -> '{}'", skeleton, file);
						creatureFileList.insert(std::make_pair(skeleton, file));
					} catch (...) {
						logger::warn("defaultBBP({},{}) : invalid creature", reader.GetRow(), reader.GetColumn());
					}
					reader.skipCurrentElement();
				} else if (reader.GetName() == "remap") {
					auto target = reader.getAttribute("target");
					Remap remap = { target, {}, {} };
					while (reader.Inspect()) {
						if (reader.GetInspected() == Xml::Inspected::StartTag) {
							if (reader.GetName() == "source") {
								int priority = 0;
								try {
									priority = reader.getAttributeAsInt("priority");
								} catch (...) {}
								auto source = reader.readText();
								remap.entries.insert({ priority, source });
							} else if (reader.GetName() == "requires") {
								auto req = reader.readText();
								remap.required.insert(req);
							} else {
								logger::warn("defaultBBP({},{}) : unknown element", reader.GetRow(), reader.GetColumn());
								reader.skipCurrentElement();
							}
						} else if (reader.GetInspected() == Xml::Inspected::EndTag) {
							break;
						}
					}
					remaps.push_back(remap);
				} else {
					logger::warn("defaultBBP({},{}) : unknown element", reader.GetRow(), reader.GetColumn());
					reader.skipCurrentElement();
				}
			} else if (reader.GetInspected() == Xml::Inspected::EndTag) {
				break;
			}
		}
	}

	DefaultBBP::PhysicsFile_t DefaultBBP::scanDefaultBBP(RE::NiNode* armor)
	{
		static std::mutex s_lock;
		std::lock_guard<std::mutex> l(s_lock);

		if (bbpFileList.empty()) {
			return { { std::string("") }, {} };
		}

		auto remappedNames = DefaultBBP::instance()->getNameMap(armor);

		auto it = std::find_if(bbpFileList.begin(), bbpFileList.end(), [&](const std::pair<std::string, std::string>& e) { return remappedNames.find(e.first) != remappedNames.end(); });
		return { it == bbpFileList.end() ? "" : it->second, remappedNames };
	}

	DefaultBBP::NameMap_t DefaultBBP::getNameMap(RE::NiNode* armor)
	{
		auto nameMap = defaultNameMap(armor);

		for (auto remap : remaps) {
			bool doRemap = true;
			for (auto req : remap.required) {
				if (nameMap.find(req) == nameMap.end()) {
					doRemap = false;
				}
			}

			if (doRemap) {
				auto start = std::find_if(remap.entries.rbegin(), remap.entries.rend(), [&](const auto& e) { return nameMap.find(e.second) != nameMap.end(); });
				auto end = std::find_if(start, remap.entries.rend(), [&](const auto& e) { return e.first != start->first; });
				if (start != remap.entries.rend()) {
					auto&& s = nameMap.insert({ remap.name, {} }).first;
					std::for_each(start, end, [&](const auto& e) {
						auto it = nameMap.find(e.second);
						if (it != nameMap.end()) {
							std::for_each(it->second.begin(), it->second.end(), [&](const std::string& name) {
								s->second.insert(name);
							});
						}
					});
				}
			}
		}
		return nameMap;
	}

	DefaultBBP::NameMap_t DefaultBBP::defaultNameMap(RE::NiNode* armor)
	{
		std::unordered_map<std::string, std::unordered_set<std::string>> nameMap;

		// This case never happens to a lurker skeleton, thus we don't need to test.
		auto skinned = findNode(armor, "BSFaceGenNiNodeSkinned");
		if (skinned) {
			const auto& skinnedNodechildren = skinned->GetChildren();
			for (uint16_t i = 0; i < skinnedNodechildren.size(); ++i) {
				if (!skinnedNodechildren[i]) {
					continue;
				}

				auto tri = skinnedNodechildren[i]->AsTriShape();
				if (!tri || !tri->name.size()) {
					continue;
				}

				nameMap.emplace(tri->name.c_str(), std::unordered_set<std::string>{ std::string(tri->name.c_str()) });
			}
		}

		const auto& armorNodechildren = armor->GetChildren();
		for (uint16_t i = 0; i < armorNodechildren.size(); ++i) {
			if (!armorNodechildren[i]) {
				continue;
			}

			auto tri = armorNodechildren[i]->AsTriShape();
			if (!tri || !tri->name.size()) {
				continue;
			}

			nameMap.emplace(tri->name.c_str(), std::unordered_set<std::string>{ std::string(tri->name.c_str()) });
		}

		return nameMap;
	}
}
