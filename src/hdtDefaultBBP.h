#pragma once

namespace hdt
{
	class DefaultBBP
	{
	public:
		using RemapEntry_t = std::pair<int, std::string>;
		using NameSet_t = std::unordered_set<std::string>;
		using NameMap_t = std::unordered_map<std::string, NameSet_t>;
		using PhysicsFile_t = std::pair<std::string, NameMap_t>;

		struct Remap
		{
			std::string name;
			std::set<RemapEntry_t> entries;
			std::unordered_set<std::string> required;
		};

		static DefaultBBP* instance();
		PhysicsFile_t scanBBP(RE::NiNode* scan);

		// Loads one "default-bbps" mapping file and merges its entries into the shape->file
		// list and bone remaps. Public so optional extra mappings (e.g. obstructionBBPs.xml,
		// used by the experimental world-collision feature) can be merged on top of the defaults.
		void loadBBP(const char* path);

	private:
		DefaultBBP();

		std::unordered_map<std::string, std::string> bbpFileList;
		std::vector<Remap> remaps;

		void loadDefaultBBPs();
		PhysicsFile_t scanDefaultBBP(RE::NiNode* scan);
		NameMap_t getNameMap(RE::NiNode* armor);
		NameMap_t defaultNameMap(RE::NiNode* armor);
	};
}
