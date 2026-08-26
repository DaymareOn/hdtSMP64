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

		// Physics file embedded directly on this node via the "HDT Skinned Mesh Physics Object"
		// NiStringExtraData, with no defaultBBP name-matching involved. Returns nullopt when the node
		// carries no marker at all, and the marker's content (whose path may be empty, for malformed
		// content) when it does. The distinction matters: a present marker is AUTHORITATIVE. scanBBP
		// consults the defaultBBPs name-matching only when no marker exists — an empty marker must
		// yield no physics, never silently fall through to a defaultBBPs mapping. The baked-geometry
		// scan uses this so it only picks up meshes that explicitly opt in.
		std::optional<PhysicsFile_t> scanEmbeddedBBP(RE::NiNode* scan);

		// Default physics file for a skeleton model, matched by its NIF path (as authored in the race
		// record, e.g. "Actors\Canine\Character Assets\skeleton.nif"). This lets a mod add physics to
		// every race using that skeleton with just an XML plus a defaultBBPs.xml <skeleton> entry — no
		// edits to any mesh. Matching is case-insensitive. Returns "" when no entry matches. The
		// baked-geometry scan uses this as a fallback when an actor carries no embedded-tag outfit.
		std::string getSkeletonDefaultFile(const char* skeletonPath) const;

	private:
		DefaultBBP();

		std::unordered_map<std::string, std::string> bbpFileList;
		// Skeleton model NIF path (lowercased) -> default physics XML. See getSkeletonDefaultFile().
		std::unordered_map<std::string, std::string> skeletonFileList;
		std::vector<Remap> remaps;

		void loadDefaultBBPs();
		// Parse one <default-bbps> document (the single defaultBBPs.xml, or a defaultBBPs/ folder drop-in)
		// into the shared tables. First-wins inserts, so load order decides precedence. See loadDefaultBBPs.
		void parseDefaultBBPsDocument(std::string& xml);
		PhysicsFile_t scanDefaultBBP(RE::NiNode* scan);
		NameMap_t getNameMap(RE::NiNode* armor);
		NameMap_t defaultNameMap(RE::NiNode* armor);
	};
}
