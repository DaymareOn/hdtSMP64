#include "hdtXMLImprover.h"

#include "hdtXSDValidator.h"

#include <pugixml.hpp>

#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdt
{
	using ChildSet = std::unordered_set<std::string>;
	using ChildMap = std::unordered_map<std::string, ChildSet>;

	// ---- DOM walker ----

	// Recursively removes child elements that are either:
	//   (a) not in knownElements — completely unknown tag, or
	//   (b) not in the allowed children set for their parent.
	// Returns true if any element was removed at this level or below.
	static bool cleanNode(
		pugi::xml_node     node,
		const std::string& parentName,
		const ChildMap&    allowed,
		const ChildSet&    known)
	{
		std::vector<pugi::xml_node> toRemove;

		for (auto child = node.first_child(); child; child = child.next_sibling())
		{
			if (child.type() != pugi::node_element)
				continue;

			std::string childName = child.name();

			// (a) completely unknown tag
			if (!known.count(childName))
			{
				toRemove.push_back(child);
				continue;
			}

			// (b) not allowed in this parent
			if (!parentName.empty())
			{
				auto it = allowed.find(parentName);
				if (it != allowed.end())
				{
					if (!it->second.count(childName))
					{
						toRemove.push_back(child);
						continue;
					}
				}
				else
				{
					// parent is a leaf element → no children allowed
					toRemove.push_back(child);
					continue;
				}
			}

			// Valid child — recurse
			cleanNode(child, childName, allowed, known);
		}

		for (auto& n : toRemove)
			node.remove_child(n);

		return !toRemove.empty();
	}

	// ---- Path helpers ----

	// Strip the leading "data/" prefix (case-insensitive, forward slashes).
	static std::string stripDataPrefix(const std::string& path)
	{
		std::string norm = path;
		std::replace(norm.begin(), norm.end(), '\\', '/');

		std::string lower = norm;
		std::transform(lower.begin(), lower.end(), lower.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });

		if (lower.size() >= 5 && lower.substr(0, 5) == "data/")
			return norm.substr(5);
		return norm;
	}

	// ---- Public entry point ----

	bool GenerateImprovedXML(const std::string& srcXMLPath, const std::string& outputDir)
	{
		namespace fs = std::filesystem;

		if (!IsPhysicsSchemaLoaded())
			return false;

		const ChildMap& allowed = GetSchemaAllowedChildren();
		const ChildSet& known   = GetSchemaKnownElements();

		pugi::xml_document doc;
		if (!doc.load_file(srcXMLPath.c_str()))
			return false;

		// Locate the root <system> element
		pugi::xml_node sysNode;
		for (auto child = doc.first_child(); child; child = child.next_sibling())
		{
			if (child.type() == pugi::node_element)
			{
				if (std::string(child.name()) != "system")
					return false; // unexpected root — skip
				sysNode = child;
				break;
			}
		}
		if (!sysNode)
			return false;

		// Clean the tree using schema data from the XSD validator
		bool changed = cleanNode(sysNode, "system", allowed, known);
		if (!changed)
			return false;

		// Compute output path: <outputDir>/<relative-from-data>
		std::string relative = stripDataPrefix(srcXMLPath);
		fs::path    outPath  = fs::path(outputDir) / relative;

		std::error_code ec;
		fs::create_directories(outPath.parent_path(), ec);
		if (ec)
			return false;

		return doc.save_file(outPath.string().c_str(), "  ", pugi::format_default);
	}

} // namespace hdt
