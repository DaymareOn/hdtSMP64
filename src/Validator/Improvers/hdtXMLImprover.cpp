#include "hdtXMLImprover.h"

#include "../Validators/hdtXSDValidator.h"

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

	// Recursively removes child elements that are either not in knownElements (completely unknown
	// tag) or not in the allowed children set for their parent. The order of remaining sibling
	// elements is preserved in the DOM tree after removal.
	//
	// Arguments:
	//   node:       The current XML node being cleaned (and whose children will be visited)
	//   parentName: Name of the parent element (used to look up allowed children in schema)
	//   allowed:    Map of parent element names to sets of allowed child element names
	//   known:      Set of all known element names from the schema
	//
	// Returns: true if any element was removed at this level or below; false if no changes made
	static bool cleanNode(
		pugi::xml_node node,
		const std::string& parentName,
		const ChildMap& allowed,
		const ChildSet& known)
	{
		std::vector<pugi::xml_node> toRemove;
		bool changed = false;

		for (auto child = node.first_child(); child; child = child.next_sibling()) {
			if (child.type() != pugi::node_element)
				continue;

			std::string childName = child.name();

			// (a) completely unknown tag
			if (!known.count(childName)) {
				toRemove.push_back(child);
				continue;
			}

			// (b) not allowed in this parent
			if (!parentName.empty()) {
				auto it = allowed.find(parentName);
				if (it != allowed.end()) {
					if (!it->second.count(childName)) {
						toRemove.push_back(child);
						continue;
					}
				} else {
					// parent is a leaf element → no children allowed
					toRemove.push_back(child);
					continue;
				}
			}

			// Valid child — recurse
			changed = cleanNode(child, childName, allowed, known) || changed;
		}

		for (auto& n : toRemove)
			node.remove_child(n);

		return changed || !toRemove.empty();
	}

	// ---- Path helpers ----

	// Strips the leading "data/" prefix from a file path using case-insensitive matching.
	// Normalizes all backslashes to forward slashes before processing. If the path does not
	// start with "data/", the normalized path is returned unchanged.
	//
	// Arguments:
	//   path: The file path to process (may contain mixed slash types)
	//
	// Returns: The path with "data/" prefix removed (if present), or the normalized path
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

	// Generates an improved XML file by removing non-conforming elements according to the
	// physics schema. Loads the source XML document, locates the root <system> element, and
	// recursively removes any child elements that are either unknown (not defined in the schema)
	// or not allowed by the schema for their parent element. The original order of remaining
	// elements is strictly preserved in the output document.
	//
	// The output path is computed as <outputDir>/<relative-from-data>, where the relative path
	// is derived by stripping the "data/" prefix from srcXMLPath.
	//
	// Arguments:
	//   srcXMLPath: Absolute or relative path to the source XML file to clean
	//   outputDir:  Directory where the improved XML file will be written
	//
	// Returns: true if the XML was successfully cleaned and saved to disk; false on parse error,
	//          missing root element, I/O error, or if no elements were removed (document already valid)
	bool GenerateImprovedXML(const std::string& srcXMLPath, const std::string& outputDir)
	{
		namespace fs = std::filesystem;

		const ChildMap& allowed = GetSchemaAllowedChildren();
		const ChildSet& known = GetSchemaKnownElements();

		pugi::xml_document doc;
		if (!doc.load_file(srcXMLPath.c_str()))
			return false;

		// Locate the root <system> element
		pugi::xml_node sysNode;
		for (auto child = doc.first_child(); child; child = child.next_sibling()) {
			if (child.type() == pugi::node_element) {
				if (std::string(child.name()) != "system")
					return false;  // unexpected root — skip
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
		fs::path outPath = fs::path(outputDir) / relative;

		std::error_code ec;
		fs::create_directories(outPath.parent_path(), ec);
		if (ec)
			return false;

		return doc.save_file(outPath.string().c_str(), "  ", pugi::format_default);
	}

}  // namespace hdt
