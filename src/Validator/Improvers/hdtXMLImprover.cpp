#include "hdtXMLImprover.h"

#include "../../NetImmerseUtils.h"
#include "../Utils/hdtStringUtils.h"
#include "../Utils/hdtTemplateDefaults.h"
#include "../Validators/hdtXSDValidator.h"

#include <pugixml.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdt
{
	using ChildSet = std::unordered_set<std::string>;
	using ChildMap = std::unordered_map<std::string, ChildSet>;

	// Normalize deprecated element names in-place so legacy XML survives
	// cleanup and is emitted with canonical tag names.
	static bool renameDeprecatedTags(pugi::xml_node node)
	{
		bool changed = false;

		for (auto child = node.first_child(); child; child = child.next_sibling()) {
			if (child.type() != pugi::node_element)
				continue;

			const std::string currentName = child.name();
			if (currentName == "prenetration") {
				child.set_name("penetration");
				changed = true;
			}

			changed = renameDeprecatedTags(child) || changed;
		}

		return changed;
	}

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

	static bool hasNoAttributes(pugi::xml_node node)
	{
		return !node.first_attribute();
	}

	static bool mergeAdjacentDefaultNodes(pugi::xml_node systemNode)
	{
		bool changed = false;
		pugi::xml_node previousDefault;
		std::string previousName;

		for (auto node = systemNode.first_child(); node; ) {
			auto next = node.next_sibling();
			if (node.type() == pugi::node_element) {
				const std::string currentName = node.name();
				if (hdt::isDefaultNodeName(currentName) && hasNoAttributes(node)) {
					if (previousDefault && previousName == currentName && hasNoAttributes(previousDefault)) {
						for (auto child = node.first_child(); child; ) {
							auto childNext = child.next_sibling();
							previousDefault.append_copy(child);
							child = childNext;
						}
						systemNode.remove_child(node);
						changed = true;
					} else {
						previousDefault = node;
						previousName = currentName;
					}
				} else {
					previousDefault = {};
					previousName.clear();
				}
			}
			node = next;
		}

		return changed;
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
		std::string srcBytes = readAllFile2(srcXMLPath.c_str());
		if (srcBytes.empty())
			return false;
		if (!doc.load_buffer(srcBytes.data(), srcBytes.size()))
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

		// Pre-pass: normalize deprecated legacy element names.
		bool changed = renameDeprecatedTags(sysNode);

		// First pass: remove schema-unknown or schema-disallowed elements.
		changed = cleanNode(sysNode, "system", allowed, known) || changed;

		// Second pass: remove tags whose explicit value is redundant relative to
		// the effective inherited template at this point in document order.
		changed = RemoveTemplateRedundantChildren(doc) || changed;

		// Third pass: remove empty default tags (e.g., <bone-default/>, etc.)
		{
			std::vector<pugi::xml_node> toRemove;
			pugi::xml_node sysNode2;
			for (auto child = doc.first_child(); child; child = child.next_sibling()) {
				if (child.type() == pugi::node_element && std::string(child.name()) == "system") {
					sysNode2 = child;
					break;
				}
			}
			if (sysNode2) {
				for (auto node = sysNode2.first_child(); node; node = node.next_sibling()) {
					if (node.type() != pugi::node_element)
						continue;
					std::string localName = node.name();
					if (hdt::isDefaultNodeName(localName)) {
						bool hasAttrs = false;
						for (auto attr = node.first_attribute(); attr; attr = attr.next_attribute()) {
							if (std::string(attr.name()) != "name" && std::string(attr.name()) != "extends") {
								hasAttrs = true;
								break;
							}
						}
						if (!hasAttrs && !node.first_child())
							toRemove.push_back(node);
					}
				}
				for (auto& n : toRemove)
					sysNode2.remove_child(n);
				if (!toRemove.empty())
					changed = true;
			}
		}

		// Fourth pass: merge adjacent anonymous default nodes of the same kind.
		changed = mergeAdjacentDefaultNodes(sysNode) || changed;
		if (!changed)
			return false;

		// Add xml-model processing instruction pointing to the Schematron schema
		auto pi = doc.prepend_child(pugi::node_pi);
		pi.set_name("xml-model");
		pi.set_value(R"(href="https://raw.githubusercontent.com/DaymareOn/FSMP-Validator/refs/heads/main/skse/plugins/hdtSkinnedMeshConfigs/hdtSMP64.sch" type="application/xml" schematypens="http://purl.oclc.org/dsdl/schematron")");

		// Replace system element namespace and schema location attributes
		sysNode.remove_attribute("xsi:schemaLocation");
		sysNode.remove_attribute("xsi:noNamespaceSchemaLocation");
		sysNode.remove_attribute("xmlns:xsi");
		sysNode.remove_attribute("xmlns");
		sysNode.append_attribute("xmlns:xsi") = "http://www.w3.org/2001/XMLSchema-instance";
		sysNode.append_attribute("xmlns") = "FSMP-Validator";
		sysNode.append_attribute("xsi:schemaLocation") = "FSMP-Validator https://raw.githubusercontent.com/DaymareOn/FSMP-Validator/main/skse/plugins/hdtSkinnedMeshConfigs/hdtSMP64.xsd";

		// Compute output path: <outputDir>/<relative-from-data>
		std::string relative = stripDataPrefix(srcXMLPath);
		fs::path outPath = fs::path(outputDir) / relative;

		std::error_code ec;
		fs::create_directories(outPath.parent_path(), ec);
		if (ec)
			return false;

		std::ostringstream oss;
		doc.save(oss, "  ", pugi::format_default);
		const std::string xmlOut = oss.str();

		std::ofstream out(outPath, std::ios::binary | std::ios::trunc);
		if (!out.is_open())
			return false;
		out.write(xmlOut.data(), static_cast<std::streamsize>(xmlOut.size()));
		return out.good();
	}

}  // namespace hdt
