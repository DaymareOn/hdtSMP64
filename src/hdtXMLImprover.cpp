#include "hdtXMLImprover.h"

#include <pugixml.hpp>

#include <algorithm>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdt
{
	static const char* kPhysicsXSDPath =
		"data/skse/plugins/hdtSkinnedMeshConfigs/hdtSMP64.xsd";

	using ChildSet = std::unordered_set<std::string>;
	using ChildMap = std::unordered_map<std::string, ChildSet>;

	// ---- XSD schema model ----

	struct XSDSchema
	{
		ChildMap allowedChildren; // parent element name → set of allowed child element names
		ChildSet knownElements;   // all element names mentioned anywhere in the schema
		bool     loaded = false;
	};

	// Recursively collects child element names from XSD compositor nodes
	// (xsd:choice, xsd:sequence, xsd:all) and xsd:element nodes within them.
	// ref= and name= values are inserted into `children` (allowed set for the
	// current parent) and into `allKnown` (the global known-elements set).
	// Inline elements that carry their own complexType are recursed into so that
	// their locally-defined child names also land in `allKnown`.
	static void collectElementChildren(
		pugi::xml_node node,
		ChildSet&      children,
		ChildSet&      allKnown)
	{
		for (auto child = node.first_child(); child; child = child.next_sibling())
		{
			std::string tag = child.name();

			if (tag == "xsd:element")
			{
				const char* ref  = child.attribute("ref").as_string("");
				const char* name = child.attribute("name").as_string("");
				if (ref[0])
				{
					children.insert(ref);
					allKnown.insert(ref);
				}
				else if (name[0])
				{
					children.insert(name);
					allKnown.insert(name);
					// If this inline element owns a complexType, gather its
					// element children into allKnown only — they belong to
					// the inline element, not to the current parent.
					for (auto inner = child.first_child(); inner; inner = inner.next_sibling())
					{
						if (std::string(inner.name()) == "xsd:complexType")
						{
							ChildSet innerChildren; // intentionally discarded
							collectElementChildren(inner, innerChildren, allKnown);
						}
					}
				}
			}
			else if (tag == "xsd:choice" || tag == "xsd:sequence" || tag == "xsd:all")
			{
				collectElementChildren(child, children, allKnown);
			}
			// xsd:annotation, xsd:attribute, xsd:simpleContent, etc. — ignored
		}
	}

	static XSDSchema parseXSD()
	{
		XSDSchema schema;

		pugi::xml_document doc;
		if (!doc.load_file(kPhysicsXSDPath))
			return schema;

		pugi::xml_node schemaRoot = doc.first_child(); // xsd:schema

		// Phase 1 — named complexTypes (e.g. "transform" → {basis, basis-axis-angle, origin}).
		// These define the allowed children for elements that reference them via type="...".
		std::unordered_map<std::string, ChildSet> typeChildren;
		for (auto node = schemaRoot.first_child(); node; node = node.next_sibling())
		{
			if (std::string(node.name()) != "xsd:complexType")
				continue;
			std::string typeName = node.attribute("name").as_string("");
			if (typeName.empty())
				continue;

			ChildSet children;
			collectElementChildren(node, children, schema.knownElements);
			if (!children.empty())
				typeChildren[typeName] = std::move(children);
		}

		// Phase 2 — global xsd:element declarations.
		for (auto node = schemaRoot.first_child(); node; node = node.next_sibling())
		{
			if (std::string(node.name()) != "xsd:element")
				continue;
			std::string elemName = node.attribute("name").as_string("");
			if (elemName.empty())
				continue;

			schema.knownElements.insert(elemName);

			// Look for an inline xsd:complexType child.
			pugi::xml_node ct;
			for (auto child = node.first_child(); child; child = child.next_sibling())
			{
				if (std::string(child.name()) == "xsd:complexType")
				{
					ct = child;
					break;
				}
			}

			if (ct)
			{
				// Element has an inline complexType — collect its element children.
				ChildSet children;
				collectElementChildren(ct, children, schema.knownElements);
				if (!children.empty())
					schema.allowedChildren[elemName] = std::move(children);
				// If children is empty (e.g. weight-threshold with simpleContent),
				// the element is a leaf: not inserted into allowedChildren.
			}
			else
			{
				// No inline complexType — resolve the named type from Phase 1.
				std::string typeName = node.attribute("type").as_string("");
				auto        it       = typeChildren.find(typeName);
				if (it != typeChildren.end())
				{
					schema.allowedChildren[elemName] = it->second;
					for (const auto& c : it->second)
						schema.knownElements.insert(c); // may already be present
				}
				// Built-in or simple types → leaf element, nothing more to do.
			}
		}

		schema.loaded = true;
		return schema;
	}

	static const XSDSchema& getXSDSchema()
	{
		static XSDSchema schema = parseXSD();
		return schema;
	}

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

		const XSDSchema& schema = getXSDSchema();
		if (!schema.loaded)
			return false;

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

		// Clean the tree using schema data parsed from the XSD file
		bool changed = cleanNode(sysNode, "system", schema.allowedChildren, schema.knownElements);
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
