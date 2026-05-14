#include "hdtXMLImprover.h"

#include "../../NetImmerseUtils.h"
#include "../Utils/hdtStringUtils.h"
#include "../Utils/hdtTemplateDefaults.h"
#include "../Utils/hdtXMLJsonWriter.h"
#include "../Validators/hdtXSDValidator.h"

#include "../../../extern/pugixml/pugixml.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdt
{
	enum class TemplateFamily
	{
		None,
		Bone,
		PerTriangle,
		PerVertex,
		Generic,
		StiffSpring,
		ConeTwist
	};

	static TemplateFamily templateFamilyForNode(const std::string& localName)
	{
		if (localName == "bone" || localName == "bone-default")
			return TemplateFamily::Bone;
		if (localName == "per-triangle-shape" || localName == "per-triangle-shape-default")
			return TemplateFamily::PerTriangle;
		if (localName == "per-vertex-shape" || localName == "per-vertex-shape-default")
			return TemplateFamily::PerVertex;
		if (localName == "generic-constraint" || localName == "generic-constraint-default")
			return TemplateFamily::Generic;
		if (localName == "stiffspring-constraint" || localName == "stiffspring-constraint-default")
			return TemplateFamily::StiffSpring;
		if (localName == "conetwist-constraint" || localName == "conetwist-constraint-default")
			return TemplateFamily::ConeTwist;
		return TemplateFamily::None;
	}

	static const char* generatedTemplatePrefix(TemplateFamily family)
	{
		switch (family) {
		case TemplateFamily::Bone:
			return "bone-default-";
		case TemplateFamily::PerTriangle:
			return "per-triangle-default-";
		case TemplateFamily::PerVertex:
			return "per-vertex-default-";
		case TemplateFamily::Generic:
			return "generic-default-";
		case TemplateFamily::StiffSpring:
			return "stiff-spring-default-";
		case TemplateFamily::ConeTwist:
			return "cone-twist-default-";
		default:
			return "default-";
		}
	}

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

	static bool hasAnonymousName(pugi::xml_node node)
	{
		auto nameAttr = node.attribute("name");
		if (!nameAttr)
			return true;
		return TrimAsciiWhitespace(nameAttr.as_string()).empty();
	}

	static bool hasZeroMassChild(pugi::xml_node node)
	{
		for (auto child = node.first_child(); child; child = child.next_sibling()) {
			if (child.type() != pugi::node_element)
				continue;
			if (std::string(child.name()) != "mass")
				continue;

			std::string value = TrimAsciiWhitespace(child.text().as_string());
			if (value.empty())
				return false;
			std::replace(value.begin(), value.end(), ',', '.');

			try {
				const double mass = std::stod(value);
				return std::abs(mass) < 1e-12;
			} catch (...) {
				return false;
			}
		}

		return false;
	}

	static bool assignKineticNamesToAnonymousZeroMassNodes(pugi::xml_node systemNode)
	{
		bool changed = false;
		std::unordered_set<std::string> usedNames;

		for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
			if (node.type() != pugi::node_element)
				continue;
			auto nameAttr = node.attribute("name");
			if (!nameAttr)
				continue;
			const std::string name = TrimAsciiWhitespace(nameAttr.as_string());
			if (!name.empty())
				usedNames.insert(name);
		}

		auto makeUniqueName = [&](const std::string& prefix) {
			std::size_t index = 1;
			for (;;) {
				const std::string candidate = prefix + "-" + std::to_string(index++);
				if (!usedNames.count(candidate)) {
					usedNames.insert(candidate);
					return candidate;
				}
			}
		};

		for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
			if (node.type() != pugi::node_element)
				continue;

			const std::string localName = node.name();
			if (localName != "bone" && localName != "bone-default")
				continue;
			if (!hasAnonymousName(node) || !hasZeroMassChild(node))
				continue;

			const std::string prefix = (localName == "bone")
				? "kinetic-zero-mass-bone"
				: "kinetic-zero-mass-template";
			const std::string generatedName = makeUniqueName(prefix);

			auto nameAttr = node.attribute("name");
			if (nameAttr)
				nameAttr.set_value(generatedName.c_str());
			else
				node.append_attribute("name") = generatedName.c_str();

			changed = true;
		}

		return changed;
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

	static bool removeUselessCollisionFilters(pugi::xml_node node)
	{
		bool changed = false;

		for (auto child = node.first_child(); child; child = child.next_sibling()) {
			if (child.type() != pugi::node_element)
				continue;
			changed = removeUselessCollisionFilters(child) || changed;
		}

		bool hasCanCollideWithTag = false;
		bool hasCanCollideWithBone = false;
		for (auto child = node.first_child(); child; child = child.next_sibling()) {
			if (child.type() != pugi::node_element)
				continue;

			const std::string childName = child.name();
			if (childName == "can-collide-with-tag")
				hasCanCollideWithTag = true;
			else if (childName == "can-collide-with-bone")
				hasCanCollideWithBone = true;
		}

		if (!hasCanCollideWithTag && !hasCanCollideWithBone)
			return changed;

		std::vector<pugi::xml_node> toRemove;
		for (auto child = node.first_child(); child; child = child.next_sibling()) {
			if (child.type() != pugi::node_element)
				continue;

			const std::string childName = child.name();
			if (hasCanCollideWithTag && childName == "no-collide-with-tag") {
				toRemove.push_back(child);
				continue;
			}
			if (hasCanCollideWithBone && childName == "no-collide-with-bone")
				toRemove.push_back(child);
		}

		for (auto& child : toRemove)
			node.remove_child(child);

		return changed || !toRemove.empty();
	}

	// After stateless conversion, consolidate template inheritance chains by skipping parents
	// whose parameters are completely covered by their children.
	// If child template C extends parent P, and C sets all parameters that P sets
	// (i.e., C covers all of P's parameters), then C should inherit from P's parent instead,
	// since P's parameters are redundant.
	static bool mergeUnusedStatelessTemplates(pugi::xml_document& doc, pugi::xml_node systemNode)
	{
		bool changed = false;
		bool iterationChanged = true;

		while (iterationChanged) {
			iterationChanged = false;

			// Collect parameter sets for each template
			std::unordered_map<std::string, std::unordered_set<std::string>> parametersByTemplate;
			std::unordered_map<std::string, std::string> extendsChain;

			for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
				if (node.type() != pugi::node_element)
					continue;

				const std::string localName = node.name();
				if (!hdt::isDefaultNodeName(localName))
					continue;

				auto nameAttr = node.attribute("name");
				auto extendsAttr = node.attribute("extends");
				const std::string name = TrimAsciiWhitespace(nameAttr.as_string());
				const std::string parentName = TrimAsciiWhitespace(extendsAttr.as_string());

				if (!name.empty()) {
					// Collect all child element names (parameters) in this template
					for (auto child = node.first_child(); child; child = child.next_sibling()) {
						if (child.type() == pugi::node_element) {
							parametersByTemplate[name].insert(child.name());
						}
					}

					if (!parentName.empty()) {
						extendsChain[name] = parentName;
					}
				}
			}

			// For each template that extends another, check if it covers all its parent's parameters.
			// If child covers all parent parameters, skip to the grandparent.
			std::unordered_map<std::string, std::string> pendingChainUpdates;
			for (const auto& [childName, parentName] : extendsChain) {
				if (!extendsChain.count(parentName)) {
					// Parent doesn't extend anything; can't skip further
					continue;
				}

				auto childParams = parametersByTemplate[childName];
				auto parentParams = parametersByTemplate[parentName];

				// Check: does child cover all parent parameters?
				bool coversAll = true;
				for (const auto& param : parentParams) {
					if (!childParams.count(param)) {
						coversAll = false;
						break;
					}
				}

				if (coversAll && !parentParams.empty()) {
					// Child covers all parent's parameters; skip parent in the chain
					pendingChainUpdates[childName] = extendsChain.at(parentName);
					iterationChanged = true;
				}
			}
			for (auto& [k, v] : pendingChainUpdates)
				extendsChain[k] = v;

			// Update XML extends attributes
			for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
				if (node.type() != pugi::node_element)
					continue;

				const std::string localName = node.name();
				if (!hdt::isDefaultNodeName(localName))
					continue;

				auto nameAttr = node.attribute("name");
				const std::string name = TrimAsciiWhitespace(nameAttr.as_string());

				if (extendsChain.count(name)) {
					auto extendsAttr = node.attribute("extends");
					const std::string oldExtends = TrimAsciiWhitespace(extendsAttr.as_string());
					const std::string newExtends = extendsChain[name];

					if (oldExtends != newExtends) {
						if (extendsAttr) {
							extendsAttr.set_value(newExtends.c_str());
						} else if (!newExtends.empty()) {
							node.append_attribute("extends") = newExtends.c_str();
						} else if (extendsAttr) {
							// Remove empty extends attribute
							node.remove_attribute(extendsAttr);
						}
						iterationChanged = true;
						changed = true;
					}
				}
			}
		}

		const auto equivalentAliases = CollectEquivalentDefaultTemplateAliases(doc);
		if (equivalentAliases.empty())
			return changed;

		for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
			if (node.type() != pugi::node_element)
				continue;

			const std::string localName = node.name();
			if (hdt::isDefaultNodeName(localName)) {
				auto extendsAttr = node.attribute("extends");
				const std::string oldExtends = TrimAsciiWhitespace(extendsAttr.as_string());
				auto aliasIt = equivalentAliases.find(oldExtends);
				if (aliasIt != equivalentAliases.end()) {
					if (extendsAttr)
						extendsAttr.set_value(aliasIt->second.c_str());
					else
						node.append_attribute("extends") = aliasIt->second.c_str();
					changed = true;
				}
			continue;
		}

			auto templateAttr = node.attribute("template");
			const std::string oldTemplate = TrimAsciiWhitespace(templateAttr.as_string());
			auto aliasIt = equivalentAliases.find(oldTemplate);
			if (aliasIt == equivalentAliases.end())
				continue;

			if (templateAttr)
				templateAttr.set_value(aliasIt->second.c_str());
			else
				node.append_attribute("template") = aliasIt->second.c_str();
			changed = true;
		}

		std::vector<pugi::xml_node> duplicateTemplates;
		for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
			if (node.type() != pugi::node_element)
				continue;
			if (!hdt::isDefaultNodeName(node.name()))
				continue;

			const std::string name = TrimAsciiWhitespace(node.attribute("name").as_string());
			if (equivalentAliases.count(name))
				duplicateTemplates.push_back(node);
		}

		for (const auto& duplicateTemplate : duplicateTemplates) {
			systemNode.remove_child(duplicateTemplate);
			changed = true;
		}

		return changed;
	}

	// Stateless mode pins all concrete nodes to explicit template names.
	// This removes dependence on document order for implicit unnamed defaults.
	static bool convertAnonymousDefaultsToStatelessTemplates(pugi::xml_node systemNode)
	{
		bool changed = false;

		std::unordered_set<std::string> usedTemplateNames;
		for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
			if (node.type() != pugi::node_element)
				continue;
			if (!hdt::isDefaultNodeName(node.name()))
				continue;
			const std::string name = TrimAsciiWhitespace(node.attribute("name").as_string());
			if (!name.empty())
				usedTemplateNames.insert(name);
		}

		auto makeUniqueTemplateName = [&](TemplateFamily family) {
			const std::string prefix = generatedTemplatePrefix(family);
			std::size_t index = 1;
			for (;;) {
				const std::string candidate = prefix + "-" + std::to_string(index++);
				if (!usedTemplateNames.count(candidate)) {
					usedTemplateNames.insert(candidate);
					return candidate;
				}
			}
		};

		std::unordered_map<TemplateFamily, std::string> currentImplicitTemplateByFamily;
		for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
			if (node.type() != pugi::node_element)
				continue;

			const std::string localName = node.name();
			const TemplateFamily family = templateFamilyForNode(localName);
			if (family == TemplateFamily::None)
				continue;

			if (hdt::isDefaultNodeName(localName)) {
				auto nameAttr = node.attribute("name");
				auto extendsAttr = node.attribute("extends");
				const std::string existingName = TrimAsciiWhitespace(nameAttr.as_string());
				const std::string existingExtends = TrimAsciiWhitespace(extendsAttr.as_string());

				if (existingName.empty()) {
					const std::string previousTemplate = currentImplicitTemplateByFamily[family];
					const std::string generatedName = makeUniqueTemplateName(family);

					if (nameAttr)
						nameAttr.set_value(generatedName.c_str());
					else
						node.append_attribute("name") = generatedName.c_str();

					const std::string resolvedExtends = existingExtends.empty() ? previousTemplate : existingExtends;
					if (!resolvedExtends.empty()) {
						if (extendsAttr)
							extendsAttr.set_value(resolvedExtends.c_str());
						else
							node.append_attribute("extends") = resolvedExtends.c_str();
					} else if (extendsAttr) {
						node.remove_attribute(extendsAttr);
					}

					currentImplicitTemplateByFamily[family] = generatedName;
					changed = true;
				}
				continue;
			}

			auto templateAttr = node.attribute("template");
			const std::string templateName = TrimAsciiWhitespace(templateAttr.as_string());
			if (!templateName.empty())
				continue;

			auto it = currentImplicitTemplateByFamily.find(family);
			if (it == currentImplicitTemplateByFamily.end() || it->second.empty())
				continue;

			if (templateAttr)
				templateAttr.set_value(it->second.c_str());
			else
				node.append_attribute("template") = it->second.c_str();
			changed = true;
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
	bool GenerateImprovedXML(
		const std::string& srcXMLPath,
		const std::string& outputDir,
		bool copyOriginal,
		bool stateless)
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

		// Third pass: when a node has explicit can-collide lists, matching
		// no-collide lists are ignored by runtime checks and can be dropped.
		changed = removeUselessCollisionFilters(sysNode) || changed;

		// Fourth pass: remove unused top-level default nodes that no later node in
		// the same file would resolve by template name or unnamed fallback.
		changed = RemoveUnusedDefaultNodes(doc) || changed;

		if (stateless) {
			// Stateless mode: make template dependencies explicit so concrete node
			// order no longer affects inherited defaults.
			changed = convertAnonymousDefaultsToStatelessTemplates(sysNode) || changed;

			// Merge unused intermediate templates after stateless conversion.
			changed = mergeUnusedStatelessTemplates(doc, sysNode) || changed;
		}

		// Fifth pass: remove empty default tags (e.g., <bone-default/>, etc.)
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

		// Sixth pass: merge adjacent anonymous default nodes of the same kind.
		changed = mergeAdjacentDefaultNodes(sysNode) || changed;

		// Seventh pass: give deterministic names to anonymous zero-mass bone/template nodes.
		changed = assignKineticNamesToAnonymousZeroMassNodes(sysNode) || changed;

		if (!changed)
			return false;

		// Add xml-model processing instruction pointing to the Schematron schema (skip if already present)
		bool hasXmlModelPi = false;
		for (auto n = doc.first_child(); n; n = n.next_sibling()) {
			if (n.type() == pugi::node_pi && std::string(n.name()) == "xml-model") {
				hasXmlModelPi = true;
				break;
			}
		}
		if (!hasXmlModelPi) {
			auto pi = doc.prepend_child(pugi::node_pi);
			pi.set_name("xml-model");
			pi.set_value(R"(href="https://raw.githubusercontent.com/DaymareOn/FSMP-Validator/refs/heads/main/skse/plugins/hdtSkinnedMeshConfigs/hdtSMP64.sch" type="application/xml" schematypens="http://purl.oclc.org/dsdl/schematron")");
		}

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
		if (!out.good())
			return false;

		// Write JSON equivalent
		fs::path jsonPath = outPath;
		jsonPath.replace_extension(".json");
		WriteXmlDocumentAsJsonFile(doc, jsonPath);

		if (copyOriginal) {
			fs::path originalOutPath = outPath;
			const std::string extension = originalOutPath.extension().string();
			originalOutPath.replace_filename(originalOutPath.stem().string() + "-original" + extension);
			fs::copy_file(fs::u8path(srcXMLPath), originalOutPath, fs::copy_options::overwrite_existing, ec);
			if (ec)
				return false;
		}

		return true;
	}

}  // namespace hdt
