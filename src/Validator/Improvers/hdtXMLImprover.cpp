#include "hdtXMLImprover.h"

#include "../../NetImmerseUtils.h"
#include "../Utils/hdtStringUtils.h"
#include "../Utils/hdtTemplateDefaults.h"
#include "../Utils/hdtValidatorFamily.h"
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
	namespace
	{
		// ── Constants ─────────────────────────────────────────────────────────

		constexpr const char*  kSystemRootTag       = "system";
		constexpr double       kZeroMassThreshold   = 1e-12;

		// xml-model PI and namespace attributes written to every improved file.
		constexpr const char* kSchematronPIValue =
			R"(href="https://raw.githubusercontent.com/DaymareOn/FSMP-Validator/refs/heads/main/skse/plugins/hdtSkinnedMeshConfigs/hdtSMP64.sch" type="application/xml" schematypens="http://purl.oclc.org/dsdl/schematron")";
		constexpr const char* kXmlnsXsi          = "http://www.w3.org/2001/XMLSchema-instance";
		constexpr const char* kXmlnsValue        = "FSMP-Validator";
		constexpr const char* kSchemaLocation    =
			"FSMP-Validator https://raw.githubusercontent.com/DaymareOn/FSMP-Validator/main/skse/plugins/hdtSkinnedMeshConfigs/hdtSMP64.xsd";

		using ChildSet = std::unordered_set<std::string>;
		using ChildMap = std::unordered_map<std::string, ChildSet>;

		// ── Template name helpers ─────────────────────────────────────────────

		const char* generatedTemplatePrefix(Family family)
		{
			switch (family) {
			case Family::Bone:        return "bone-default-";
			case Family::PerTriangle: return "per-triangle-default-";
			case Family::PerVertex:   return "per-vertex-default-";
			case Family::Generic:     return "generic-default-";
			case Family::StiffSpring: return "stiff-spring-default-";
			case Family::ConeTwist:   return "cone-twist-default-";
			default:                  return "default-";
			}
		}

		// ── DOM helpers ───────────────────────────────────────────────────────

		// Normalize deprecated element names in-place so legacy XML survives
		// cleanup and is emitted with canonical tag names.
		bool renameDeprecatedTags(pugi::xml_node node)
		{
			bool changed = false;
			for (auto child = node.first_child(); child; child = child.next_sibling()) {
				if (child.type() != pugi::node_element)
					continue;
				if (std::string(child.name()) == "prenetration") {
					child.set_name("penetration");
					changed = true;
				}
				changed = renameDeprecatedTags(child) || changed;
			}
			return changed;
		}

		// Recursively removes child elements that are unknown (not in the schema)
		// or not allowed by the schema content model for their parent.
		// The order of remaining siblings is preserved.
		bool cleanNode(
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

				const std::string childName = child.name();

				if (!known.count(childName)) {
					toRemove.push_back(child);
					continue;
				}

				if (!parentName.empty()) {
					auto it = allowed.find(parentName);
					if (it != allowed.end()) {
						if (!it->second.count(childName)) {
							toRemove.push_back(child);
							continue;
						}
					} else {
						// parent is a leaf element — no children allowed
						toRemove.push_back(child);
						continue;
					}
				}

				changed = cleanNode(child, childName, allowed, known) || changed;
			}

			for (auto& n : toRemove)
				node.remove_child(n);

			return changed || !toRemove.empty();
		}

		bool hasNoAttributes(pugi::xml_node node)
		{
			return !node.first_attribute();
		}

		bool hasAnonymousName(pugi::xml_node node)
		{
			auto nameAttr = node.attribute("name");
			if (!nameAttr)
				return true;
			return TrimAsciiWhitespace(nameAttr.as_string()).empty();
		}

		bool hasZeroMassChild(pugi::xml_node node)
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
					return std::abs(std::stod(value)) < kZeroMassThreshold;
				} catch (...) {
					return false;
				}
			}
			return false;
		}

		// ── Improvement passes ────────────────────────────────────────────────

		bool assignKineticNamesToAnonymousZeroMassNodes(pugi::xml_node systemNode)
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

		bool mergeAdjacentDefaultNodes(pugi::xml_node systemNode)
		{
			bool changed = false;
			pugi::xml_node previousDefault;
			std::string previousName;

			for (auto node = systemNode.first_child(); node; ) {
				auto next = node.next_sibling();
				if (node.type() == pugi::node_element) {
					const std::string currentName = node.name();
					if (isDefaultNodeName(currentName) && hasNoAttributes(node)) {
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

		bool removeUselessCollisionFilters(pugi::xml_node node)
		{
			bool changed = false;

			for (auto child = node.first_child(); child; child = child.next_sibling()) {
				if (child.type() != pugi::node_element)
					continue;
				changed = removeUselessCollisionFilters(child) || changed;
			}

			bool hasCanCollideWithTag  = false;
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
				if (hasCanCollideWithTag && childName == "no-collide-with-tag")
					toRemove.push_back(child);
				else if (hasCanCollideWithBone && childName == "no-collide-with-bone")
					toRemove.push_back(child);
			}

			for (auto& child : toRemove)
				node.remove_child(child);

			return changed || !toRemove.empty();
		}

		// Remove top-level default nodes whose only attributes are "name" and/or
		// "extends" and that have no child elements — they carry no parameters.
		bool removeEmptyDefaultNodes(pugi::xml_node systemNode)
		{
			std::vector<pugi::xml_node> toRemove;

			for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
				if (node.type() != pugi::node_element)
					continue;
				if (!isDefaultNodeName(node.name()))
					continue;

				bool hasNonStructuralAttr = false;
				for (auto attr = node.first_attribute(); attr; attr = attr.next_attribute()) {
					const std::string attrName = attr.name();
					if (attrName != "name" && attrName != "extends") {
						hasNonStructuralAttr = true;
						break;
					}
				}

				if (!hasNonStructuralAttr && !node.first_child())
					toRemove.push_back(node);
			}

			for (auto& n : toRemove)
				systemNode.remove_child(n);

			return !toRemove.empty();
		}

		// Stateless mode: pins all concrete nodes to explicit template names so
		// behaviour no longer depends on document order for unnamed defaults.
		bool convertAnonymousDefaultsToStatelessTemplates(pugi::xml_node systemNode)
		{
			bool changed = false;

			std::unordered_set<std::string> usedTemplateNames;
			for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
				if (node.type() != pugi::node_element)
					continue;
				if (!isDefaultNodeName(node.name()))
					continue;
				const std::string name = TrimAsciiWhitespace(node.attribute("name").as_string());
				if (!name.empty())
					usedTemplateNames.insert(name);
			}

			auto makeUniqueTemplateName = [&](Family family) {
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

			std::unordered_map<Family, std::string> currentImplicitTemplateByFamily;
			for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
				if (node.type() != pugi::node_element)
					continue;

				const std::string localName = node.name();
				const Family family = familyForNode(localName);
				if (family == Family::None)
					continue;

				if (isDefaultNodeName(localName)) {
					auto nameAttr    = node.attribute("name");
					auto extendsAttr = node.attribute("extends");
					const std::string existingName    = TrimAsciiWhitespace(nameAttr.as_string());
					const std::string existingExtends = TrimAsciiWhitespace(extendsAttr.as_string());

					if (existingName.empty()) {
						const std::string previousTemplate = currentImplicitTemplateByFamily[family];
						const std::string generatedName    = makeUniqueTemplateName(family);

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
				if (!TrimAsciiWhitespace(templateAttr.as_string()).empty())
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

		// After stateless conversion: skip intermediate templates whose parameters
		// are fully covered by their child, shortening the inheritance chain.
		// Iterates until no further chain compression is possible.
		bool shortenTemplateInheritanceChains(pugi::xml_node systemNode)
		{
			bool changed        = false;
			bool iterationChanged = true;

			while (iterationChanged) {
				iterationChanged = false;

				std::unordered_map<std::string, std::unordered_set<std::string>> paramsByTemplate;
				std::unordered_map<std::string, std::string> extendsChain;

				for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
					if (node.type() != pugi::node_element || !isDefaultNodeName(node.name()))
						continue;

					const std::string name       = TrimAsciiWhitespace(node.attribute("name").as_string());
					const std::string parentName = TrimAsciiWhitespace(node.attribute("extends").as_string());

					if (name.empty())
						continue;

					for (auto child = node.first_child(); child; child = child.next_sibling()) {
						if (child.type() == pugi::node_element)
							paramsByTemplate[name].insert(child.name());
					}
					if (!parentName.empty())
						extendsChain[name] = parentName;
				}

				std::unordered_map<std::string, std::string> pendingUpdates;
				for (const auto& [childName, parentName] : extendsChain) {
					if (!extendsChain.count(parentName))
						continue;

					const auto& childParams  = paramsByTemplate[childName];
					const auto& parentParams = paramsByTemplate[parentName];

					bool coversAll = true;
					for (const auto& param : parentParams) {
						if (!childParams.count(param)) { coversAll = false; break; }
					}

					if (coversAll && !parentParams.empty()) {
						pendingUpdates[childName] = extendsChain.at(parentName);
						iterationChanged = true;
					}
				}
				for (auto& [k, v] : pendingUpdates)
					extendsChain[k] = v;

				for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
					if (node.type() != pugi::node_element || !isDefaultNodeName(node.name()))
						continue;

					const std::string name = TrimAsciiWhitespace(node.attribute("name").as_string());
					if (!extendsChain.count(name))
						continue;

					auto extendsAttr = node.attribute("extends");
					const std::string oldExtends = TrimAsciiWhitespace(extendsAttr.as_string());
					const std::string newExtends = extendsChain[name];

					if (oldExtends != newExtends) {
						if (extendsAttr)
							extendsAttr.set_value(newExtends.c_str());
						else if (!newExtends.empty())
							node.append_attribute("extends") = newExtends.c_str();
						iterationChanged = true;
						changed = true;
					}
				}
			}

			return changed;
		}

		// After chain-shortening: remove named default nodes that are duplicates of
		// another (identical effective field set), redirecting all references to the
		// canonical copy.
		bool deduplicateEquivalentTemplates(pugi::xml_document& doc, pugi::xml_node systemNode)
		{
			const auto equivalentAliases = CollectEquivalentDefaultTemplateAliases(doc);
			if (equivalentAliases.empty())
				return false;

			bool changed = false;

			for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
				if (node.type() != pugi::node_element)
					continue;

				const std::string localName = node.name();
				if (isDefaultNodeName(localName)) {
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

			std::vector<pugi::xml_node> toRemove;
			for (auto node = systemNode.first_child(); node; node = node.next_sibling()) {
				if (node.type() != pugi::node_element || !isDefaultNodeName(node.name()))
					continue;
				if (equivalentAliases.count(TrimAsciiWhitespace(node.attribute("name").as_string())))
					toRemove.push_back(node);
			}
			for (const auto& n : toRemove) {
				systemNode.remove_child(n);
				changed = true;
			}

			return changed;
		}

		// ── Orchestration helpers ─────────────────────────────────────────────

		// Run all improvement passes in order. Returns true when at least one pass
		// made a change.
		bool applyImprovementPasses(pugi::xml_document& doc, pugi::xml_node sysNode, bool stateless)
		{
			const ChildMap& allowed = GetSchemaAllowedChildren();
			const ChildSet& known   = GetSchemaKnownElements();

			bool changed = renameDeprecatedTags(sysNode);
			changed = cleanNode(sysNode, kSystemRootTag, allowed, known)  || changed;
			changed = RemoveTemplateRedundantChildren(doc)                 || changed;
			changed = removeUselessCollisionFilters(sysNode)               || changed;
			changed = RemoveUnusedDefaultNodes(doc)                        || changed;

			if (stateless) {
				changed = convertAnonymousDefaultsToStatelessTemplates(sysNode) || changed;
				changed = shortenTemplateInheritanceChains(sysNode)             || changed;
				changed = deduplicateEquivalentTemplates(doc, sysNode)          || changed;
			}

			changed = removeEmptyDefaultNodes(sysNode)                     || changed;
			changed = mergeAdjacentDefaultNodes(sysNode)                   || changed;
			changed = assignKineticNamesToAnonymousZeroMassNodes(sysNode)  || changed;

			return changed;
		}

		// Annotate the document with schema references, then serialize and write to
		// <outputDir>/<relative-from-data>. Also writes a sibling .json file.
		bool writeImprovedXML(
			pugi::xml_document& doc,
			pugi::xml_node sysNode,
			const std::string& srcXMLPath,
			const std::string& outputDir,
			bool copyOriginal)
		{
			namespace fs = std::filesystem;

			// Add xml-model PI pointing to the Schematron schema (skip if already present).
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
				pi.set_value(kSchematronPIValue);
			}

			// Replace system element namespace and schema location attributes.
			sysNode.remove_attribute("xsi:schemaLocation");
			sysNode.remove_attribute("xsi:noNamespaceSchemaLocation");
			sysNode.remove_attribute("xmlns:xsi");
			sysNode.remove_attribute("xmlns");
			sysNode.append_attribute("xmlns:xsi")         = kXmlnsXsi;
			sysNode.append_attribute("xmlns")             = kXmlnsValue;
			sysNode.append_attribute("xsi:schemaLocation") = kSchemaLocation;

			// Output path: <outputDir>/<relative-from-data>
			fs::path outPath = fs::path(outputDir) / stripDataPrefix(srcXMLPath);

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

			WriteXmlDocumentAsJsonFile(doc, fs::path(outPath).replace_extension(".json"));

			if (copyOriginal) {
				fs::path originalOutPath = outPath;
				const std::string ext    = originalOutPath.extension().string();
				originalOutPath.replace_filename(originalOutPath.stem().string() + "-original" + ext);
				fs::copy_file(fs::u8path(srcXMLPath), originalOutPath,
					fs::copy_options::overwrite_existing, ec);
				if (ec)
					return false;
			}

			return true;
		}

	}  // namespace

	// ── Public API ────────────────────────────────────────────────────────────

	// Load srcXMLPath, apply improvement passes in order, and write the result
	// to <outputDir>/<relative-from-data>. Returns false when nothing changed.
	//
	// Passes: (1) rename deprecated tags → (2) remove unknown/disallowed elements →
	// (3) remove redundant template children → (4) drop useless collision filters →
	// (5) remove unused default nodes → (6) remove empty defaults →
	// (7) merge adjacent anonymous defaults → (8) assign kinetic names to zero-mass nodes.
	// In stateless mode, three extra passes run between (5) and (6): pin anonymous
	// defaults to explicit template names, shorten inheritance chains, then
	// deduplicate templates with identical effective field sets.
	bool GenerateImprovedXML(
		const std::string& srcXMLPath,
		const std::string& outputDir,
		bool copyOriginal,
		bool stateless)
	{
		pugi::xml_document doc;
		std::string srcBytes = readAllFile2(srcXMLPath.c_str());
		if (srcBytes.empty())
			return false;
		if (!doc.load_buffer(srcBytes.data(), srcBytes.size()))
			return false;

		pugi::xml_node sysNode;
		for (auto child = doc.first_child(); child; child = child.next_sibling()) {
			if (child.type() != pugi::node_element)
				continue;
			if (std::string(child.name()) != kSystemRootTag)
				return false;  // unexpected root — skip
			sysNode = child;
			break;
		}
		if (!sysNode)
			return false;

		if (!applyImprovementPasses(doc, sysNode, stateless))
			return false;

		return writeImprovedXML(doc, sysNode, srcXMLPath, outputDir, copyOriginal);
	}

}  // namespace hdt
