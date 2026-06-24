#include "hdtNIFBoneRefValidator.h"

#include "../Utils/hdtPhysicsXmlSource.h"  // readAndExpandPhysicsXml
#include "../Utils/hdtTemplateDefaults.h"  // isDefaultNodeName
#include "../Utils/hdtValidatorFamily.h"   // familyForNode
#include "NetImmerseUtils.h"               // readAllFile2
#include "hdtNIFValidator.h"               // CollectNamedSkeletonNodes

#include <pugixml.hpp>

#include <algorithm>
#include <string>
#include <unordered_set>

namespace hdt
{
	namespace
	{
		// Mirror of SkyrimSystemCreator::getRenamedBone: a mapped name resolves to its
		// merged-skeleton form, an unmapped name is looked up verbatim.
		std::string applyRename(const std::string& name,
			const std::unordered_map<std::string, std::string>& renameMap)
		{
			auto it = renameMap.find(name);
			return it == renameMap.end() ? name : it->second;
		}

		// Per-resolved-name accumulator: how many times, and in what roles, the XML reaches
		// a node that the skeleton does not provide.
		struct MissingAcc
		{
			std::string referenced;  // first-seen written form
			bool usedAsBone = false;
			int constraintRefs = 0;
		};

		// Depth-first walk over every element, classifying bone/constraint references.
		// Recursion (rather than first-level children) is needed because constraints may be
		// nested inside <constraint-group> and bones/shapes are siblings under <system>.
		void collectReferences(pugi::xml_node node,
			const std::unordered_set<std::string>& nodeSet,
			const std::unordered_map<std::string, std::string>& renameMap,
			std::unordered_map<std::string, MissingAcc>& missingByResolved)
		{
			auto record = [&](const char* written, bool asBone) {
				if (!written || written[0] == '\0')
					return;
				std::string resolved = applyRename(written, renameMap);
				if (nodeSet.count(resolved))
					return;  // resolves fine — SMP would find it
				auto& acc = missingByResolved[resolved];
				if (acc.referenced.empty())
					acc.referenced = written;
				if (asBone)
					acc.usedAsBone = true;
				else
					++acc.constraintRefs;
			};

			for (pugi::xml_node child = node.first_child(); child; child = child.next_sibling()) {
				if (child.type() != pugi::node_element)
					continue;
				// Classify by element family, but skip "-default" template nodes: their
				// name/bodyA/bodyB carry a template class name, not a skeleton node.
				const std::string tag = child.name();
				if (!isDefaultNodeName(tag)) {
					switch (familyForNode(tag)) {
					case Family::Bone:
						record(child.attribute("name").value(), true);
						break;
					case Family::Generic:
					case Family::StiffSpring:
					case Family::ConeTwist:
						record(child.attribute("bodyA").value(), false);
						record(child.attribute("bodyB").value(), false);
						break;
					default:
						break;
					}
				}
				collectReferences(child, nodeSet, renameMap, missingByResolved);
			}
		}
	}  // namespace

	std::vector<MissingBoneRef> FindMissingPhysicsXmlBoneRefs(
		RE::NiNode* skeletonRoot,
		const std::string& xmlPath,
		const std::unordered_map<std::string, std::string>& renameMap)
	{
		// A missing/malformed XML yields no findings on purpose: reporting bad XML is the
		// schema validator's job, and it runs over these same equipped XMLs in the report.
		const PhysicsXmlSource src = readAndExpandPhysicsXml(xmlPath);
		if (!src.ok)
			return {};  // malformed patterns are reported by the XSD validator
		pugi::xml_document doc;
		if (!doc.load_buffer(src.xml.data(), src.xml.size()))
			return {};

		std::vector<std::string> nodeNames;
		CollectNamedSkeletonNodes(skeletonRoot, nodeNames);
		std::unordered_set<std::string> nodeSet(nodeNames.begin(), nodeNames.end());

		std::unordered_map<std::string, MissingAcc> missingByResolved;
		collectReferences(doc, nodeSet, renameMap, missingByResolved);

		std::vector<MissingBoneRef> missing;
		missing.reserve(missingByResolved.size());
		for (auto& [resolved, acc] : missingByResolved) {
			MissingBoneRef m;
			m.referencedName = acc.referenced;
			m.resolvedName = resolved;
			m.usedAsBone = acc.usedAsBone;
			m.constraintRefs = acc.constraintRefs;
			missing.push_back(std::move(m));
		}
		// Deterministic report ordering.
		std::sort(missing.begin(), missing.end(),
			[](const MissingBoneRef& a, const MissingBoneRef& b) { return a.resolvedName < b.resolvedName; });

		return missing;
	}
}  // namespace hdt
