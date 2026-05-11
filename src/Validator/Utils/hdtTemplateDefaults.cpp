#include "hdtTemplateDefaults.h"

#include "../Config/hdtValidatorPaths.h"
#include "hdtStringUtils.h"
#include "hdtXMLUtils.h"

#include <algorithm>
#include <cctype>
#include <mutex>
#include <regex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hdt
{
	namespace
	{
		enum class Family
		{
			None,
			Bone,
			Generic,
			StiffSpring,
			ConeTwist
		};

		using FieldMap = std::unordered_map<std::string, std::string>;
		using TemplateMap = std::unordered_map<std::string, FieldMap>;

		struct AnalysisResult
		{
			std::unordered_set<std::string> locations;
			std::vector<TemplateRedundantChildInfo> infos;
			std::vector<pugi::xml_node> removableNodes;
		};

		static std::string toLowerAscii(std::string s)
		{
			std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
				return static_cast<char>(std::tolower(c));
			});
			return s;
		}

		static std::string normalizeNumericText(const std::string& raw)
		{
			std::string s = TrimAsciiWhitespace(raw);
			std::replace(s.begin(), s.end(), ',', '.');
			if (s.empty())
				return s;

			try {
				double value = std::stod(s);
				std::string out = std::to_string(value);
				while (!out.empty() && out.back() == '0')
					out.pop_back();
				if (!out.empty() && out.back() == '.')
					out.pop_back();
				if (out.empty())
					return "0";
				if (out == "-0")
					return "0";
				return out;
			} catch (...) {
				return s;
			}
		}

		static std::string normalizeBoolText(const std::string& raw)
		{
			const std::string s = toLowerAscii(TrimAsciiWhitespace(raw));
			if (s == "1" || s == "true")
				return "true";
			if (s == "0" || s == "false")
				return "false";
			return s;
		}

		static std::string normalizeAttrNumber(const pugi::xml_node& node, const char* attr)
		{
			return normalizeNumericText(node.attribute(attr).as_string());
		}

		static std::string normalizeVector3Attrs(const pugi::xml_node& node)
		{
			return normalizeAttrNumber(node, "x") + "," +
			       normalizeAttrNumber(node, "y") + "," +
			       normalizeAttrNumber(node, "z");
		}

		static std::string normalizeQuaternionAttrs(const pugi::xml_node& node)
		{
			return normalizeAttrNumber(node, "x") + "," +
			       normalizeAttrNumber(node, "y") + "," +
			       normalizeAttrNumber(node, "z") + "," +
			       normalizeAttrNumber(node, "w");
		}

		static std::string normalizeTransformValue(const pugi::xml_node& transformNode)
		{
			std::string basis = "0,0,0,1";
			std::string origin = "0,0,0";
			std::string axisAngle;
			bool hasBasis = false;
			bool hasOrigin = false;
			bool hasAxisAngle = false;

			for (auto child = transformNode.first_child(); child; child = child.next_sibling()) {
				if (child.type() != pugi::node_element)
					continue;

				const std::string name = std::string(XmlLocalName(child.name()));
				if (name == "basis") {
					hasBasis = true;
					hasAxisAngle = false;
					basis = normalizeQuaternionAttrs(child);
				} else if (name == "basis-axis-angle") {
					hasAxisAngle = true;
					hasBasis = false;
					axisAngle = normalizeAttrNumber(child, "x") + "," +
					            normalizeAttrNumber(child, "y") + "," +
					            normalizeAttrNumber(child, "z") + "," +
					            normalizeAttrNumber(child, "angle");
				} else if (name == "origin") {
					hasOrigin = true;
					origin = normalizeVector3Attrs(child);
				}
			}

			std::string key = "basis=";
			if (hasAxisAngle)
				key += "axis:" + axisAngle;
			else if (hasBasis)
				key += "quat:" + basis;
			else
				key += "quat:0,0,0,1";

			key += ";origin=";
			if (hasOrigin)
				key += origin;
			else
				key += "0,0,0";

			return key;
		}

		// Frame specification tags (transform or lerp type).
		// These control how constraint pivot frames are computed relative to body coordinate spaces.
		static bool isFrameTagName(const Family family, const std::string& localTag)
		{
			if (family != Family::Generic && family != Family::ConeTwist)
				return false;
			return localTag == "frameInA" || localTag == "frameInB" || localTag == "frameInLerp";
		}

		static std::string normalizeLerpValue(const pugi::xml_node& lerpNode)
		{
			std::string translationLerp = "0";
			std::string rotationLerp = "0";

			for (auto child = lerpNode.first_child(); child; child = child.next_sibling()) {
				if (child.type() != pugi::node_element)
					continue;

				const std::string childName = std::string(XmlLocalName(child.name()));
				if (childName == "translationLerp")
					translationLerp = normalizeNumericText(child.text().as_string());
				else if (childName == "rotationLerp")
					rotationLerp = normalizeNumericText(child.text().as_string());
			}

			return translationLerp + "," + rotationLerp;
		}

		static std::string normalizeFrameSpec(const std::string& frameTagName, const pugi::xml_node& frameNode)
		{
			static const std::string kIdentityTransform = "basis=quat:0,0,0,1;origin=0,0,0";

			if (frameTagName == "frameInA")
				return "A:" + normalizeTransformValue(frameNode);
			if (frameTagName == "frameInB")
				return "B:" + normalizeTransformValue(frameNode);

			const std::string lerp = normalizeLerpValue(frameNode);
			if (lerp == "1,1")
				return "B:" + kIdentityTransform;
			return "L:" + lerp;
		}

		static Family familyForNode(const std::string& localName)
		{
			if (localName == "bone" || localName == "bone-default")
				return Family::Bone;
			if (localName == "generic-constraint" || localName == "generic-constraint-default")
				return Family::Generic;
			if (localName == "stiffspring-constraint" || localName == "stiffspring-constraint-default")
				return Family::StiffSpring;
			if (localName == "conetwist-constraint" || localName == "conetwist-constraint-default")
				return Family::ConeTwist;
			return Family::None;
		}


	} // end anonymous namespace



		static std::string canonicalFieldForConetwist(const std::string& tag)
		{
			if (tag == "coneLimit" || tag == "limitZ" || tag == "swingSpan1")
				return "limitZ";
			if (tag == "planeLimit" || tag == "limitY" || tag == "swingSpan2")
				return "limitY";
			if (tag == "twistLimit" || tag == "limitX" || tag == "twistSpan")
				return "limitX";
			return tag;
		}

		static std::string fieldKeyFor(Family family, const std::string& localTag)
		{
			switch (family) {
			case Family::ConeTwist:
				return canonicalFieldForConetwist(localTag);
			case Family::Bone:
			case Family::Generic:
			case Family::StiffSpring:
				return localTag;
			default:
				return {};
			}
		}

		static std::string stripTypePrefix(std::string typeName)
		{
			auto colon = typeName.find(':');
			if (colon != std::string::npos)
				typeName.erase(0, colon + 1);
			return typeName;
		}

		static const std::unordered_map<std::string, std::string>& getElementTypeByNameFromXsd()
		{
			static std::once_flag once;
			static std::unordered_map<std::string, std::string> out;

			std::call_once(once, []() {
				pugi::xml_document doc;
				auto loadResult = doc.load_file(kPhysicsXSDPath);
				if (!loadResult)
					return;

				auto schema = doc.child("xsd:schema");
				if (!schema)
					schema = doc.child("schema");
				if (!schema)
					return;

				for (auto element : schema.children()) {
					if (std::string(XmlLocalName(element.name())) != "element")
						continue;

					std::string name = element.attribute("name").as_string();
					std::string type = stripTypePrefix(element.attribute("type").as_string());
					if (!name.empty() && !type.empty())
						out[name] = type;
				}
			});

			return out;
		}

		struct SchDefaults
		{
			std::unordered_map<Family, FieldMap> byFamily;
			FieldMap global;
		};

		static std::vector<Family> parseFamiliesFromContext(const std::string& context)
		{
			std::vector<Family> out;
			if (context.find("bone-default") != std::string::npos || context.find("bone") != std::string::npos)
				out.push_back(Family::Bone);
			if (context.find("generic-constraint-default") != std::string::npos || context.find("generic-constraint") != std::string::npos)
				out.push_back(Family::Generic);
			if (context.find("stiffspring-constraint-default") != std::string::npos || context.find("stiffspring-constraint") != std::string::npos)
				out.push_back(Family::StiffSpring);
			if (context.find("conetwist-constraint-default") != std::string::npos || context.find("conetwist-constraint") != std::string::npos)
				out.push_back(Family::ConeTwist);
			return out;
		}

		static std::unordered_map<std::string, std::string> parseFieldDefaultsFromContext(const std::string& context)
		{
			std::unordered_map<std::string, std::string> out;

			// bool default form: normalize-space(.) = 'false' or normalize-space(.) = '0'
			static const std::regex boolRule(R"(f:([A-Za-z0-9_\-]+)\[\s*normalize-space\(\.\)\s*=\s*'([^']+)')");
			for (auto it = std::sregex_iterator(context.begin(), context.end(), boolRule); it != std::sregex_iterator(); ++it) {
				std::string tag = (*it)[1].str();
				std::string value = normalizeBoolText((*it)[2].str());
				out[tag] = value;
			}

			// vector3 default form: number(@x)=... and number(@y)=... and number(@z)=...
			static const std::regex vecRule(R"(f:([A-Za-z0-9_\-]+)\[\s*number\(@x\)\s*=\s*([-0-9.]+)\s*and\s*number\(@y\)\s*=\s*([-0-9.]+)\s*and\s*number\(@z\)\s*=\s*([-0-9.]+))");
			for (auto it = std::sregex_iterator(context.begin(), context.end(), vecRule); it != std::sregex_iterator(); ++it) {
				std::string tag = (*it)[1].str();
				std::string value = normalizeNumericText((*it)[2].str()) + "," +
				                    normalizeNumericText((*it)[3].str()) + "," +
				                    normalizeNumericText((*it)[4].str());
				out[tag] = value;
			}

			// scalar default form: number(.) = <number>
			static const std::regex scalarRule(R"(f:([A-Za-z0-9_\-]+)\[\s*number\(\.\)\s*=\s*([-0-9.]+))");
			for (auto it = std::sregex_iterator(context.begin(), context.end(), scalarRule); it != std::sregex_iterator(); ++it) {
				std::string tag = (*it)[1].str();
				if (out.find(tag) != out.end())
					continue;
				out[tag] = normalizeNumericText((*it)[2].str());
			}

			return out;
		}

		static const SchDefaults& getDefaultsFromSchematron()
		{
			static std::once_flag once;
			static SchDefaults defaults;

			std::call_once(once, []() {
				pugi::xml_document doc;
				auto loadResult = doc.load_file(kPhysicsSCHPath);
				if (!loadResult)
					return;

				auto schemaRoot = doc.first_child();
				if (!schemaRoot)
					return;

				for (auto pattern : schemaRoot.children()) {
					if (std::string(XmlLocalName(pattern.name())) != "pattern")
						continue;
					if (std::string(pattern.attribute("id").as_string()) != "redundant-default-values")
						continue;

					for (auto rule : pattern.children()) {
						if (std::string(XmlLocalName(rule.name())) != "rule")
							continue;

						const std::string context = rule.attribute("context").as_string();
						auto fields = parseFieldDefaultsFromContext(context);
						if (fields.empty())
							continue;

						const auto families = parseFamiliesFromContext(context);
						if (families.empty()) {
							for (auto& [k, v] : fields)
								defaults.global[k] = v;
							continue;
						}

						for (auto family : families) {
							for (const auto& kv : fields) {
								std::string k = kv.first;
								const std::string& v = kv.second;
								if (family == Family::ConeTwist)
									k = canonicalFieldForConetwist(k);
								defaults.byFamily[family][k] = v;
							}
						}
					}
				}
			});

			return defaults;
		}

		static std::string normalizedValueForField(const std::string& field, const pugi::xml_node& valueNode)
		{
			const auto& typeByName = getElementTypeByNameFromXsd();
			auto typeIt = typeByName.find(field);
			const std::string type = (typeIt != typeByName.end()) ? typeIt->second : std::string();

			if (field == "centerOfMassTransform" || type == "transform")
				return normalizeTransformValue(valueNode);
			if (type == "lerp")
				return normalizeLerpValue(valueNode);
			if (type == "boolean")
				return normalizeBoolText(valueNode.text().as_string());
			if (type == "vector3")
				return normalizeVector3Attrs(valueNode);
			return normalizeNumericText(valueNode.text().as_string());
		}

		static std::string makeTransformDefaultKey()
		{
			return "basis=quat:0,0,0,1;origin=0,0,0";
		}

		static std::unordered_map<Family, FieldMap> makeBaseDefaults()
		{
			std::unordered_map<Family, FieldMap> defaults;

			const auto& schDefaults = getDefaultsFromSchematron();
			defaults = schDefaults.byFamily;

			for (auto family : { Family::Bone, Family::Generic, Family::StiffSpring, Family::ConeTwist }) {
				for (const auto& [k, v] : schDefaults.global) {
					std::string field = (family == Family::ConeTwist) ? canonicalFieldForConetwist(k) : k;
					defaults[family].insert_or_assign(field, v);
				}
			}

			// Fallback defaults that are not currently encoded in SCH literals.
			defaults[Family::Bone].insert_or_assign("centerOfMassTransform", makeTransformDefaultKey());
			// Runtime default is FrameInB; frameInA is an explicit body-A-space choice and is only
			// tracked for redundancy when it is repeated in template inheritance.
			defaults[Family::Generic].insert_or_assign("__frameSpec", "B:" + makeTransformDefaultKey());
			defaults[Family::Generic].insert_or_assign("frameInB", makeTransformDefaultKey());
			defaults[Family::Generic].insert_or_assign("linearDamping", "0,0,0");
			defaults[Family::Generic].insert_or_assign("angularDamping", "0,0,0");
			defaults[Family::ConeTwist].insert_or_assign("__frameSpec", "B:" + makeTransformDefaultKey());
			defaults[Family::ConeTwist].insert_or_assign("frameInB", makeTransformDefaultKey());

			return defaults;
		}

		static FieldMap getEffectiveTemplate(const TemplateMap& templates, const std::string& name)
		{
			auto it = templates.find(name);
			if (it != templates.end())
				return it->second;

			it = templates.find("");
			if (it != templates.end())
				return it->second;

			return {};
		}

		static pugi::xml_node findSystemNode(const pugi::xml_document& doc)
		{
			for (auto child = doc.first_child(); child; child = child.next_sibling()) {
				if (child.type() != pugi::node_element)
					continue;
				if (std::string(XmlLocalName(child.name())) == "system")
					return child;
			}
			return {};
		}

		static AnalysisResult analyzeTemplateRedundantChildren(
			const pugi::xml_document& doc,
			const bool collectRemovals,
			const std::string* sourceBytes)
		{
			AnalysisResult result;
			auto sysNode = findSystemNode(doc);
			if (!sysNode)
				return result;

			const auto baseDefaults = makeBaseDefaults();
			std::unordered_map<Family, TemplateMap> familyTemplates;
			for (const auto& [family, fields] : baseDefaults)
				familyTemplates[family][""] = fields;

			for (auto node = sysNode.first_child(); node; node = node.next_sibling()) {
				if (node.type() != pugi::node_element)
					continue;

				const std::string localName = std::string(XmlLocalName(node.name()));
				const Family family = familyForNode(localName);
				if (family == Family::None)
					continue;

				std::string templateName;
				FieldMap effective;

				if (isDefaultNodeName(localName)) {
					const std::string extendsName = node.attribute("extends").as_string();
					effective = getEffectiveTemplate(familyTemplates[family], extendsName);
					templateName = node.attribute("name").as_string();
				} else {
					templateName = node.attribute("template").as_string();
					effective = getEffectiveTemplate(familyTemplates[family], templateName);
				}

				pugi::xml_node lastFrameTag;
				std::string lastFrameTagName;
				for (auto child = node.first_child(); child; child = child.next_sibling()) {
					if (child.type() != pugi::node_element)
						continue;
					const std::string childName = std::string(XmlLocalName(child.name()));
					if (isFrameTagName(family, childName)) {
						lastFrameTag = child;
						lastFrameTagName = childName;
					}
				}

				for (auto child = node.first_child(); child; child = child.next_sibling()) {
					if (child.type() != pugi::node_element)
						continue;

					const std::string childName = std::string(XmlLocalName(child.name()));
					std::string fieldKey;
					std::string currentValue;

					if (isFrameTagName(family, childName)) {
						if (child != lastFrameTag) {
							TemplateRedundantChildInfo info;
							info.location = BuildNodeLocationPath(child);
							info.tagName = childName;
							info.shadowedByLaterFrameTag = true;
							info.shadowingTagName = lastFrameTagName;
							if (sourceBytes)
								info.line = OffsetToLineNumber(*sourceBytes, child.offset_debug());

							result.locations.insert(info.location);
							result.infos.push_back(std::move(info));
							if (collectRemovals)
								result.removableNodes.push_back(child);
							continue;
						}
						fieldKey = "__frameSpec";
						currentValue = normalizeFrameSpec(childName, child);
					} else {
						fieldKey = fieldKeyFor(family, childName);
						if (fieldKey.empty())
							continue;
						currentValue = normalizedValueForField(fieldKey, child);
					}

					if (fieldKey.empty())
						continue;

					auto itDefault = effective.find(fieldKey);
					const bool isRedundant = (itDefault != effective.end() && itDefault->second == currentValue);

					if (isRedundant) {
						TemplateRedundantChildInfo info;
						info.location = BuildNodeLocationPath(child);
						info.tagName = childName;
						if (sourceBytes)
							info.line = OffsetToLineNumber(*sourceBytes, child.offset_debug());

						result.locations.insert(info.location);
						result.infos.push_back(std::move(info));
						if (collectRemovals)
							result.removableNodes.push_back(child);
					}

					// Overlay values while processing node in runtime order.
					effective[fieldKey] = currentValue;
				}

				if (isDefaultNodeName(localName))
					familyTemplates[family][templateName] = std::move(effective);
			}

			return result;
		}

		bool isDefaultNodeName(const std::string& localName)
		{
			return localName == "bone-default" ||
			       localName == "per-triangle-shape-default" ||
			       localName == "per-vertex-shape-default" ||
			       localName == "generic-constraint-default" ||
			       localName == "stiffspring-constraint-default" ||
			       localName == "conetwist-constraint-default";
		}

	std::unordered_set<std::string> CollectTemplateRedundantChildLocations(const pugi::xml_document& doc)
	{
		return analyzeTemplateRedundantChildren(doc, false, nullptr).locations;
	}

	std::vector<TemplateRedundantChildInfo> CollectTemplateRedundantChildrenInfo(
		const pugi::xml_document& doc,
		const std::string* sourceBytes)
	{
		return analyzeTemplateRedundantChildren(doc, false, sourceBytes).infos;
	}

	bool RemoveTemplateRedundantChildren(pugi::xml_document& doc)
	{
		auto analysis = analyzeTemplateRedundantChildren(doc, true, nullptr);
		if (analysis.removableNodes.empty())
			return false;

		for (const auto& child : analysis.removableNodes) {
			auto parent = child.parent();
			if (parent)
				parent.remove_child(child);
		}

		return !analysis.removableNodes.empty();
	}

}  // namespace hdt
