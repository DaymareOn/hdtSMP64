#include "hdtTemplateDefaults.h"

#include "../Config/hdtValidatorPaths.h"
#include "hdtStringUtils.h"
#include "hdtValidatorFamily.h"
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
		// ── Types ─────────────────────────────────────────────────────────────

		using FieldMap = std::unordered_map<std::string, std::string>;
		using TemplateMap = std::unordered_map<std::string, FieldMap>;

		struct AnalysisResult
		{
			std::unordered_set<std::string> locations;
			std::vector<TemplateRedundantChildInfo> infos;
			std::vector<pugi::xml_node> removableNodes;
		};

		// Schematron-parsed default values, split by family and by global (all families).
		struct SchDefaults
		{
			std::unordered_map<Family, FieldMap> byFamily;
			FieldMap global;
		};

		// ── Value normalizers ─────────────────────────────────────────────────
		// Produce canonical string representations so field values from XML can
		// be compared correctly regardless of whitespace, locale, or float format.

		static std::string toLowerAscii(std::string s)
		{
			std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
				return static_cast<char>(std::tolower(c));
			});
			return s;
		}

		// Strips trailing zeros and normalises "-0" → "0".
		// Allows comparing "1.00" with "1" and "0.50" with ".5".
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

		// Build a canonical key from a <transform> node regardless of which child
		// representation was used (basis quaternion, basis-axis-angle, or absent/default).
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
			key += hasOrigin ? origin : "0,0,0";

			return key;
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

		// Collapse frameInA, frameInB, and frameInLerp to a single canonical key.
		// lerp(1,1) is semantically equivalent to a full FrameInB identity, so
		// both normalise to the same "B:<identity>" key.
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

		// ── Node classifiers ──────────────────────────────────────────────────

		// Frame tags (frameInA / frameInB / frameInLerp) are only valid on Generic
		// and ConeTwist constraints, and only the LAST one in a node is effective —
		// earlier ones are shadowed rather than simply redundant.
		static bool isFrameTagName(const Family family, const std::string& localTag)
		{
			if (family != Family::Generic && family != Family::ConeTwist)
				return false;
			return localTag == "frameInA" || localTag == "frameInB" || localTag == "frameInLerp";
		}

		// These four tags say which things a shape may or may not bump into. They work
		// differently from normal settings: the moment a shape (or template) writes ANY of
		// them, the game throws away the whole collision list it inherited from its template
		// and keeps only the ones written right here (see SkyrimSystemCreator::readPerVertexShape
		// / readPerVertexShapeTemplate). So re-writing a tag the template already had is NOT a
		// pointless repeat — delete it and the tag is gone. That makes "this equals the
		// inherited default, so drop it" wrong for these tags, so we leave them out of that
		// check. (Plain <tag> is different: it ADDS to the inherited list instead of replacing
		// it, so repeating one there really is redundant.)
		static bool isReplaceSemanticsCollisionList(const std::string& localTag)
		{
			return localTag == "can-collide-with-tag" || localTag == "no-collide-with-tag" ||
			       localTag == "can-collide-with-bone" || localTag == "no-collide-with-bone";
		}

		// ConeTwist constraint fields have accrued legacy aliases across FSMP versions.
		// Canonicalise all synonyms so template inheritance comparison works correctly.
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

		static std::string fieldKeyFor(const Family family, const std::string& localTag)
		{
			switch (family) {
			case Family::ConeTwist:
				return canonicalFieldForConetwist(localTag);
			case Family::Bone:
			case Family::PerTriangle:
			case Family::PerVertex:
			case Family::Generic:
			case Family::StiffSpring:
				return localTag;
			default:
				return {};
			}
		}

		// ── Schema loaders ────────────────────────────────────────────────────
		// Both loaders are lazy (std::call_once) and read-only after first call.

		static std::string stripTypePrefix(std::string typeName)
		{
			auto colon = typeName.find(':');
			if (colon != std::string::npos)
				typeName.erase(0, colon + 1);
			return typeName;
		}

		// Lazy-load a map of element-name → XSD type from the physics XSD.
		// Used by normalizedValueForField to dispatch to the correct normalizer
		// (boolean, vector3, transform, lerp, or scalar) without hardcoding names.
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

		// Parse a Schematron rule `context` attribute such as
		// "//f:bone-default/f:damping[...]" to determine which constraint
		// families the default applies to.
		static std::vector<Family> parseFamiliesFromContext(const std::string& context)
		{
			std::vector<Family> out;
			if (context.find("bone-default") != std::string::npos || context.find("bone") != std::string::npos)
				out.push_back(Family::Bone);
			if (context.find("per-triangle-shape-default") != std::string::npos || context.find("per-triangle-shape") != std::string::npos)
				out.push_back(Family::PerTriangle);
			if (context.find("per-vertex-shape-default") != std::string::npos || context.find("per-vertex-shape") != std::string::npos)
				out.push_back(Family::PerVertex);
			if (context.find("generic-constraint-default") != std::string::npos || context.find("generic-constraint") != std::string::npos)
				out.push_back(Family::Generic);
			if (context.find("stiffspring-constraint-default") != std::string::npos || context.find("stiffspring-constraint") != std::string::npos)
				out.push_back(Family::StiffSpring);
			if (context.find("conetwist-constraint-default") != std::string::npos || context.find("conetwist-constraint") != std::string::npos)
				out.push_back(Family::ConeTwist);
			return out;
		}

		// Extract field-name → default-value pairs from the XPath equality
		// predicates encoded in a Schematron rule context string.
		// Handles three forms: boolean normalize-space(.), vector3 number(@x/y/z),
		// and scalar number(.).
		static std::unordered_map<std::string, std::string> parseFieldDefaultsFromContext(const std::string& context)
		{
			std::unordered_map<std::string, std::string> out;

			static const std::regex boolRule(R"(f:([A-Za-z0-9_\-]+)\[\s*normalize-space\(\.\)\s*=\s*'([^']+)')");
			for (auto it = std::sregex_iterator(context.begin(), context.end(), boolRule); it != std::sregex_iterator(); ++it) {
				std::string tag = (*it)[1].str();
				std::string value = normalizeBoolText((*it)[2].str());
				out[tag] = value;
			}

			static const std::regex vecRule(R"(f:([A-Za-z0-9_\-]+)\[\s*number\(@x\)\s*=\s*([-0-9.]+)\s*and\s*number\(@y\)\s*=\s*([-0-9.]+)\s*and\s*number\(@z\)\s*=\s*([-0-9.]+))");
			for (auto it = std::sregex_iterator(context.begin(), context.end(), vecRule); it != std::sregex_iterator(); ++it) {
				std::string tag = (*it)[1].str();
				std::string value = normalizeNumericText((*it)[2].str()) + "," +
				                    normalizeNumericText((*it)[3].str()) + "," +
				                    normalizeNumericText((*it)[4].str());
				out[tag] = value;
			}

			static const std::regex scalarRule(R"(f:([A-Za-z0-9_\-]+)\[\s*number\(\.\)\s*=\s*([-0-9.]+))");
			for (auto it = std::sregex_iterator(context.begin(), context.end(), scalarRule); it != std::sregex_iterator(); ++it) {
				std::string tag = (*it)[1].str();
				if (out.find(tag) != out.end())
					continue;
				out[tag] = normalizeNumericText((*it)[2].str());
			}

			return out;
		}

		// Lazy-load the `redundant-default-values` Schematron pattern to build
		// the authoritative per-family and global default maps used by the
		// redundancy analysis.  Only the named pattern is consumed; all other
		// SCH rules are ignored.
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

		// ── Runtime model ─────────────────────────────────────────────────────

		// Normalize a field value to a canonical string for comparison.
		// Some fields are overloaded across families in the XSD (e.g. linearDamping
		// is a scalar for bones but a vector3 for constraints), so family context
		// is checked before falling back to the XSD type map.
		static std::string normalizedValueForField(const Family family, const std::string& field, const pugi::xml_node& valueNode)
		{
			if (field == "linearDamping" || field == "angularDamping") {
				if (family == Family::Bone)
					return normalizeNumericText(valueNode.text().as_string());
				return normalizeVector3Attrs(valueNode);
			}

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

		// Build the complete per-family default FieldMap used as the starting
		// point for each node during analysis.  Merges SCH-loaded defaults with
		// hardcoded fallbacks for values not representable as SCH literals
		// (identity transforms, zero vectors, frame specs).
		static std::unordered_map<Family, FieldMap> makeBaseDefaults()
		{
			std::unordered_map<Family, FieldMap> defaults;

			const auto& schDefaults = getDefaultsFromSchematron();
			defaults = schDefaults.byFamily;

			for (auto family : { Family::Bone, Family::PerTriangle, Family::PerVertex, Family::Generic, Family::StiffSpring, Family::ConeTwist }) {
				for (const auto& [k, v] : schDefaults.global) {
					std::string field = (family == Family::ConeTwist) ? canonicalFieldForConetwist(k) : k;
					defaults[family].insert_or_assign(field, v);
				}
			}

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

		// Look up a named template, falling back to the unnamed ("") template
		// when the name is absent — matching the runtime inheritance fallback.
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

		// ── Document helpers ──────────────────────────────────────────────────

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

		// Stable sorted signature over a FieldMap — used to detect two named
		// default templates with identical effective field sets.
		static std::string makeFieldMapSignature(const FieldMap& fields)
		{
			std::vector<std::pair<std::string, std::string>> entries(fields.begin(), fields.end());
			std::sort(entries.begin(), entries.end(), [](const auto& l, const auto& r) {
				return l.first < r.first;
			});

			std::string signature;
			for (const auto& [key, value] : entries) {
				signature += key;
				signature += '=';
				signature += value;
				signature += '\n';
			}

			return signature;
		}

		// Single reverse pass over top-level nodes: a default node is removed
		// if no instantiation or extending default that appears after it references
		// its name.  Reverse scan means we see users before their templates.
		static bool removeUnusedDefaultNodes(pugi::xml_document& doc)
		{
			auto sysNode = findSystemNode(doc);
			if (!sysNode)
				return false;

			std::vector<pugi::xml_node> topLevelNodes;
			for (auto node = sysNode.first_child(); node; node = node.next_sibling()) {
				if (node.type() == pugi::node_element)
					topLevelNodes.push_back(node);
			}

			std::unordered_map<Family, bool> unnamedTemplateNeeded;
			std::unordered_map<Family, std::unordered_set<std::string>> namedTemplatesNeeded;
			std::vector<pugi::xml_node> toRemove;

			for (auto it = topLevelNodes.rbegin(); it != topLevelNodes.rend(); ++it) {
				const auto& node = *it;
				const std::string localName = std::string(XmlLocalName(node.name()));
				const Family family = familyForNode(localName);
				if (family == Family::None)
					continue;

				if (isDefaultNodeName(localName)) {
					const std::string templateName = TrimAsciiWhitespace(node.attribute("name").as_string());
					const std::string extendsName = TrimAsciiWhitespace(node.attribute("extends").as_string());

					bool needed = false;
					if (templateName.empty()) {
						needed = unnamedTemplateNeeded[family];
						if (needed)
							unnamedTemplateNeeded[family] = false;
					} else {
						auto& liveNames = namedTemplatesNeeded[family];
						auto liveIt = liveNames.find(templateName);
						if (liveIt != liveNames.end()) {
							needed = true;
							liveNames.erase(liveIt);
						}
					}

					if (!needed) {
						toRemove.push_back(node);
						continue;
					}

					if (extendsName.empty())
						unnamedTemplateNeeded[family] = true;
					else
						namedTemplatesNeeded[family].insert(extendsName);
					continue;
				}

				const std::string templateName = TrimAsciiWhitespace(node.attribute("template").as_string());
				if (templateName.empty())
					unnamedTemplateNeeded[family] = true;
				else
					namedTemplatesNeeded[family].insert(templateName);
			}

			for (const auto& node : toRemove)
				sysNode.remove_child(node);

			return !toRemove.empty();
		}

		// ── Core analysis ─────────────────────────────────────────────────────

		// Walk all top-level nodes in document order, maintaining an effective
		// FieldMap that mirrors runtime template inheritance.  A child element is
		// redundant when its canonical value equals the inherited effective default
		// at that point; a frame tag that is not the last one in its parent node
		// is shadowed (inactive) rather than redundant in the strict sense.
		// collectRemovals=true populates result.removableNodes for callers that
		// need to mutate the document.
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

				// Pre-scan to find the last frame tag: only it is effective.
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

					// Collision lists don't keep an inherited value to compare against (the game
					// wipes it the moment one is written), so skip them here.
					if (isReplaceSemanticsCollisionList(childName))
						continue;

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
						currentValue = normalizedValueForField(family, fieldKey, child);
						// weight-threshold sets the threshold of ONE bone (named by its bone
						// attribute), so two elements for different bones are independent —
						// never duplicates of each other. Track one value per bone.
						if (childName == "weight-threshold")
							fieldKey += "@" + TrimAsciiWhitespace(child.attribute("bone").as_string());
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

					// Overlay value so subsequent siblings see the updated effective state.
					effective[fieldKey] = currentValue;
				}

				if (isDefaultNodeName(localName))
					familyTemplates[family][templateName] = std::move(effective);
			}

			return result;
		}

		// Turns a list of collision tags/bones into one string we can compare. The game
		// stores these as a set — order doesn't matter and duplicates don't count — so we
		// sort the values and throw out repeats before gluing them together. Two lists with
		// the same members (in any order, with any repeats) then come out as the same string.
		static std::string canonicalCollisionSet(std::vector<std::string> values)
		{
			std::sort(values.begin(), values.end());
			values.erase(std::unique(values.begin(), values.end()), values.end());
			std::string out;
			for (const auto& v : values) {
				out += v;
				// Glue character between values: an invisible control character that can
				// never appear inside a tag or bone name, so two different lists can
				// never accidentally glue into the same string.
				out.push_back('\x1f');
			}
			return out;
		}

		// Records this template's finished collision lists so two templates can be told apart.
		// We copy the game's rule: if the node writes ANY tag-collision line, both its "can"
		// and "no" tag lists are first wiped clean, then filled only from what's written here
		// (bones work the same way; see SkyrimSystemCreator::readPerVertexShapeTemplate). Each
		// finished list is saved as a single entry, so two templates count as the same only
		// when every collision member matches — but the order they were written in is ignored.
		static void applyCollisionListOverrides(const pugi::xml_node& node, FieldMap& effective)
		{
			std::vector<std::string> canTags, noTags, canBones, noBones;
			bool hasTagCollide = false;
			bool hasBoneCollide = false;

			for (auto child = node.first_child(); child; child = child.next_sibling()) {
				if (child.type() != pugi::node_element)
					continue;
				const std::string name = std::string(XmlLocalName(child.name()));
				const std::string text = TrimAsciiWhitespace(child.text().as_string());
				if (name == "can-collide-with-tag") {
					hasTagCollide = true;
					canTags.push_back(text);
				} else if (name == "no-collide-with-tag") {
					hasTagCollide = true;
					noTags.push_back(text);
				} else if (name == "can-collide-with-bone") {
					hasBoneCollide = true;
					canBones.push_back(text);
				} else if (name == "no-collide-with-bone") {
					hasBoneCollide = true;
					noBones.push_back(text);
				}
			}

			// Writing just one side (say only a "no" list) still wipes BOTH lists, so the
			// other side ends up empty here rather than keeping whatever was inherited.
			if (hasTagCollide) {
				effective["__canCollideWithTags"] = canonicalCollisionSet(std::move(canTags));
				effective["__noCollideWithTags"] = canonicalCollisionSet(std::move(noTags));
			}
			if (hasBoneCollide) {
				effective["__canCollideWithBones"] = canonicalCollisionSet(std::move(canBones));
				effective["__noCollideWithBones"] = canonicalCollisionSet(std::move(noBones));
			}
		}

		// Build a node's complete effective FieldMap the way the runtime resolves one:
		// start from the inherited template (a default node inherits via `extends`, an
		// instance via `template`; both fall back to the unnamed "" default), overlay each
		// own field tag — counting only the LAST frame tag, since earlier ones are shadowed
		// — then fold collision lists in as whole replace-semantics sets. Shared by every
		// walk that needs a node's final settings, so an instance and an equivalent default
		// normalise to the same map. (The per-child redundancy walk does NOT use this: it
		// must interleave detection with each overlay, so it keeps its own copy of the loop.)
		static FieldMap computeNodeEffectiveFields(
			const pugi::xml_node& node,
			const Family family,
			const bool isDefaultNode,
			const TemplateMap& familyTemplate)
		{
			const std::string parentName = isDefaultNode ? TrimAsciiWhitespace(node.attribute("extends").as_string()) : TrimAsciiWhitespace(node.attribute("template").as_string());
			FieldMap effective = getEffectiveTemplate(familyTemplate, parentName);

			// Pre-scan for the last frame tag: only it is effective (constraints only;
			// other families have no frame tags, so this stays empty for them).
			pugi::xml_node lastFrameTag;
			for (auto child = node.first_child(); child; child = child.next_sibling()) {
				if (child.type() != pugi::node_element)
					continue;
				if (isFrameTagName(family, std::string(XmlLocalName(child.name()))))
					lastFrameTag = child;
			}

			for (auto child = node.first_child(); child; child = child.next_sibling()) {
				if (child.type() != pugi::node_element)
					continue;

				const std::string childName = std::string(XmlLocalName(child.name()));
				std::string fieldKey;
				std::string currentValue;

				// Collision lists are folded in as whole sets below, not value-by-value.
				if (isReplaceSemanticsCollisionList(childName))
					continue;

				if (isFrameTagName(family, childName)) {
					if (child != lastFrameTag)
						continue;
					fieldKey = "__frameSpec";
					currentValue = normalizeFrameSpec(childName, child);
				} else {
					fieldKey = fieldKeyFor(family, childName);
					if (fieldKey.empty())
						continue;
					currentValue = normalizedValueForField(family, fieldKey, child);
					// weight-threshold sets the threshold of ONE named bone, so keep one
					// value per bone — templates with thresholds on different bones differ.
					if (childName == "weight-threshold")
						fieldKey += "@" + TrimAsciiWhitespace(child.attribute("bone").as_string());
				}

				effective[fieldKey] = currentValue;
			}

			applyCollisionListOverrides(node, effective);
			return effective;
		}

		// Walk default nodes in document order to compute each one's effective
		// FieldMap, then detect duplicates by signature.  Two templates are
		// equivalent when they produce identical canonical field sets after
		// inheritance is resolved.
		static std::unordered_map<std::string, std::string> collectEquivalentDefaultTemplateAliases(
			const pugi::xml_document& doc)
		{
			auto sysNode = findSystemNode(doc);
			if (!sysNode)
				return {};

			const auto baseDefaults = makeBaseDefaults();
			std::unordered_map<Family, TemplateMap> familyTemplates;
			for (const auto& [family, fields] : baseDefaults)
				familyTemplates[family][""] = fields;

			std::unordered_map<std::string, std::string> canonicalBySignature;
			std::unordered_map<std::string, std::string> aliases;

			for (auto node = sysNode.first_child(); node; node = node.next_sibling()) {
				if (node.type() != pugi::node_element)
					continue;

				const std::string localName = std::string(XmlLocalName(node.name()));
				const Family family = familyForNode(localName);
				if (family == Family::None || !isDefaultNodeName(localName))
					continue;

				const std::string templateName = TrimAsciiWhitespace(node.attribute("name").as_string());
				FieldMap effective = computeNodeEffectiveFields(node, family, /*isDefaultNode=*/true, familyTemplates[family]);

				familyTemplates[family][templateName] = effective;
				if (templateName.empty())
					continue;

				const std::string signature = localName + "\n" + makeFieldMapSignature(effective);
				auto [canonicalIt, inserted] = canonicalBySignature.emplace(signature, templateName);
				if (!inserted && canonicalIt->second != templateName)
					aliases[templateName] = canonicalIt->second;
			}

			return aliases;
		}

		// Detect top-level <bone> declarations whose complete effective settings are
		// identical to the bone the engine auto-creates on demand for an undeclared
		// node — the unnamed bone-default, a kinematic (mass-0) body.
		//
		// Why these are removable: when a node is referenced by a constraint or shape
		// but has no <bone> of its own, FSMP fabricates one from getBoneTemplate("")
		// (the unnamed bone-default). So a <bone> that only restates that default adds
		// nothing — a referenced node would get an identical body anyway, and an
		// unreferenced one leaves the declaration inert — and can be deleted either way.
		//
		// The check reuses the same effective-template machinery as the redundant-child
		// and template-equivalence walks: step through top-level nodes in document order,
		// keep boneTemplates[""] current as bone-default nodes are seen, and for each
		// plain <bone> build its full effective FieldMap (inherited template + own field
		// tags + collision-list overrides) exactly as a bone-default's is built, then
		// compare it to boneTemplates[""] at that point.
		//
		// It is conservative in both directions. Every element child either overwrites a
		// known field key or, being unmodelled, injects its own tag-name key — so a bone
		// carrying anything beyond the defaults (a non-default value, a collision-list
		// override, an unexpected child) ends up with a FieldMap that differs from the
		// default and is left alone. And with the .sch defaults absent, boneTemplates[""]
		// holds fewer keys, so an explicit non-default field can only fail to match —
		// the walk under-reports rather than ever flagging a non-default bone.
		static std::vector<RedundantBoneInfo> collectRedundantBoneDeclarations(
			const pugi::xml_document& doc,
			const std::string* sourceBytes)
		{
			std::vector<RedundantBoneInfo> out;
			auto sysNode = findSystemNode(doc);
			if (!sysNode)
				return out;

			const auto baseDefaults = makeBaseDefaults();
			TemplateMap boneTemplates;
			{
				auto it = baseDefaults.find(Family::Bone);
				boneTemplates[""] = (it != baseDefaults.end()) ? it->second : FieldMap{};
			}

			for (auto node = sysNode.first_child(); node; node = node.next_sibling()) {
				if (node.type() != pugi::node_element)
					continue;

				const std::string localName = std::string(XmlLocalName(node.name()));
				// Only the bone family touches boneTemplates; everything else is irrelevant
				// to both the inheritance state and the candidate set, so skip it.
				if (familyForNode(localName) != Family::Bone)
					continue;

				const bool isDefault = isDefaultNodeName(localName);  // "bone-default"
				// Same effective-map construction as the bone-default templates, so a <bone>
				// and an equivalent bone-default normalise to the same map and compare equal.
				FieldMap effective = computeNodeEffectiveFields(node, Family::Bone, isDefault, boneTemplates);

				if (isDefault) {
					const std::string templateName = TrimAsciiWhitespace(node.attribute("name").as_string());
					boneTemplates[templateName] = std::move(effective);
					continue;
				}

				auto defaultIt = boneTemplates.find("");
				if (defaultIt != boneTemplates.end() && effective == defaultIt->second) {
					RedundantBoneInfo info;
					info.location = BuildNodeLocationPath(node);
					info.boneName = TrimAsciiWhitespace(node.attribute("name").as_string());
					if (sourceBytes)
						info.line = OffsetToLineNumber(*sourceBytes, node.offset_debug());
					out.push_back(std::move(info));
				}
			}

			return out;
		}

	}  // anonymous namespace

	// ── Public API ────────────────────────────────────────────────────────────

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

	bool RemoveUnusedDefaultNodes(pugi::xml_document& doc)
	{
		return removeUnusedDefaultNodes(doc);
	}

	std::unordered_map<std::string, std::string> CollectEquivalentDefaultTemplateAliases(
		const pugi::xml_document& doc)
	{
		return collectEquivalentDefaultTemplateAliases(doc);
	}

	std::vector<RedundantBoneInfo> CollectRedundantBoneDeclarations(
		const pugi::xml_document& doc,
		const std::string* sourceBytes)
	{
		return collectRedundantBoneDeclarations(doc, sourceBytes);
	}

}  // namespace hdt
