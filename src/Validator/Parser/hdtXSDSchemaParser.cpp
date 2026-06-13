#include "hdtXSDSchemaParser.h"

#include <charconv>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hdt
{
	namespace
	{
		/// Recursively collects allowed child element names from XSD compositor nodes
		/// and nested complex content sections.
		void walkContentModel(pugi::xml_node node, std::unordered_set<std::string>& children)
		{
			for (auto child : node.children()) {
				const std::string_view tag = child.name();
				if (tag == "xsd:element") {
					const char* ref = child.attribute("ref").as_string("");
					const char* name = child.attribute("name").as_string("");
					if (ref[0])
						children.insert(ref);
					else if (name[0])
						children.insert(name);
				} else if (tag == "xsd:choice" || tag == "xsd:sequence" || tag == "xsd:all" || tag == "xsd:complexType" || tag == "xsd:complexContent") {
					walkContentModel(child, children);
				}
			}
		}

		/// Recursively collects all xsd:enumeration values under node.
		void collectEnumerations(pugi::xml_node node, std::unordered_set<std::string>& out)
		{
			for (auto child : node.children()) {
				if (std::string_view(child.name()) == "xsd:enumeration") {
					const char* v = child.attribute("value").as_string("");
					if (v[0])
						out.insert(v);
				} else {
					collectEnumerations(child, out);
				}
			}
		}

		/// Recursively collects required attribute names, skipping nested element branches.
		void collectRequiredAttrs(pugi::xml_node node, std::vector<std::string>& out)
		{
			for (auto child : node.children()) {
				const std::string_view tag = child.name();
				if (tag == "xsd:attribute") {
					const char* name = child.attribute("name").as_string("");
					const char* use = child.attribute("use").as_string("");
					if (name[0] && std::string_view(use) == "required")
						out.push_back(name);
				} else if (tag != "xsd:element") {
					collectRequiredAttrs(child, out);
				}
			}
		}

		/// Maps known XSD built-in type names to validator type constraints.
		TypeConstraint builtinTypeConstraint(const std::string& typeName)
		{
			TypeConstraint tc;
			if (typeName == "xsd:float" || typeName == "xs:float" ||
				typeName == "xsd:double" || typeName == "xs:double")
				tc.base = TypeConstraint::Base::Float;
			else if (typeName == "xsd:boolean" || typeName == "xs:boolean")
				tc.base = TypeConstraint::Base::Boolean;
			else if (typeName == "xsd:integer" || typeName == "xs:integer" ||
					 typeName == "xsd:int" || typeName == "xs:int" ||
					 typeName == "xsd:long" || typeName == "xs:long")
				tc.base = TypeConstraint::Base::Integer;
			return tc;
		}

		/// Resolves a type name to a constraint using named types first, then built-ins.
		TypeConstraint resolveTypeConstraint(
			const std::string& typeName,
			const std::unordered_map<std::string, TypeConstraint>& namedTypes)
		{
			const auto it = namedTypes.find(typeName);
			return (it != namedTypes.end()) ? it->second : builtinTypeConstraint(typeName);
		}

		/// Recursively extracts element text and attribute type constraints for one element.
		void collectTypeConstraints(
			pugi::xml_node node, bool inCompositor,
			const std::string& elemName,
			const std::unordered_map<std::string, TypeConstraint>& namedSimpleTypes,
			std::unordered_map<std::string, TypeConstraint>& textCons,
			std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>>& attrCons)
		{
			for (auto child : node.children()) {
				const std::string_view tag = child.name();
				if (tag == "xsd:attribute") {
					const char* aName = child.attribute("name").as_string("");
					const char* aType = child.attribute("type").as_string("");
					if (aName[0] && aType[0]) {
						TypeConstraint tc = resolveTypeConstraint(aType, namedSimpleTypes);
						if (tc.base != TypeConstraint::Base::Any)
							attrCons[elemName][aName] = tc;
					}
				} else if (tag == "xsd:extension" || tag == "xsd:restriction") {
					const char* base = child.attribute("base").as_string("");
					if (base[0]) {
						TypeConstraint tc = resolveTypeConstraint(base, namedSimpleTypes);
						if (tc.base != TypeConstraint::Base::Any)
							textCons[elemName] = tc;
					}
					collectTypeConstraints(child, inCompositor, elemName, namedSimpleTypes, textCons, attrCons);
				} else if (tag == "xsd:choice" || tag == "xsd:sequence" || tag == "xsd:all") {
					collectTypeConstraints(child, true, elemName, namedSimpleTypes, textCons, attrCons);
				} else if (tag == "xsd:element") {
					if (inCompositor)
						continue;
					collectTypeConstraints(child, inCompositor, elemName, namedSimpleTypes, textCons, attrCons);
				} else {
					collectTypeConstraints(child, inCompositor, elemName, namedSimpleTypes, textCons, attrCons);
				}
			}
		}

		/// Recursively gathers typed attributes declared inside a complexType node.
		void gatherComplexTypeAttrs(
			pugi::xml_node n,
			const std::unordered_map<std::string, TypeConstraint>& namedSimpleTypes,
			std::unordered_map<std::string, TypeConstraint>& attrCons)
		{
			for (auto child : n.children()) {
				const std::string_view childTag = child.name();
				if (childTag == "xsd:attribute") {
					const char* aName = child.attribute("name").as_string("");
					const char* aType = child.attribute("type").as_string("");
					if (aName[0] && aType[0]) {
						TypeConstraint tc = resolveTypeConstraint(aType, namedSimpleTypes);
						if (tc.base != TypeConstraint::Base::Any)
							attrCons[aName] = tc;
					}
				}
				// Skip recursing into xsd:element children to avoid collecting nested element attributes
				if (childTag != "xsd:element") {
					gatherComplexTypeAttrs(child, namedSimpleTypes, attrCons);
				}
			}
		}

		/// Parses all top-level elements and returns their allowed enumeration values.
		std::unordered_map<std::string, std::unordered_set<std::string>>
			parseAllElementEnumerations(pugi::xml_node schema)
		{
			std::unordered_map<std::string, std::unordered_set<std::string>> namedEnums;
			for (auto st : schema.children("xsd:simpleType")) {
				const char* name = st.attribute("name").as_string("");
				if (!name[0])
					continue;
				std::unordered_set<std::string> vals;
				collectEnumerations(st, vals);
				if (!vals.empty())
					namedEnums[name] = std::move(vals);
			}

			std::unordered_map<std::string, std::unordered_set<std::string>> result;
			for (auto elem : schema.children("xsd:element")) {
				const char* name = elem.attribute("name").as_string("");
				if (!name[0])
					continue;
				std::unordered_set<std::string> vals;
				const char* typeName = elem.attribute("type").as_string("");
				if (typeName[0]) {
					const auto it = namedEnums.find(typeName);
					if (it != namedEnums.end())
						vals.insert(it->second.begin(), it->second.end());
				}
				collectEnumerations(elem, vals);
				if (!vals.empty())
					result[name] = std::move(vals);
			}

			return result;
		}

		/// Parses required attributes per top-level element.
		std::unordered_map<std::string, std::vector<std::string>>
			parseAllRequiredAttrs(pugi::xml_node schema)
		{
			// First, collect all named complexTypes and their required attributes
			std::unordered_map<std::string, std::vector<std::string>> namedComplexTypeRequiredAttrs;
			for (auto ct : schema.children("xsd:complexType")) {
				const char* typeName = ct.attribute("name").as_string("");
				if (!typeName[0])
					continue;
				std::vector<std::string> attrs;
				collectRequiredAttrs(ct, attrs);
				if (!attrs.empty())
					namedComplexTypeRequiredAttrs[typeName] = std::move(attrs);
			}

			std::unordered_map<std::string, std::vector<std::string>> result;
			for (auto elem : schema.children("xsd:element")) {
				const char* name = elem.attribute("name").as_string("");
				if (!name[0])
					continue;
				std::vector<std::string> attrs;
				// First collect inline required attributes
				collectRequiredAttrs(elem, attrs);
				// If element has a type attribute, also include required attributes from that named complexType
				const char* typeName = elem.attribute("type").as_string("");
				if (typeName[0]) {
					const auto it = namedComplexTypeRequiredAttrs.find(typeName);
					if (it != namedComplexTypeRequiredAttrs.end()) {
						for (const auto& attr : it->second) {
							// Avoid duplicates
							if (std::find(attrs.begin(), attrs.end(), attr) == attrs.end())
								attrs.push_back(attr);
						}
					}
				}
				if (!attrs.empty())
					result[name] = std::move(attrs);
			}
			return result;
		}

		/// Parses an XSD selector XPath into a set of element names for key/keyref checks.
		std::unordered_set<std::string> parseXPathElems(const std::string& xpath)
		{
			std::unordered_set<std::string> result;
			std::string token;
			for (char c : xpath) {
				if (c == '|') {
					size_t start = token.find_first_not_of(" \t./");
					if (start != std::string::npos)
						result.insert(token.substr(start));
					token.clear();
				} else {
					token += c;
				}
			}
			size_t start = token.find_first_not_of(" \t./");
			if (start != std::string::npos)
				result.insert(token.substr(start));
			return result;
		}

		/// Recursively collects xsd:key, xsd:unique, and xsd:keyref definitions.
		void collectKeyNodes(
			pugi::xml_node node,
			std::unordered_map<std::string, PhysicsSchema::KeyDef>& keyDefs,
			std::unordered_map<std::string, PhysicsSchema::KeyRefDef>& keyRefDefs)
		{
			for (auto child : node.children()) {
				const std::string_view tag = child.name();
				if (tag == "xsd:key" || tag == "xsd:unique") {
					const char* name = child.attribute("name").as_string("");
					if (!name[0]) {
						collectKeyNodes(child, keyDefs, keyRefDefs);
						continue;
					}
					std::string selector, field;
					for (auto sub : child.children()) {
						const std::string_view stag = sub.name();
						if (stag == "xsd:selector")
							selector = sub.attribute("xpath").as_string("");
						else if (stag == "xsd:field") {
							const std::string fp = sub.attribute("xpath").as_string("");
							field = (!fp.empty() && fp[0] == '@') ? fp.substr(1) : fp;
						}
					}
					if (!selector.empty() && !field.empty())
						keyDefs[name] = { parseXPathElems(selector), field };
				} else if (tag == "xsd:keyref") {
					const char* name = child.attribute("name").as_string("");
					const char* refer = child.attribute("refer").as_string("");
					if (!name[0] || !refer[0]) {
						collectKeyNodes(child, keyDefs, keyRefDefs);
						continue;
					}
					std::string selector, field;
					for (auto sub : child.children()) {
						const std::string_view stag = sub.name();
						if (stag == "xsd:selector")
							selector = sub.attribute("xpath").as_string("");
						else if (stag == "xsd:field") {
							const std::string fp = sub.attribute("xpath").as_string("");
							field = (!fp.empty() && fp[0] == '@') ? fp.substr(1) : fp;
						}
					}
					if (!selector.empty() && !field.empty())
						keyRefDefs[name] = { refer, parseXPathElems(selector), field };
				} else {
					collectKeyNodes(child, keyDefs, keyRefDefs);
				}
			}
		}

		/// Parses all key/keyref constraints from the schema node.
		void parseKeyConstraints(
			pugi::xml_node schema,
			std::unordered_map<std::string, PhysicsSchema::KeyDef>& keyDefs,
			std::unordered_map<std::string, PhysicsSchema::KeyRefDef>& keyRefDefs)
		{
			collectKeyNodes(schema, keyDefs, keyRefDefs);
		}

		/// Builds allowed-children maps and the known-element set from XSD elements/types.
		std::unordered_map<std::string, std::unordered_set<std::string>>
			parseAllowedChildren(pugi::xml_node schema, std::unordered_set<std::string>& knownElements)
		{
			std::unordered_map<std::string, std::unordered_set<std::string>> typeChildren;
			for (auto ct : schema.children("xsd:complexType")) {
				const char* typeName = ct.attribute("name").as_string("");
				if (!typeName[0])
					continue;
				std::unordered_set<std::string> children;
				walkContentModel(ct, children);
				if (!children.empty()) {
					for (const auto& c : children)
						knownElements.insert(c);
					typeChildren[typeName] = std::move(children);
				}
			}

			std::unordered_map<std::string, std::unordered_set<std::string>> result;
			for (auto elem : schema.children("xsd:element")) {
				const char* name = elem.attribute("name").as_string("");
				if (!name[0])
					continue;
				knownElements.insert(name);
				const char* typeName = elem.attribute("type").as_string("");
				if (typeName[0]) {
					const auto it = typeChildren.find(typeName);
					if (it != typeChildren.end())
						result[name] = it->second;
					continue;
				}
				std::unordered_set<std::string> children;
				walkContentModel(elem, children);
				if (!children.empty()) {
					for (const auto& c : children)
						knownElements.insert(c);
					result[name] = std::move(children);
				}
			}

			return result;
		}

		/// Parses text and attribute type constraints for all top-level schema elements.
		void parseAllTypeConstraints(
			pugi::xml_node schema,
			std::unordered_map<std::string, TypeConstraint>& elementTextConstraints,
			std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>>& elementAttrConstraints)
		{
			std::unordered_map<std::string, TypeConstraint> namedSimpleTypes;
			for (auto st : schema.children("xsd:simpleType")) {
				const char* typeName = st.attribute("name").as_string("");
				if (!typeName[0])
					continue;
				TypeConstraint tc;
				for (auto child : st.children()) {
					if (std::string_view(child.name()) != "xsd:restriction")
						continue;
					const char* base = child.attribute("base").as_string("");
					if (base[0])
						tc = resolveTypeConstraint(base, namedSimpleTypes);
					for (auto facet : child.children()) {
						const std::string_view ftag = facet.name();
						if (ftag == "xsd:minInclusive") {
							std::string_view vstr(facet.attribute("value").as_string(""));
							double v;
							if (std::from_chars(vstr.data(), vstr.data() + vstr.size(), v).ec == std::errc{}) {
								tc.minInclusive = v;
								tc.hasMin = true;
							}
						} else if (ftag == "xsd:maxInclusive") {
							std::string_view vstr(facet.attribute("value").as_string(""));
							double v;
							if (std::from_chars(vstr.data(), vstr.data() + vstr.size(), v).ec == std::errc{}) {
								tc.maxInclusive = v;
								tc.hasMax = true;
							}
						}
					}
				}
				namedSimpleTypes[typeName] = tc;
			}

			std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>> namedComplexTypeAttrs;
			for (auto ct : schema.children("xsd:complexType")) {
				const char* typeName = ct.attribute("name").as_string("");
				if (!typeName[0])
					continue;
				std::unordered_map<std::string, TypeConstraint> attrCons;
				gatherComplexTypeAttrs(ct, namedSimpleTypes, attrCons);
				if (!attrCons.empty())
					namedComplexTypeAttrs[typeName] = std::move(attrCons);
			}

			for (auto elem : schema.children("xsd:element")) {
				const char* name = elem.attribute("name").as_string("");
				if (!name[0])
					continue;
				const char* typeName = elem.attribute("type").as_string("");
				if (typeName[0]) {
					TypeConstraint tc = resolveTypeConstraint(typeName, namedSimpleTypes);
					if (tc.base != TypeConstraint::Base::Any)
						elementTextConstraints[name] = tc;
					const auto cit = namedComplexTypeAttrs.find(typeName);
					if (cit != namedComplexTypeAttrs.end())
						elementAttrConstraints[name] = cit->second;
					continue;
				}
				collectTypeConstraints(elem, false, name, namedSimpleTypes, elementTextConstraints, elementAttrConstraints);
			}
		}
	}  // namespace

	/// Parses XSD-derived validation metadata into PhysicsSchema for runtime checks.
	bool ParsePhysicsSchemaFromXSD(const pugi::xml_document& doc, PhysicsSchema& schema)
	{
		// Use document_element() to get the xsd:schema root node
		pugi::xml_node schemaNode = doc.document_element();
		if (!schemaNode) {
			// Fall back to child("xsd:schema") if document_element fails
			schemaNode = doc.child("xsd:schema");
		}
		if (!schemaNode)
			return false;

		schema.elementEnums = parseAllElementEnumerations(schemaNode);
		schema.rootTag = "system";
		parseKeyConstraints(schemaNode, schema.keyDefs, schema.keyRefDefs);
		schema.requiredAttrs = parseAllRequiredAttrs(schemaNode);
		schema.knownElements.clear();
		schema.allowedChildren = parseAllowedChildren(schemaNode, schema.knownElements);
		parseAllTypeConstraints(schemaNode, schema.elementTextConstraints, schema.elementAttrConstraints);
		return true;
	}

}  // namespace hdt
