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
		// Recursively collects allowed child element names from XSD compositor nodes
		// (xsd:choice, xsd:sequence, xsd:all) and forwards into xsd:complexType/xsd:complexContent.
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

		// Recursively finds all xsd:enumeration[@value] descendants and adds them to `out`.
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

		// Recursively finds xsd:attribute[@use='required'] descendants, skipping xsd:element subtrees.
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

		// Maps an XSD built-in type name (with namespace prefix) to a TypeConstraint.
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

		TypeConstraint resolveTypeConstraint(
			const std::string& typeName,
			const std::unordered_map<std::string, TypeConstraint>& namedTypes)
		{
			const auto it = namedTypes.find(typeName);
			return (it != namedTypes.end()) ? it->second : builtinTypeConstraint(typeName);
		}

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
				gatherComplexTypeAttrs(child, namedSimpleTypes, attrCons);
			}
		}

		std::unordered_map<std::string, std::unordered_set<std::string>>
		parseAllElementEnumerations(const pugi::xml_document& doc)
		{
			pugi::xml_node schema = doc.first_child();

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

		std::unordered_map<std::string, std::vector<std::string>>
		parseAllRequiredAttrs(const pugi::xml_document& doc)
		{
			std::unordered_map<std::string, std::vector<std::string>> result;
			for (auto elem : doc.first_child().children("xsd:element")) {
				const char* name = elem.attribute("name").as_string("");
				if (!name[0])
					continue;
				std::vector<std::string> attrs;
				collectRequiredAttrs(elem, attrs);
				if (!attrs.empty())
					result[name] = std::move(attrs);
			}
			return result;
		}

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

		void parseKeyConstraints(
			const pugi::xml_document& doc,
			std::unordered_map<std::string, PhysicsSchema::KeyDef>& keyDefs,
			std::unordered_map<std::string, PhysicsSchema::KeyRefDef>& keyRefDefs)
		{
			collectKeyNodes(doc.first_child(), keyDefs, keyRefDefs);
		}

		std::unordered_map<std::string, std::unordered_set<std::string>>
		parseAllowedChildren(const pugi::xml_document& doc, std::unordered_set<std::string>& knownElements)
		{
			pugi::xml_node schema = doc.first_child();
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

		void parseAllTypeConstraints(
			const pugi::xml_document& doc,
			std::unordered_map<std::string, TypeConstraint>& elementTextConstraints,
			std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>>& elementAttrConstraints)
		{
			pugi::xml_node schema = doc.first_child();

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

	bool ParsePhysicsSchemaFromXSD(const pugi::xml_document& doc, PhysicsSchema& schema)
	{
		schema.elementEnums = parseAllElementEnumerations(doc);
		schema.rootTag = "system";
		parseKeyConstraints(doc, schema.keyDefs, schema.keyRefDefs);
		schema.requiredAttrs = parseAllRequiredAttrs(doc);
		schema.knownElements.clear();
		schema.allowedChildren = parseAllowedChildren(doc, schema.knownElements);
		parseAllTypeConstraints(doc, schema.elementTextConstraints, schema.elementAttrConstraints);
		return true;
	}

}  // namespace hdt
