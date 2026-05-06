#include "hdtXSDValidator.h"

#include "hdtValidatorPaths.h"
#include "NetImmerseUtils.h"
#include "XmlReader.h"

#include <pugixml.hpp>

#include <algorithm>
#include <charconv>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hdt
{

	// Generic value type constraint derived from xs:restriction facets in the XSD.
	// All fields are populated from the XSD at load time -- no type names are hardcoded.
	struct TypeConstraint
	{
		enum class Base
		{
			Any,
			Float,
			Boolean,
			Integer
		} base = Base::Any;
		bool hasMin = false, hasMax = false;
		double minInclusive = 0.0, maxInclusive = 0.0;
	};

	// Parsed schema enumerations loaded once from hdtSMP64.xsd.
	struct PhysicsSchema
	{
		// Maps element tag names to their allowed values (inline anonymous simpleType enumerations).
		std::unordered_map<std::string, std::unordered_set<std::string>> elementEnums;
		// Root element tag — the first top-level xs:element in the XSD.
		std::string rootTag;
		// Required attributes per element — driven entirely by xs:attribute use="required".
		std::unordered_map<std::string, std::vector<std::string>> requiredAttrs;
		// Key/unique and keyref definitions parsed from xsd:key, xsd:unique, xsd:keyref.
		struct KeyDef
		{
			std::unordered_set<std::string> elems;
			std::string fieldAttr;
		};
		struct KeyRefDef
		{
			std::string refer;
			std::unordered_set<std::string> elems;
			std::string fieldAttr;
		};
		std::unordered_map<std::string, KeyDef> keyDefs;        // key/unique name — declared value sets
		std::unordered_map<std::string, KeyRefDef> keyRefDefs;  // keyref name — reference definitions
		// Allowed children per element — maps each named xs:element to the set of child element
		// tag names declared in its XSD content model (xs:choice/xs:all/xs:sequence).
		std::unordered_map<std::string, std::unordered_set<std::string>> allowedChildren;
		// All element tag names mentioned anywhere in the schema (for unknown-element detection).
		std::unordered_set<std::string> knownElements;
		// Per element: type constraint on its text content for non-enum typed elements.
		std::unordered_map<std::string, TypeConstraint> elementTextConstraints;
		// Per element: per-attribute type constraint on attribute values.
		std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>> elementAttrConstraints;
		bool loaded = false;  // true only when the XSD was successfully parsed
	};

	// Build a sorted, comma-separated string from a set of strings (for error messages).
	static std::string joinSet(const std::unordered_set<std::string>& s)
	{
		std::vector<std::string> sorted(s.begin(), s.end());
		std::sort(sorted.begin(), sorted.end());
		std::string result;
		for (const auto& v : sorted) {
			if (!result.empty())
				result += ", ";
			result += v;
		}
		return result;
	}

	// ---- DOM helpers for XSD schema parsing (using pugixml) ----

	// Recursively collects allowed child element names from XSD compositor nodes
	// (xsd:choice, xsd:sequence, xsd:all) and forwards into xsd:complexType/xsd:complexContent.
	static void walkContentModel(pugi::xml_node node, std::unordered_set<std::string>& children)
	{
		for (auto child : node.children()) {
			const std::string_view tag = child.name();
			if (tag == "xsd:element") {
				const char* ref  = child.attribute("ref").as_string("");
				const char* name = child.attribute("name").as_string("");
				if (ref[0])       children.insert(ref);
				else if (name[0]) children.insert(name);
			} else if (tag == "xsd:choice" || tag == "xsd:sequence" || tag == "xsd:all"
					|| tag == "xsd:complexType" || tag == "xsd:complexContent") {
				walkContentModel(child, children);
			}
		}
	}

	// Recursively finds all xsd:enumeration[@value] descendants and adds them to `out`.
	static void collectEnumerations(pugi::xml_node node, std::unordered_set<std::string>& out)
	{
		for (auto child : node.children()) {
			if (std::string_view(child.name()) == "xsd:enumeration") {
				const char* v = child.attribute("value").as_string("");
				if (v[0]) out.insert(v);
			} else {
				collectEnumerations(child, out);
			}
		}
	}

	// Recursively finds xsd:attribute[@use='required'] descendants, skipping xsd:element subtrees.
	static void collectRequiredAttrs(pugi::xml_node node, std::vector<std::string>& out)
	{
		for (auto child : node.children()) {
			const std::string_view tag = child.name();
			if (tag == "xsd:attribute") {
				const char* name = child.attribute("name").as_string("");
				const char* use  = child.attribute("use").as_string("");
				if (name[0] && std::string_view(use) == "required")
					out.push_back(name);
			} else if (tag != "xsd:element") {
				collectRequiredAttrs(child, out);
			}
		}
	}

	// Recursively collects xsd:attribute type constraints and xsd:extension/xsd:restriction
	// text-content type constraints from within an element's XSD declaration.
	// `inCompositor` is true when inside a content-model compositor (xsd:choice etc.);
	// xsd:element children are skipped in that context.
	static void collectTypeConstraints(
		pugi::xml_node node, bool inCompositor,
		const std::string& elemName,
		const std::unordered_map<std::string, TypeConstraint>& namedSimpleTypes,
		std::unordered_map<std::string, TypeConstraint>& textCons,
		std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>>& attrCons);

	// Collect all xsd:element enum constraints from the XSD — both inline anonymous
	// simpleType enumerations and elements with a type="..." attribute referencing
	// a named simpleType that itself has enumeration constraints.
	// Returns a map from element name to its set of allowed values.
	// No element, type, or attribute names are hardcoded.
	static std::unordered_map<std::string, std::unordered_set<std::string>>
		parseAllElementEnumerations(const pugi::xml_document& doc)
	{
		pugi::xml_node schema = doc.first_child();

		// Pass 1: named xsd:simpleType → enum values.
		std::unordered_map<std::string, std::unordered_set<std::string>> namedEnums;
		for (auto st : schema.children("xsd:simpleType")) {
			const char* name = st.attribute("name").as_string("");
			if (!name[0]) continue;
			std::unordered_set<std::string> vals;
			collectEnumerations(st, vals);
			if (!vals.empty())
				namedEnums[name] = std::move(vals);
		}

		// Pass 2: named xsd:element → inline enums and type= references.
		std::unordered_map<std::string, std::unordered_set<std::string>> result;
		for (auto elem : schema.children("xsd:element")) {
			const char* name = elem.attribute("name").as_string("");
			if (!name[0]) continue;
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

	// Build a map from every xsd:element name to its list of xsd:attribute use="required" names.
	// This is fully generic: no element or attribute names are hardcoded.
	static std::unordered_map<std::string, std::vector<std::string>>
		parseAllRequiredAttrs(const pugi::xml_document& doc)
	{
		std::unordered_map<std::string, std::vector<std::string>> result;
		for (auto elem : doc.first_child().children("xsd:element")) {
			const char* name = elem.attribute("name").as_string("");
			if (!name[0]) continue;
			std::vector<std::string> attrs;
			collectRequiredAttrs(elem, attrs);
			if (!attrs.empty())
				result[name] = std::move(attrs);
		}
		return result;
	}

	// Split an XSD xpath selector (e.g. "bone|bone-default") into a set of element names.
	static std::unordered_set<std::string> parseXPathElems(const std::string& xpath)
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

	// Recursively finds all xsd:key, xsd:unique, and xsd:keyref nodes in the XSD tree
	// and populates keyDefs / keyRefDefs.
	static void collectKeyNodes(
		pugi::xml_node node,
		std::unordered_map<std::string, PhysicsSchema::KeyDef>& keyDefs,
		std::unordered_map<std::string, PhysicsSchema::KeyRefDef>& keyRefDefs)
	{
		for (auto child : node.children()) {
			const std::string_view tag = child.name();
			if (tag == "xsd:key" || tag == "xsd:unique") {
				const char* name = child.attribute("name").as_string("");
				if (!name[0]) { collectKeyNodes(child, keyDefs, keyRefDefs); continue; }
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
				const char* name  = child.attribute("name").as_string("");
				const char* refer = child.attribute("refer").as_string("");
				if (!name[0] || !refer[0]) { collectKeyNodes(child, keyDefs, keyRefDefs); continue; }
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

	// Parse all xsd:key, xsd:unique, and xsd:keyref declarations from the XSD.
	// key/unique declarations establish which attribute values are valid keys.
	// keyref declarations establish which attributes must reference a declared key.
	static void parseKeyConstraints(
		const pugi::xml_document& doc,
		std::unordered_map<std::string, PhysicsSchema::KeyDef>& keyDefs,
		std::unordered_map<std::string, PhysicsSchema::KeyRefDef>& keyRefDefs)
	{
		collectKeyNodes(doc.first_child(), keyDefs, keyRefDefs);
	}

	// Collect the set of allowed child element names for each named xsd:element in the XSD,
	// and populate `knownElements` with every element name mentioned anywhere in the schema.
	// Two passes:
	//   Pass 1 -- named xsd:complexType content models -> child name sets.
	//   Pass 2 -- named xsd:element inline content models; resolves type= references from Pass 1
	//             so elements like centerOfMassTransform (type="transform") correctly inherit
	//             transform's content model instead of getting an empty set.
	// Elements whose type= references a type with no content model get no entry -- permissive.
	static std::unordered_map<std::string, std::unordered_set<std::string>>
		parseAllowedChildren(const pugi::xml_document& doc, std::unordered_set<std::string>& knownElements)
	{
		pugi::xml_node schema = doc.first_child();

		// Pass 1: named xsd:complexType → content-model child names.
		std::unordered_map<std::string, std::unordered_set<std::string>> typeChildren;
		for (auto ct : schema.children("xsd:complexType")) {
			const char* typeName = ct.attribute("name").as_string("");
			if (!typeName[0]) continue;
			std::unordered_set<std::string> children;
			walkContentModel(ct, children);
			if (!children.empty()) {
				for (const auto& c : children)
					knownElements.insert(c);
				typeChildren[typeName] = std::move(children);
			}
		}

		// Pass 2: named xsd:element → inline content model or type= reference resolution.
		std::unordered_map<std::string, std::unordered_set<std::string>> result;
		for (auto elem : schema.children("xsd:element")) {
			const char* name = elem.attribute("name").as_string("");
			if (!name[0]) continue;
			knownElements.insert(name);  // Every top-level named element is "known".
			const char* typeName = elem.attribute("type").as_string("");
			if (typeName[0]) {
				const auto it = typeChildren.find(typeName);
				if (it != typeChildren.end())
					result[name] = it->second;
				// else: type has no content model (simpleContent) -- no entry -> permissive
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

	// Maps an XSD built-in type name (with namespace prefix) to a TypeConstraint.
	// Returns Base::Any for unconstrained or unknown types.
	static TypeConstraint builtinTypeConstraint(const std::string& typeName)
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

	// Resolves a type name against named simpleTypes first, then built-in XSD types.
	static TypeConstraint resolveTypeConstraint(
		const std::string& typeName,
		const std::unordered_map<std::string, TypeConstraint>& namedTypes)
	{
		const auto it = namedTypes.find(typeName);
		return (it != namedTypes.end()) ? it->second : builtinTypeConstraint(typeName);
	}

	// Recursively gathers all xsd:attribute type constraints from a complexType subtree.
	// Extracted from parseAllTypeConstraints to avoid a std::function recursive lambda.
	static void gatherComplexTypeAttrs(
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

	// Definition of collectTypeConstraints (forward-declared above).
	// Recursively walks `node` to collect xsd:attribute type constraints and
	// xsd:extension/xsd:restriction text-content type constraints.
	// When inside a compositor (xsd:choice / xsd:sequence / xsd:all) any xsd:element
	// children are content-model references and are NOT attribute/text-content definitions,
	// so they are skipped.
	static void collectTypeConstraints(
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
				// Content-model element references inside a compositor: skip.
				if (inCompositor) continue;
				collectTypeConstraints(child, inCompositor, elemName, namedSimpleTypes, textCons, attrCons);
			} else {
				collectTypeConstraints(child, inCompositor, elemName, namedSimpleTypes, textCons, attrCons);
			}
		}
	}

	// Parse all value type constraints from the XSD -- three passes:
	//   Pass 1: named xsd:simpleType restrictions -> TypeConstraint (base type + optional range).
	//   Pass 2: named xsd:complexType attribute declarations -> { attr name -> TypeConstraint }.
	//   Pass 3: named top-level xsd:element declarations -> elementTextConstraints
	//           and elementAttrConstraints (inline or inherited from complexType in Pass 2).
	// Generic -- no element, type, or attribute names are hardcoded.
	static void parseAllTypeConstraints(
		const pugi::xml_document& doc,
		std::unordered_map<std::string, TypeConstraint>& elementTextConstraints,
		std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>>& elementAttrConstraints)
	{
		pugi::xml_node schema = doc.first_child();

		// Pass 1: named xsd:simpleType → TypeConstraint.
		std::unordered_map<std::string, TypeConstraint> namedSimpleTypes;
		for (auto st : schema.children("xsd:simpleType")) {
			const char* typeName = st.attribute("name").as_string("");
			if (!typeName[0]) continue;
			TypeConstraint tc;
			for (auto child : st.children()) {
				if (std::string_view(child.name()) != "xsd:restriction") continue;
				const char* base = child.attribute("base").as_string("");
				if (base[0]) tc = resolveTypeConstraint(base, namedSimpleTypes);
				for (auto facet : child.children()) {
					const std::string_view ftag = facet.name();
					if (ftag == "xsd:minInclusive") {
						std::string_view vstr(facet.attribute("value").as_string(""));
						double v;
						if (std::from_chars(vstr.data(), vstr.data() + vstr.size(), v).ec == std::errc{})
							{ tc.minInclusive = v; tc.hasMin = true; }
					} else if (ftag == "xsd:maxInclusive") {
						std::string_view vstr(facet.attribute("value").as_string(""));
						double v;
						if (std::from_chars(vstr.data(), vstr.data() + vstr.size(), v).ec == std::errc{})
							{ tc.maxInclusive = v; tc.hasMax = true; }
					}
				}
			}
			namedSimpleTypes[typeName] = tc;
		}

		// Pass 2: named xsd:complexType → { attribute name → TypeConstraint }.
		std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>> namedComplexTypeAttrs;
		for (auto ct : schema.children("xsd:complexType")) {
			const char* typeName = ct.attribute("name").as_string("");
			if (!typeName[0]) continue;
			std::unordered_map<std::string, TypeConstraint> attrCons;
			gatherComplexTypeAttrs(ct, namedSimpleTypes, attrCons);
			if (!attrCons.empty())
				namedComplexTypeAttrs[typeName] = std::move(attrCons);
		}

		// Pass 3: named top-level xsd:element → text and attr constraints.
		for (auto elem : schema.children("xsd:element")) {
			const char* name = elem.attribute("name").as_string("");
			if (!name[0]) continue;
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

	static PhysicsSchema g_physicsSchema;
	static std::once_flag g_schemaOnce;

	static const PhysicsSchema& getPhysicsSchema()
	{
		std::call_once(g_schemaOnce, []() {
			// Use load_file (direct filesystem) only: schema files are always on disk
			// and readAllFile (BSA VFS) is unsafe before BSAs are mounted during SKSEPlugin_Load.
			pugi::xml_document doc;
			auto res = doc.load_file(kPhysicsXSDPath);

			if (!res) {
				logger::error(
					"[XSDValidator] Could not load physics schema from '{}': {}; "
					"physics XML validation will be skipped.",
					kPhysicsXSDPath, res.description());
				return;
			}

			try {
				// element enums — all xsd:element nodes with inline anonymous simpleType enumerations
				g_physicsSchema.elementEnums = parseAllElementEnumerations(doc);

				// The root element of every physics XML is always <system>.
				// Do not derive this from the XSD — other top-level xsd:element declarations
				// (e.g. angularBounce) appear before <system> and would be picked incorrectly.
				g_physicsSchema.rootTag = "system";
				// Key/unique/keyref constraints — drive generic referential integrity checks.
				parseKeyConstraints(doc, g_physicsSchema.keyDefs, g_physicsSchema.keyRefDefs);
				// Required attributes per element — driven by xsd:attribute use="required".
				g_physicsSchema.requiredAttrs = parseAllRequiredAttrs(doc);
				// Allowed children per element — driven by the XSD content model.
				// Also populates knownElements with all element names mentioned in the schema.
				g_physicsSchema.allowedChildren =
					parseAllowedChildren(doc, g_physicsSchema.knownElements);
				// Element text and attribute value constraints -- driven by XSD type system.
				parseAllTypeConstraints(doc,
					g_physicsSchema.elementTextConstraints,
					g_physicsSchema.elementAttrConstraints);
				g_physicsSchema.loaded = true;

				logger::info(
					"[XSDValidator] Loaded physics schema: root '{}', "
					"{} enumerated element type(s), {} elements with required attr(s), "
					"{} elements with allowed-children constraint(s), "
					"{} with text type constraint(s), {} with attr type constraint(s).",
					g_physicsSchema.rootTag,
					g_physicsSchema.elementEnums.size(),
					g_physicsSchema.requiredAttrs.size(),
					g_physicsSchema.allowedChildren.size(),
					g_physicsSchema.elementTextConstraints.size(),
					g_physicsSchema.elementAttrConstraints.size());
			} catch (const std::exception& e) {
				logger::warn("[XSDValidator] Failed to parse physics schema '{}': {}",
					kPhysicsXSDPath, e.what());
			} catch (...) {
				logger::warn("[XSDValidator] Failed to parse physics schema '{}'.",
					kPhysicsXSDPath);
			}
		});

		return g_physicsSchema;
	}

	// ---- public schema accessors ----

	bool IsPhysicsSchemaLoaded()
	{
		return getPhysicsSchema().loaded;
	}

	const std::unordered_map<std::string, std::unordered_set<std::string>>& GetSchemaAllowedChildren()
	{
		return getPhysicsSchema().allowedChildren;
	}

	const std::unordered_set<std::string>& GetSchemaKnownElements()
	{
		return getPhysicsSchema().knownElements;
	}

	// ---- validation context ----

	struct ValidationContext
	{
		std::string xmlPath;
		std::vector<XSDViolation>& violations;
		std::vector<std::string> elementStack;
		// Generic referential integrity tracking — driven by XSD key/unique/keyref.
		// keyValues: key/unique name — set of declared values encountered during parsing.
		// keyRefPending: keyref name — list of (line, value) pairs to verify after parsing.
		std::unordered_map<std::string, std::unordered_set<std::string>> keyValues;
		std::unordered_map<std::string, std::vector<std::pair<uint64_t, std::string>>> keyRefPending;

		void addViolation(uint64_t line, uint64_t col, const std::string& msg)
		{
			std::string path;
			for (const auto& e : elementStack) {
				path += "/" + e;
			}
			violations.push_back({ xmlPath, line, col, path, msg });
		}
	};

	// ---- element validators ----

	// Validates that val satisfies a TypeConstraint derived from the XSD.
	// Trims leading/trailing whitespace before checking.  Reports a violation if the
	// value does not parse as the expected base type or falls outside the declared range.
	static void validateTypedValue(
		const std::string& tag, const std::string& attrName,
		const std::string& rawVal, const TypeConstraint& tc,
		uint64_t row, uint64_t col, ValidationContext& ctx)
	{
		const auto b = rawVal.find_first_not_of(" \t\r\n");
		const auto e = rawVal.find_last_not_of(" \t\r\n");
		const std::string val = (b == std::string::npos) ? "" : rawVal.substr(b, e - b + 1);

		const std::string where = attrName.empty() ? "<" + tag + ">" : "<" + tag + "> attribute '" + attrName + "'";

		if (tc.base == TypeConstraint::Base::Boolean) {
			if (val != "true" && val != "false" && val != "1" && val != "0")
				ctx.addViolation(row, col, where + " has invalid boolean value '" + val + "'");
			return;
		}

		if (tc.base == TypeConstraint::Base::Float || tc.base == TypeConstraint::Base::Integer) {
			double dval = 0.0;
			const auto [ptr, ec] = std::from_chars(val.data(), val.data() + val.size(), dval);
			if (ec != std::errc{} || ptr != val.data() + val.size()) {
				const std::string typeName = (tc.base == TypeConstraint::Base::Float) ? "float" : "integer";
				ctx.addViolation(row, col, where + " has invalid " + typeName + " value '" + val + "'");
				return;
			}
			if (tc.hasMin && dval < tc.minInclusive)
				ctx.addViolation(row, col, where + " value " + val + " is below minimum " + std::to_string(tc.minInclusive));
			if (tc.hasMax && dval > tc.maxInclusive)
				ctx.addViolation(row, col, where + " value " + val + " is above maximum " + std::to_string(tc.maxInclusive));
		}
	}

	// Reads and validates all children of parentTag until its closing EndTag.
	// Assumes the opening StartTag of parentTag has already been consumed.
	// Validates child elements against the XSD-derived allowed-children set, tracks
	// key/keyref referential integrity, enforces required attributes, and validates
	// enum-constrained text content.  Fully recursive — handles arbitrary nesting depth.
	static void validateChildren(
		const std::string& parentTag, XMLReader& reader,
		ValidationContext& ctx, const PhysicsSchema& schema)
	{
		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::EndTag)
				return;
			if (reader.GetInspected() != XMLReader::Inspected::StartTag)
				continue;

			const std::string tag = reader.GetLocalName();

			// Check that this element is allowed inside parentTag (XSD content model).
			const auto parentIt = schema.allowedChildren.find(parentTag);
			if (parentIt != schema.allowedChildren.end() &&
				!parentIt->second.count(tag)) {
				ctx.addViolation(reader.GetRow(), reader.GetColumn(),
					"<" + tag + "> is not allowed inside <" + parentTag + ">");
				reader.skipCurrentElement();
				continue;
			}

			// Generic key/keyref tracking (XSD-driven referential integrity).
			for (const auto& [kn, kd] : schema.keyDefs) {
				if (kd.elems.count(tag) && reader.hasAttribute(kd.fieldAttr))
					ctx.keyValues[kn].insert(reader.getAttribute(kd.fieldAttr));
			}
			for (const auto& [rn, rd] : schema.keyRefDefs) {
				if (rd.elems.count(tag) && reader.hasAttribute(rd.fieldAttr))
					ctx.keyRefPending[rn].emplace_back(reader.GetRow(), reader.getAttribute(rd.fieldAttr));
			}

			ctx.elementStack.push_back(tag);

			// Generic required attribute check (XSD-driven, no hardcoded names).
			if (schema.requiredAttrs.count(tag)) {
				for (const auto& attr : schema.requiredAttrs.at(tag)) {
					if (!reader.hasAttribute(attr))
						ctx.addViolation(reader.GetRow(), reader.GetColumn(),
							"<" + tag + "> is missing required attribute '" + attr + "'");
				}
			}

			// Attribute value type validation (XSD-driven, no hardcoded names).
			const auto attrConstraintIt = schema.elementAttrConstraints.find(tag);
			if (attrConstraintIt != schema.elementAttrConstraints.end()) {
				const uint64_t row = reader.GetRow(), col = reader.GetColumn();
				for (const auto& [attrName, tc] : attrConstraintIt->second) {
					if (reader.hasAttribute(attrName))
						validateTypedValue(tag, attrName, reader.getAttribute(attrName), tc, row, col, ctx);
				}
			}

			// Enum-constrained text content — read and validate; otherwise recurse into children.
			if (schema.elementEnums.count(tag)) {
				const uint64_t row = reader.GetRow(), col = reader.GetColumn();
				const std::string val = reader.readText();
				const auto& allowed = schema.elementEnums.at(tag);
				if (!allowed.count(val))
					ctx.addViolation(row, col,
						"<" + tag + "> has invalid value '" + val +
							"' (see hdtSMP64.xsd for valid values: " + joinSet(allowed) + ")");
			} else if (schema.elementTextConstraints.count(tag)) {
				const uint64_t row = reader.GetRow(), col = reader.GetColumn();
				const std::string val = reader.readText();
				validateTypedValue(tag, "", val, schema.elementTextConstraints.at(tag), row, col, ctx);
			} else {
				validateChildren(tag, reader, ctx, schema);
			}

			ctx.elementStack.pop_back();
		}
	}

	static void validateSystem(XMLReader& reader, ValidationContext& ctx)
	{
		const PhysicsSchema& schema = getPhysicsSchema();
		ctx.elementStack.push_back(schema.rootTag);
		validateChildren(schema.rootTag, reader, ctx, schema);
		ctx.elementStack.pop_back();
	}

	// ---- public API ----

	XSDValidationResult ValidatePhysicsXML(const std::string& xmlPath)
	{
		XSDValidationResult result;

		// Skip validation entirely if the physics schema could not be loaded.
		// The error was already logged once by getPhysicsSchema().
		if (!getPhysicsSchema().loaded) {
			result.isValid = true;
			return result;
		}

		// Use readAllFile2 (direct filesystem) only: physics XML files are on disk
		// and readAllFile (BSA VFS) is unsafe before BSAs are mounted during SKSEPlugin_Load.
		auto bytes = readAllFile2(xmlPath.c_str());

		if (bytes.empty()) {
			result.isValid = false;
			result.violations.push_back({ xmlPath, 0, 0, "", "File not found or empty" });
			return result;
		}

		const PhysicsSchema& schema = getPhysicsSchema();
		ValidationContext ctx{
			xmlPath,
			result.violations,
			{}
		};

		try {
			XMLReader reader((uint8_t*)bytes.data(), bytes.size());
			bool foundSystem = false;

			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
					const std::string tag = reader.GetLocalName();
					if (tag == schema.rootTag) {
						foundSystem = true;
						validateSystem(reader, ctx);
						// Referential integrity — check all pending keyref values against declared keys.
						const std::unordered_set<std::string> emptySet;
						for (const auto& [refName, refDef] : schema.keyRefDefs) {
							const auto pendIt = ctx.keyRefPending.find(refName);
							if (pendIt == ctx.keyRefPending.end())
								continue;
							const auto keyIt = ctx.keyValues.find(refDef.refer);
							const auto& declared =
								(keyIt != ctx.keyValues.end()) ? keyIt->second : emptySet;
							for (const auto& [line, val] : pendIt->second) {
								if (!val.empty() && !declared.count(val)) {
									result.violations.push_back({ xmlPath, line, 0,
										"/" + schema.rootTag,
										"Undeclared reference '" + val + "' (must be declared in '" +
											refDef.refer + "')" });
								}
							}
						}
					} else {
						ctx.addViolation(reader.GetRow(), reader.GetColumn(),
							"Root element must be <" + schema.rootTag + ">, found <" + tag + ">");
						reader.skipCurrentElement();
					}
				}
			}

			if (!foundSystem) {
				result.violations.push_back({ xmlPath, 0, 0, "/", "No <" + schema.rootTag + "> root element found" });
			}
		} catch (const std::exception& e) {
			result.violations.push_back(
				{ xmlPath, 0, 0, "", std::string("XML parse error: ") + e.what() });
		} catch (...) {
			result.violations.push_back({ xmlPath, 0, 0, "", "Unknown XML parse error" });
		}

		result.isValid = result.violations.empty();
		return result;
	}

}  // namespace hdt
