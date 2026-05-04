#include "hdtXSDValidator.h"

#include "NetImmerseUtils.h"
#include "XmlReader.h"

#include <algorithm>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hdt
{
	// Path to the hdtSMP64 XSD schema, relative to the game working directory.
	// This is the same schema shipped with the FSMP-Validator tool and already
	// present in users' data folders. The validator reads accepted enum values
	// from it at startup so they never need to be hardcoded here.
	static const char* kPhysicsXSDPath =
		"data/skse/plugins/hdtSkinnedMeshConfigs/hdtSMP64.xsd";

	// Generic value type constraint derived from xs:restriction facets in the XSD.
	// All fields are populated from the XSD at load time -- no type names are hardcoded.
	struct TypeConstraint
	{
		enum class Base { Any, Float, Boolean, Integer } base = Base::Any;
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
		struct KeyDef { std::unordered_set<std::string> elems; std::string fieldAttr; };
		struct KeyRefDef { std::string refer; std::unordered_set<std::string> elems; std::string fieldAttr; };
		std::unordered_map<std::string, KeyDef> keyDefs;       // key/unique name — declared value sets
		std::unordered_map<std::string, KeyRefDef> keyRefDefs; // keyref name — reference definitions
		// Allowed children per element — maps each named xs:element to the set of child element
		// tag names declared in its XSD content model (xs:choice/xs:all/xs:sequence).
		std::unordered_map<std::string, std::unordered_set<std::string>> allowedChildren;
		// Per element: type constraint on its text content for non-enum typed elements.
		std::unordered_map<std::string, TypeConstraint> elementTextConstraints;
		// Per element: per-attribute type constraint on attribute values.
		std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>> elementAttrConstraints;
		bool loaded = false;  // true only when the XSD was successfully parsed
	};

	// Build a comma-separated string from a set of strings (for error messages).
	static std::string joinSet(const std::unordered_set<std::string>& s)
	{
		std::string result;
		for (const auto& v : s) {
			if (!result.empty()) result += ", ";
			result += v;
		}
		return result;
	}

	// Collect all xs:element enum constraints from the XSD — both inline anonymous
	// simpleType enumerations and elements with a type="..." attribute referencing
	// a named simpleType that itself has enumeration constraints.
	// Returns a map from element name to its set of allowed values.
	// No element, type, or attribute names are hardcoded.
	static std::unordered_map<std::string, std::unordered_set<std::string>>
		parseAllElementEnumerations(std::string& bytes)
	{
		// Pass 1: collect named simpleType -> enum values.
		std::unordered_map<std::string, std::unordered_set<std::string>> namedEnums;
		{
			XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());
			bool inType = false;
			std::string currentType;
			int depth = 0;
			std::unordered_set<std::string> currentEnums;
			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
					const std::string localName = reader.GetLocalName();
					if (!inType) {
						if (localName == "simpleType" && reader.hasAttribute("name")) {
							inType = true;
							currentType = reader.getAttribute("name");
							depth = 0;
							currentEnums.clear();
						}
					} else {
						++depth;
						if (localName == "enumeration" && reader.hasAttribute("value"))
							currentEnums.insert(reader.getAttribute("value"));
					}
				} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
					if (inType) {
						if (depth == 0) {
							if (!currentEnums.empty())
								namedEnums[currentType] = std::move(currentEnums);
							currentEnums.clear();
							inType = false;
							currentType.clear();
						} else --depth;
					}
				}
			}
		}

		// Pass 2: collect inline enum values and type= references from xs:element.
		std::unordered_map<std::string, std::unordered_set<std::string>> result;
		std::unordered_map<std::string, std::string> typeRefs; // element name -> named type
		{
			XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());
			bool inElement = false;
			std::string currentElement;
			int depth = 0;
			std::unordered_set<std::string> currentEnums;
			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
					const std::string localName = reader.GetLocalName();
					if (!inElement) {
						if (localName == "element" && reader.hasAttribute("name")) {
							inElement = true;
							currentElement = reader.getAttribute("name");
							depth = 0;
							currentEnums.clear();
							if (reader.hasAttribute("type")) {
								const std::string typeName = reader.getAttribute("type");
								if (namedEnums.count(typeName))
									typeRefs[currentElement] = typeName;
							}
						}
					} else {
						++depth;
						if (localName == "enumeration" && reader.hasAttribute("value"))
							currentEnums.insert(reader.getAttribute("value"));
					}
				} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
					if (inElement) {
						if (depth == 0) {
							if (!currentEnums.empty())
								result[currentElement] = std::move(currentEnums);
							currentEnums.clear();
							inElement = false;
							currentElement.clear();
						} else --depth;
					}
				}
			}
		}

		// Resolve type= references: copy named simpleType enum values into result.
		for (const auto& [elem, typeName] : typeRefs) {
			const auto& vals = namedEnums.at(typeName);
			result[elem].insert(vals.begin(), vals.end());
		}

		return result;
	}

	// Build a map from every xs:element name to its list of xs:attribute use="required" names.
	// This is fully generic: no element or attribute names are hardcoded.
	static std::unordered_map<std::string, std::vector<std::string>>
		parseAllRequiredAttrs(std::string& bytes)
	{
		std::unordered_map<std::string, std::vector<std::string>> result;
		XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());

		bool inElement = false;
		std::string currentElementName;
		int depth = 0;

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
				const std::string localName = reader.GetLocalName();

				if (!inElement) {
					if (localName == "element" && reader.hasAttribute("name")) {
						inElement = true;
						currentElementName = reader.getAttribute("name");
						depth = 0;
					}
				} else {
					++depth;
					if (localName == "attribute" &&
						reader.hasAttribute("name") &&
						reader.hasAttribute("use") && reader.getAttribute("use") == "required") {
						result[currentElementName].push_back(reader.getAttribute("name"));
					}
				}
			} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				if (inElement) {
					if (depth == 0) {
						inElement = false;
						currentElementName.clear();
					} else {
						--depth;
					}
				}
			}
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
				if (start != std::string::npos) result.insert(token.substr(start));
				token.clear();
			} else {
				token += c;
			}
		}
		size_t start = token.find_first_not_of(" \t./");
		if (start != std::string::npos) result.insert(token.substr(start));
		return result;
	}

	// Find the first top-level xs:element in the XSD (the root element of physics XML files)
	// and collect its xs:key name — selector xpath pairs.
	// Returns {rootElementName, {keyName: selectorXpath}} with no hardcoded names.
	static std::pair<std::string, std::unordered_map<std::string, std::string>>
		parseSystemKeys(std::string& bytes)
	{
		std::unordered_map<std::string, std::string> keys;
		std::string rootTag;
		XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());

		bool inSchema = false;
		bool inRoot = false;
		bool inKey = false;
		int schemaDepth = 0; // depth inside xs:schema, outside the root element
		int rootDepth = 0;   // depth inside the root element
		int keyDepth = 0;    // depth inside a xs:key element
		std::string currentKeyName;

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
				const std::string localName = reader.GetLocalName();

				if (!inSchema) {
					if (localName == "schema") inSchema = true;
				} else if (!inRoot) {
					// At depth 0 inside xs:schema: the first xs:element with a name is the root.
					if (schemaDepth == 0 && localName == "element" && reader.hasAttribute("name")) {
						rootTag = reader.getAttribute("name");
						inRoot = true;
						rootDepth = 0;
					} else {
						++schemaDepth; // skip non-root top-level constructs
					}
				} else if (!inKey) {
					++rootDepth;
					if (localName == "key" && reader.hasAttribute("name")) {
						inKey = true;
						currentKeyName = reader.getAttribute("name");
						keyDepth = 0;
					}
				} else {
					++keyDepth;
					if (localName == "selector" && reader.hasAttribute("xpath")) {
						keys[currentKeyName] = reader.getAttribute("xpath");
					}
				}
			} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				if (inKey) {
					if (keyDepth == 0) { inKey = false; --rootDepth; }
					else --keyDepth;
				} else if (inRoot) {
					if (rootDepth == 0) break; // done with root element
					--rootDepth;
				} else if (inSchema) {
					if (schemaDepth == 0) break; // end of xs:schema
					--schemaDepth;
				}
			}
		}

		return { rootTag, keys };
	}

	// Parse all xsd:key, xsd:unique, and xsd:keyref declarations from the XSD.
	// key/unique declarations establish which attribute values are valid keys.
	// keyref declarations establish which attributes must reference a declared key.
	static void parseKeyConstraints(
		std::string& bytes,
		std::unordered_map<std::string, PhysicsSchema::KeyDef>& keyDefs,
		std::unordered_map<std::string, PhysicsSchema::KeyRefDef>& keyRefDefs)
	{
		XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());
		int mode = 0; // 0 = none, 1 = key/unique, 2 = keyref
		int depth = 0;
		std::string curName, curRefer, curSelector, curField;

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
				const std::string ln = reader.GetLocalName();
				if (mode == 0) {
					if ((ln == "key" || ln == "unique") && reader.hasAttribute("name")) {
						mode = 1; depth = 0;
						curName = reader.getAttribute("name");
						curSelector.clear(); curField.clear();
					} else if (ln == "keyref" &&
							reader.hasAttribute("name") && reader.hasAttribute("refer")) {
						mode = 2; depth = 0;
						curName = reader.getAttribute("name");
						curRefer = reader.getAttribute("refer");
						curSelector.clear(); curField.clear();
					}
				} else {
					++depth;
					if (ln == "selector" && reader.hasAttribute("xpath"))
						curSelector = reader.getAttribute("xpath");
					else if (ln == "field" && reader.hasAttribute("xpath")) {
						const std::string fp = reader.getAttribute("xpath");
						curField = (!fp.empty() && fp[0] == '@') ? fp.substr(1) : fp;
					}
				}
			} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				if (mode != 0) {
					if (depth == 0) {
						if (!curSelector.empty() && !curField.empty()) {
							if (mode == 1)
								keyDefs[curName] = { parseXPathElems(curSelector), curField };
							else
								keyRefDefs[curName] = { curRefer, parseXPathElems(curSelector), curField };
						}
						mode = 0;
					} else {
						--depth;
					}
				}
			}
		}
	}

	// Collect the set of allowed child element names for each named xs:element in the XSD.
	// Two passes:
	//   Pass 1 -- named xs:complexType content models -> child name sets.
	//   Pass 2 -- named xs:element inline content models; resolves type= references from Pass 1
	//             so elements like centerOfMassTransform (type="transform") correctly inherit
	//             transform's content model instead of getting an empty set.
	// Elements whose type= references a type with no content model get no entry -- permissive.
	static std::unordered_map<std::string, std::unordered_set<std::string>>
		parseAllowedChildren(std::string& bytes)
	{
		// Pass 1: named xs:complexType -- collect content-model children.
		std::unordered_map<std::string, std::unordered_set<std::string>> complexTypeChildren;
		{
			XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());
			bool inType = false;
			std::string currentType;
			std::unordered_set<std::string> children;
			bool hasContentModel = false;
			int depth = 0;

			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
					const std::string ln = reader.GetLocalName();
					if (!inType) {
						if (ln == "complexType" && reader.hasAttribute("name")) {
							inType = true;
							currentType = reader.getAttribute("name");
							depth = 0;
							children.clear();
							hasContentModel = false;
						}
					} else {
						if (ln == "element" && depth >= 1) {
							std::string childName;
							if (reader.hasAttribute("ref"))
								childName = reader.getAttribute("ref");
							else if (reader.hasAttribute("name"))
								childName = reader.getAttribute("name");
							if (!childName.empty()) { hasContentModel = true; children.insert(childName); }
							reader.skipCurrentElement();
						} else { ++depth; }
					}
				} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
					if (inType) {
						if (depth == 0) {
							if (hasContentModel) complexTypeChildren[currentType] = std::move(children);
							children.clear(); inType = false; currentType.clear();
						} else { --depth; }
					}
				}
			}
		}

		// Pass 2: named xs:elements -- inline content models + type= reference resolution.
		std::unordered_map<std::string, std::unordered_set<std::string>> result;
		std::unordered_map<std::string, std::string> typeRefs;
		{
			XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());
			bool inElement = false;
			std::string currentParent;
			std::unordered_set<std::string> children;
			bool hasContentModel = false;
			int depth = 0;

			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
					const std::string ln = reader.GetLocalName();
					if (!inElement) {
						if (ln == "element" && reader.hasAttribute("name")) {
							inElement = true;
							currentParent = reader.getAttribute("name");
							depth = 0;
							children.clear();
							hasContentModel = false;
							if (reader.hasAttribute("type")) {
								typeRefs[currentParent] = reader.getAttribute("type");
								reader.skipCurrentElement(); inElement = false;
							}
						}
					} else {
						if (ln == "element" && depth >= 2) {
							std::string childName;
							if (reader.hasAttribute("ref"))
								childName = reader.getAttribute("ref");
							else if (reader.hasAttribute("name"))
								childName = reader.getAttribute("name");
							if (!childName.empty()) { hasContentModel = true; children.insert(childName); }
							reader.skipCurrentElement(); // consume child + subtree; do not increment depth
						} else { ++depth; }
					}
				} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
					if (inElement) {
						if (depth == 0) {
							if (hasContentModel) result[currentParent] = std::move(children);
							children.clear(); inElement = false; currentParent.clear();
						} else { --depth; }
					}
				}
			}
		}

		// Resolve type= references: inherit content model from the named complexType.
		for (const auto& [elemName, typeName] : typeRefs) {
			const auto it = complexTypeChildren.find(typeName);
			if (it != complexTypeChildren.end())
				result[elemName] = it->second;
			// else: type has no content model (simpleContent) -- no entry -> permissive
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

	// Parse all value type constraints from the XSD -- three passes:
	//   Pass 1: named xs:simpleType restrictions -> TypeConstraint (base type + optional range).
	//   Pass 2: named xs:complexType attribute declarations -> { attr name -> TypeConstraint }.
	//   Pass 3: named top-level xs:element declarations -> elementTextConstraints
	//           and elementAttrConstraints (inline or inherited from complexType in Pass 2).
	// Generic -- no element, type, or attribute names are hardcoded.
	static void parseAllTypeConstraints(
		std::string& bytes,
		std::unordered_map<std::string, TypeConstraint>& elementTextConstraints,
		std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>>& elementAttrConstraints)
	{
		// Pass 1: named xs:simpleType -> TypeConstraint.
		std::unordered_map<std::string, TypeConstraint> namedSimpleTypes;
		{
			XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());
			bool inType = false;
			std::string currentType;
			TypeConstraint currentTc;
			int depth = 0;

			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
					const std::string ln = reader.GetLocalName();
					if (!inType) {
						if (ln == "simpleType" && reader.hasAttribute("name")) {
							inType = true;
							currentType = reader.getAttribute("name");
							currentTc = TypeConstraint{};
							depth = 0;
						}
					} else {
						if (ln == "restriction" && reader.hasAttribute("base"))
							currentTc = builtinTypeConstraint(reader.getAttribute("base"));
						else if (ln == "minInclusive" && reader.hasAttribute("value")) {
							try { currentTc.minInclusive = std::stod(reader.getAttribute("value")); currentTc.hasMin = true; } catch (...) {}
						} else if (ln == "maxInclusive" && reader.hasAttribute("value")) {
							try { currentTc.maxInclusive = std::stod(reader.getAttribute("value")); currentTc.hasMax = true; } catch (...) {}
						}
						++depth;
					}
				} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
					if (inType) {
						if (depth == 0) { namedSimpleTypes[currentType] = currentTc; inType = false; currentType.clear(); }
						else { --depth; }
					}
				}
			}
		}

		// Pass 2: named xs:complexType -> { attribute name -> TypeConstraint }.
		std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>> namedComplexTypeAttrs;
		{
			XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());
			bool inType = false;
			std::string currentType;
			int depth = 0;

			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
					const std::string ln = reader.GetLocalName();
					if (!inType) {
						if (ln == "complexType" && reader.hasAttribute("name")) {
							inType = true;
							currentType = reader.getAttribute("name");
							depth = 0;
						}
					} else {
						if (ln == "attribute" && reader.hasAttribute("name") && reader.hasAttribute("type")) {
							TypeConstraint tc = resolveTypeConstraint(reader.getAttribute("type"), namedSimpleTypes);
							if (tc.base != TypeConstraint::Base::Any)
								namedComplexTypeAttrs[currentType][reader.getAttribute("name")] = tc;
						}
						++depth;
					}
				} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
					if (inType) {
						if (depth == 0) { inType = false; currentType.clear(); }
						else { --depth; }
					}
				}
			}
		}

		// Pass 3: named top-level xs:element declarations -> text and attr constraints.
		{
			XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());
			bool inElement = false;
			std::string currentElem;
			int depth = 0;

			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
					const std::string ln = reader.GetLocalName();
					if (!inElement) {
						if (ln == "element" && reader.hasAttribute("name")) {
							inElement = true;
							currentElem = reader.getAttribute("name");
							depth = 0;
							if (reader.hasAttribute("type")) {
								const std::string typeName = reader.getAttribute("type");
								TypeConstraint tc = resolveTypeConstraint(typeName, namedSimpleTypes);
								if (tc.base != TypeConstraint::Base::Any) elementTextConstraints[currentElem] = tc;
								const auto cit = namedComplexTypeAttrs.find(typeName);
								if (cit != namedComplexTypeAttrs.end()) elementAttrConstraints[currentElem] = cit->second;
								reader.skipCurrentElement(); inElement = false;
							}
						}
					} else {
						if (ln == "element" && depth >= 2) {
							reader.skipCurrentElement(); // content-model child, not an attr/text decl
						} else {
							if (ln == "attribute" && reader.hasAttribute("name") && reader.hasAttribute("type")) {
								TypeConstraint tc = resolveTypeConstraint(reader.getAttribute("type"), namedSimpleTypes);
								if (tc.base != TypeConstraint::Base::Any)
									elementAttrConstraints[currentElem][reader.getAttribute("name")] = tc;
							}
							if ((ln == "extension" || ln == "restriction") && reader.hasAttribute("base")) {
								TypeConstraint tc = resolveTypeConstraint(reader.getAttribute("base"), namedSimpleTypes);
								if (tc.base != TypeConstraint::Base::Any) elementTextConstraints[currentElem] = tc;
							}
							++depth;
						}
					}
				} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
					if (inElement) {
						if (depth == 0) { inElement = false; currentElem.clear(); }
						else { --depth; }
					}
				}
			}
		}
	}

	static PhysicsSchema g_physicsSchema;
	static std::once_flag g_schemaOnce;

	static const PhysicsSchema& getPhysicsSchema()
	{
		std::call_once(g_schemaOnce, []() {
			// Use readAllFile2 (direct filesystem) only: schema files are always on disk
			// and readAllFile (BSA VFS) is unsafe before BSAs are mounted during SKSEPlugin_Load.
			auto bytes = readAllFile2(kPhysicsXSDPath);

			if (bytes.empty()) {
				logger::error("[XSDValidator] Could not load physics schema from '{}'; "
							  "physics XML validation will be skipped.",
					kPhysicsXSDPath);
				return;
			}

			try {
				// element enums — all xs:element nodes with inline anonymous simpleType enumerations
				g_physicsSchema.elementEnums = parseAllElementEnumerations(bytes);

				// Root tag derived from the first top-level xs:element in the XSD.
				g_physicsSchema.rootTag = parseSystemKeys(bytes).first;
				// Key/unique/keyref constraints — drive generic referential integrity checks.
				parseKeyConstraints(bytes, g_physicsSchema.keyDefs, g_physicsSchema.keyRefDefs);
				// Required attributes per element — driven by xs:attribute use="required".
				g_physicsSchema.requiredAttrs = parseAllRequiredAttrs(bytes);
				// Allowed children per element — driven by the XSD content model.
				g_physicsSchema.allowedChildren = parseAllowedChildren(bytes);
				// Element text and attribute value constraints -- driven by XSD type system.
				parseAllTypeConstraints(bytes,
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

		const std::string where = attrName.empty()
			? "<" + tag + ">"
			: "<" + tag + "> attribute '" + attrName + "'";

		if (tc.base == TypeConstraint::Base::Boolean) {
			if (val != "true" && val != "false" && val != "1" && val != "0")
				ctx.addViolation(row, col, where + " has invalid boolean value '" + val + "'");
			return;
		}

		if (tc.base == TypeConstraint::Base::Float || tc.base == TypeConstraint::Base::Integer) {
			size_t idx = 0;
			double dval = 0.0;
			try { dval = std::stod(val, &idx); } catch (...) { idx = 0; }
			if (idx != val.size()) {
				const std::string typeName = (tc.base == TypeConstraint::Base::Float) ? "float" : "integer";
				ctx.addViolation(row, col, where + " has invalid " + typeName + " value '" + val + "'");
				return;
			}
			if (tc.hasMin && dval < tc.minInclusive)
				ctx.addViolation(row, col, where + " value " + val +
					" is below minimum " + std::to_string(tc.minInclusive));
			if (tc.hasMax && dval > tc.maxInclusive)
				ctx.addViolation(row, col, where + " value " + val +
					" is above maximum " + std::to_string(tc.maxInclusive));
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
			if (reader.GetInspected() == XMLReader::Inspected::EndTag) return;
			if (reader.GetInspected() != XMLReader::Inspected::StartTag) continue;

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

		// Store original locale and switch to "C" for numeric parsing.
		// "C" locale is guaranteed available on all systems and provides
		// the same dot-as-decimal-separator behaviour needed for XML values.
		char saved_locale[64];
		strcpy_s(saved_locale, std::setlocale(LC_NUMERIC, nullptr));
		std::setlocale(LC_NUMERIC, "C");

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
							if (pendIt == ctx.keyRefPending.end()) continue;
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

		std::setlocale(LC_NUMERIC, saved_locale);

		result.isValid = result.violations.empty();
		return result;
	}

}  // namespace hdt

