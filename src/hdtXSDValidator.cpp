#include "hdtXSDValidator.h"

#include "NetImmerseUtils.h"
#include "XmlReader.h"

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

	// Parsed schema enumerations loaded once from hdtSMP64.xsd.
	struct PhysicsSchema
	{
		// Maps element tag names to their allowed values (inline anonymous simpleType enumerations).
		std::unordered_map<std::string, std::unordered_set<std::string>> elementEnums;
		std::unordered_set<std::string> constraintTags;
		// Derived from xs:key selectors and element structure inside hdtSMP64.xsd.
		std::string boneTag;
		std::unordered_set<std::string> perMeshShapeTags;
		std::string weightThresholdTag;
		// Required attributes on constraint elements, parsed from the XSD.
		std::vector<std::string> constraintBodyAttrs;
		// Key/unique and keyref definitions parsed from xsd:key, xsd:unique, xsd:keyref.
		struct KeyDef { std::unordered_set<std::string> elems; std::string fieldAttr; };
		struct KeyRefDef { std::string refer; std::unordered_set<std::string> elems; std::string fieldAttr; };
		std::unordered_map<std::string, KeyDef> keyDefs;       // key/unique name — declared value sets
		std::unordered_map<std::string, KeyRefDef> keyRefDefs; // keyref name — reference definitions
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

	// Collect xs:enumeration/@value from every *named* xs:simpleType in the XSD.
	// Returns a map from simpleType name to its set of enumerated values.
	// No type names are hardcoded here; the validator discovers them structurally.
	static std::unordered_map<std::string, std::unordered_set<std::string>>
		parseAllNamedSimpleTypeEnumerations(std::string& bytes)
	{
		std::unordered_map<std::string, std::unordered_set<std::string>> result;
		XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());

		std::string currentType;
		bool inType = false;
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
					if (localName == "enumeration" && reader.hasAttribute("value")) {
						currentEnums.insert(reader.getAttribute("value"));
					}
				}
			} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				if (inType) {
					if (depth == 0) {
						if (!currentEnums.empty()) {
							result[currentType] = std::move(currentEnums);
							currentEnums.clear();
						}
						inType = false;
						currentType.clear();
					} else {
						--depth;
					}
				}
			}
		}

		return result;
	}

	// Collect all xs:enumeration/@value from every xs:element that embeds an
	// anonymous simpleType with restrictions inline. Returns a map from element
	// name to its set of allowed values. No element names are hardcoded here;
	// the validator discovers them structurally from the XSD.
	static std::unordered_map<std::string, std::unordered_set<std::string>>
		parseAllElementEnumerations(std::string& bytes)
	{
		std::unordered_map<std::string, std::unordered_set<std::string>> result;
		XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());

		std::string currentElement;
		bool inElement = false;
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
					}
				} else {
					++depth;
					if (localName == "enumeration" && reader.hasAttribute("value")) {
						currentEnums.insert(reader.getAttribute("value"));
					}
				}
			} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				if (inElement) {
					if (depth == 0) {
						if (!currentEnums.empty()) {
							result[currentElement] = std::move(currentEnums);
							currentEnums.clear();
						}
						inElement = false;
						currentElement.clear();
					} else {
						--depth;
					}
				}
			}
		}

		return result;
	}

	// Collect constraint element names from the xs:element named "constraint-group".
	// hdtSMP64.xsd defines a <constraint-group> whose <xs:choice> lists all valid
	// constraint elements by ref. We collect ref values that end with "-constraint"
	// (excluding "-constraint-default" variants).
	// Suffix used to identify constraint element refs (excludes "-constraint-default").
	static const std::string kConstraintSuffix = "-constraint";

	static std::unordered_set<std::string> parseConstraintTypes(std::string& bytes)
	{
		std::unordered_set<std::string> result;
		XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());

		bool inGroup = false;
		int depth = 0;

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
				const std::string localName = reader.GetLocalName();

				if (!inGroup) {
					if (localName == "element" && reader.hasAttribute("name") &&
						reader.getAttribute("name") == "constraint-group") {
						inGroup = true;
						depth = 0;
					}
				} else {
					++depth;
					if (localName == "element" && reader.hasAttribute("ref")) {
						const std::string ref = reader.getAttribute("ref");
						// Accept anything ending in "-constraint" but not "-constraint-default"
						if (ref.size() > kConstraintSuffix.size() &&
							ref.compare(ref.size() - kConstraintSuffix.size(),
								kConstraintSuffix.size(), kConstraintSuffix) == 0) {
							result.insert(ref);
						}
					}
				}
			} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				if (inGroup) {
					if (depth == 0) {
						break;
					}
					--depth;
				}
			}
		}

		return result;
	}

	// Parse xs:key elements inside <element name="system"> and return a map of
	// (key name → selector xpath value). xs:key entries capture the roles of
	// uniquely-named element types: "boneKey" → "bone", "per-*-shapeKey" → shape names.
	static std::unordered_map<std::string, std::string> parseSystemKeys(std::string& bytes)
	{
		std::unordered_map<std::string, std::string> result;
		XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());

		bool inSystem = false;
		bool inKey = false;
		std::string currentKeyName;
		int systemDepth = 0;
		int keyDepth = 0;

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
				const std::string localName = reader.GetLocalName();

				if (!inSystem) {
					if (localName == "element" && reader.hasAttribute("name") &&
						reader.getAttribute("name") == "system") {
						inSystem = true;
						systemDepth = 0;
					}
				} else if (!inKey) {
					++systemDepth;
					if (localName == "key" && reader.hasAttribute("name")) {
						inKey = true;
						currentKeyName = reader.getAttribute("name");
						keyDepth = 0;
					}
				} else {
					++keyDepth;
					if (localName == "selector" && reader.hasAttribute("xpath")) {
						result[currentKeyName] = reader.getAttribute("xpath");
					}
				}
			} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				if (inKey) {
					if (keyDepth == 0) {
						inKey = false;
						--systemDepth;
					} else {
						--keyDepth;
					}
				} else if (inSystem) {
					if (systemDepth == 0) {
						break;
					}
					--systemDepth;
				}
			}
		}

		return result;
	}

	// Find the XSD element whose xs:complexType contains an xs:attribute with the given
	// name and use="required". Used to identify elements by structural signature rather
	// than by hardcoded name.
	static std::string parseElementByRequiredAttr(std::string& bytes, const std::string& attrName)
	{
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
						reader.hasAttribute("name") && reader.getAttribute("name") == attrName &&
						reader.hasAttribute("use") && reader.getAttribute("use") == "required") {
						return currentElementName;
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

		return {};
	}

	// Collect all xs:attribute use="required" names from the named XSD element.
	static std::vector<std::string> parseRequiredAttrs(std::string& bytes, const std::string& elementName)
	{
		std::vector<std::string> result;
		XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());

		bool inElement = false;
		int depth = 0;

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
				const std::string localName = reader.GetLocalName();

				if (!inElement) {
					if (localName == "element" && reader.hasAttribute("name") &&
						reader.getAttribute("name") == elementName) {
						inElement = true;
						depth = 0;
					}
				} else {
					++depth;
					if (localName == "attribute" &&
						reader.hasAttribute("name") &&
						reader.hasAttribute("use") && reader.getAttribute("use") == "required") {
						result.push_back(reader.getAttribute("name"));
					}
				}
			} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				if (inElement) {
					if (depth == 0) break;
					--depth;
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
				// constraint element names — refs inside <element name="constraint-group">
				g_physicsSchema.constraintTags = parseConstraintTypes(bytes);

				// bone tag, per-mesh shape tags, and weight-threshold tag — derived
				// from xs:key selectors and element structure inside <element name="system">.
				auto systemKeys = parseSystemKeys(bytes);
				if (systemKeys.count("boneKey")) {
					g_physicsSchema.boneTag = systemKeys["boneKey"];
				}
				{
					std::unordered_set<std::string> parsedShapeTags;
					for (const auto& [keyName, selector] : systemKeys) {
						if (keyName.size() >= 8 &&
							keyName.compare(keyName.size() - 8, 8, "shapeKey") == 0) {
							parsedShapeTags.insert(selector);
						}
					}
					if (!parsedShapeTags.empty()) {
						g_physicsSchema.perMeshShapeTags = std::move(parsedShapeTags);
					}
				}
				// named simpleType enumerations — their values are element tag names
				// (e.g. shapeType lists all valid shape element names).
				for (const auto& [typeName, values] : parseAllNamedSimpleTypeEnumerations(bytes)) {
					g_physicsSchema.perMeshShapeTags.insert(values.begin(), values.end());
				}
				// key/unique/keyref constraints — drive generic referential integrity checks.
				parseKeyConstraints(bytes, g_physicsSchema.keyDefs, g_physicsSchema.keyRefDefs);
				// weight-threshold tag — the element with a required attribute named after
				// the bone tag (derived from XSD structure, not a hardcoded name).
				auto wtTag = parseElementByRequiredAttr(bytes, g_physicsSchema.boneTag);
				if (!wtTag.empty()) {
					g_physicsSchema.weightThresholdTag = wtTag;
				}
				// constraint required attributes — parsed from the first constraint element
				// (all constraint elements share the same required attributes by convention).
				if (!g_physicsSchema.constraintTags.empty()) {
					g_physicsSchema.constraintBodyAttrs =
						parseRequiredAttrs(bytes, *g_physicsSchema.constraintTags.begin());
				}

				g_physicsSchema.loaded = true;

				logger::info(
					"[XSDValidator] Loaded physics schema: {} enumerated element type(s), "
					"{} constraint type(s) with {} required attr(s), "
					"bone tag '{}', {} known shape tag(s), weight-threshold tag '{}'.",
					g_physicsSchema.elementEnums.size(),
					g_physicsSchema.constraintTags.size(), g_physicsSchema.constraintBodyAttrs.size(),
					g_physicsSchema.boneTag, g_physicsSchema.perMeshShapeTags.size(),
					g_physicsSchema.weightThresholdTag);
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
		bool& hasWeightThreshold;
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

	static void validateSystem(XMLReader& reader, ValidationContext& ctx)
	{
		ctx.elementStack.push_back("system");

		const PhysicsSchema& schema = getPhysicsSchema();

		// Returns true and records the attribute value if present; logs a violation otherwise.
		auto requireAttr = [&](const std::string& element,
			const std::string& attrName) -> bool {
			if (reader.hasAttribute(attrName)) return true;
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"<" + element + "> is missing required attribute '" + attrName + "'");
			return false;
		};

		// Logs a violation if val is not in the allowed set for elemTag.
		auto checkEnumValue = [&](const std::string& elemTag, const std::string& val) {
			const auto& allowed = schema.elementEnums.at(elemTag);
			if (!allowed.count(val)) {
				ctx.addViolation(reader.GetRow(), reader.GetColumn(),
					"<" + elemTag + "> has invalid value '" + val +
						"' (see hdtSMP64.xsd for valid values: " + joinSet(allowed) + ")");
			}
		};

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				ctx.elementStack.pop_back();
				return;
			}
			if (reader.GetInspected() != XMLReader::Inspected::StartTag) {
				continue;
			}

			const std::string tag = reader.GetLocalName();

			// Generic key/keyref tracking (XSD-driven referential integrity).
			for (const auto& [kn, kd] : schema.keyDefs) {
				if (kd.elems.count(tag) && reader.hasAttribute(kd.fieldAttr))
					ctx.keyValues[kn].insert(reader.getAttribute(kd.fieldAttr));
			}
			for (const auto& [rn, rd] : schema.keyRefDefs) {
				if (rd.elems.count(tag) && reader.hasAttribute(rd.fieldAttr))
					ctx.keyRefPending[rn].emplace_back(reader.GetRow(), reader.getAttribute(rd.fieldAttr));
			}

			if (tag == schema.boneTag) {
				ctx.elementStack.push_back(tag);
				requireAttr(tag, "name");
				while (reader.Inspect()) {
					if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
						ctx.elementStack.pop_back();
						break;
					}
					if (reader.GetInspected() != XMLReader::Inspected::StartTag) {
						continue;
					}
					const std::string childTag = reader.GetLocalName();
					ctx.elementStack.push_back(childTag);
					if (schema.elementEnums.count(childTag)) {
						checkEnumValue(childTag, reader.readText());
					} else {
						reader.skipCurrentElement();
					}
					ctx.elementStack.pop_back();
				}
			} else if (schema.constraintTags.count(tag)) {
				ctx.elementStack.push_back(tag);
				for (const auto& attr : schema.constraintBodyAttrs) {
					requireAttr(tag, attr);
				}
				reader.skipCurrentElement();
				ctx.elementStack.pop_back();
			} else if (schema.perMeshShapeTags.count(tag)) {
				reader.skipCurrentElement();
			} else if (tag == schema.weightThresholdTag) {
				ctx.elementStack.push_back(tag);
				ctx.hasWeightThreshold = true;
				requireAttr(tag, schema.boneTag);
				reader.skipCurrentElement();
				ctx.elementStack.pop_back();
			} else {
				reader.skipCurrentElement();
			}
		}

		if (!ctx.elementStack.empty() && ctx.elementStack.back() == "system") {
			ctx.elementStack.pop_back();
		}
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

		ValidationContext ctx{
			xmlPath,
			result.violations,
			{},
			result.hasWeightThreshold
		};

		try {
			XMLReader reader((uint8_t*)bytes.data(), bytes.size());
			bool foundSystem = false;

			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
					const std::string tag = reader.GetLocalName();
					if (tag == "system") {
						foundSystem = true;
						validateSystem(reader, ctx);
						// Referential integrity — check all pending keyref values against declared keys.
						const PhysicsSchema& schema = getPhysicsSchema();
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
										"/system",
										"Undeclared reference '" + val + "' (must be declared in '" +
											refDef.refer + "')" });
								}
							}
						}
					} else {
						ctx.addViolation(reader.GetRow(), reader.GetColumn(),
							"Root element must be <system>, found <" + tag + ">");
						reader.skipCurrentElement();
					}
				}
			}

			if (!foundSystem) {
				result.violations.push_back({ xmlPath, 0, 0, "/", "No <system> root element found" });
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

