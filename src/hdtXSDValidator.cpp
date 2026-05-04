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

	// Parsed schema enumerations loaded once from hdtSMP64.xsd.
	struct PhysicsSchema
	{
		// Maps element tag names to their allowed values (inline anonymous simpleType enumerations).
		std::unordered_map<std::string, std::unordered_set<std::string>> elementEnums;
		// Root element tag — the first top-level xs:element in the XSD.
		std::string rootTag;
		// Derived from xs:key selectors nested inside the root element.
		std::string keySourceTag;
		std::unordered_set<std::string> perMeshShapeTags;
		std::string weightThresholdTag;
		// Required attributes per element — driven entirely by xs:attribute use="required".
		std::unordered_map<std::string, std::vector<std::string>> requiredAttrs;
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

				// root tag, bone tag, per-mesh shape tags — derived from the top-level xs:element
				// and its xs:key selectors.
				auto [rootTag, systemKeys] = parseSystemKeys(bytes);
				g_physicsSchema.rootTag = rootTag;
				if (systemKeys.count("boneKey")) {
					g_physicsSchema.keySourceTag = systemKeys["boneKey"];
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
				// required attributes per element — generic, read entirely from xs:attribute use="required".
				g_physicsSchema.requiredAttrs = parseAllRequiredAttrs(bytes);
				// weight-threshold tag — the element that requires the bone tag as an attribute.
				for (const auto& [elemName, attrs] : g_physicsSchema.requiredAttrs) {
					if (std::find(attrs.begin(), attrs.end(), g_physicsSchema.keySourceTag) != attrs.end()) {
						g_physicsSchema.weightThresholdTag = elemName;
						break;
					}
				}

				g_physicsSchema.loaded = true;

				logger::info(
					"[XSDValidator] Loaded physics schema: root '{}', "
					"{} enumerated element type(s), {} elements with required attr(s), "
					"bone tag '{}', {} known shape tag(s), weight-threshold tag '{}'.",
					g_physicsSchema.rootTag,
					g_physicsSchema.elementEnums.size(),
					g_physicsSchema.requiredAttrs.size(),
					g_physicsSchema.keySourceTag, g_physicsSchema.perMeshShapeTags.size(),
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
		bool weightThresholdSeen = false;
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
		const PhysicsSchema& schema = getPhysicsSchema();
		ctx.elementStack.push_back(schema.rootTag);

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

			// Generic required attribute check (XSD-driven, no hardcoded names).
			ctx.elementStack.push_back(tag);
			if (schema.requiredAttrs.count(tag)) {
				for (const auto& attr : schema.requiredAttrs.at(tag)) {
					requireAttr(tag, attr);
				}
			}

			if (tag == schema.keySourceTag) {
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
			} else {
				if (tag == schema.weightThresholdTag) {
					ctx.weightThresholdSeen = true;
				}
				reader.skipCurrentElement();
				ctx.elementStack.pop_back();
			}
		}

		if (!ctx.elementStack.empty() && ctx.elementStack.back() == schema.rootTag) {
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
			// Emit advisory warnings for XSD-defined recommended elements that were absent.
			if (foundSystem && !schema.weightThresholdTag.empty() && !ctx.weightThresholdSeen) {
				result.warnings.push_back({ xmlPath, 0, 0, "/" + schema.rootTag,
					"No <" + schema.weightThresholdTag + "> defined (may impact performance)" });
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

