#include "hdtXSDValidator.h"

#include "NetImmerseUtils.h"
#include "XmlReader.h"

#include <mutex>
#include <string>
#include <unordered_set>
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
		std::unordered_set<std::string> sharedValues;
		std::unordered_set<std::string> shapeTypes;
		std::unordered_set<std::string> constraintTags;
		bool loaded = false;
	};

	// Build a comma-separated string from a set of strings (for error messages).
	static std::string joinSet(const std::unordered_set<std::string>& s)
	{
		std::string result;
		for (const auto& v : s) {
			if (!result.empty())
				result += ", ";
			result += v;
		}
		return result;
	}

	// Collect all xs:enumeration/@value from a *named* xs:simpleType.
	// Used for hdtSMP64.xsd's top-level "shapeType" simpleType.
	static std::unordered_set<std::string> parseNamedSimpleTypeEnumerations(
		std::string& bytes, const std::string& typeName)
	{
		std::unordered_set<std::string> result;
		XMLReader reader(reinterpret_cast<uint8_t*>(bytes.data()), bytes.size());

		bool inTargetType = false;
		int depth = 0;

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
				const std::string localName = reader.GetLocalName();

				if (!inTargetType) {
					if (localName == "simpleType" && reader.hasAttribute("name") &&
						reader.getAttribute("name") == typeName) {
						inTargetType = true;
						depth = 0;
					}
				} else {
					++depth;
					if (localName == "enumeration" && reader.hasAttribute("value")) {
						result.insert(reader.getAttribute("value"));
					}
				}
			} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				if (inTargetType) {
					if (depth == 0) {
						break;
					}
					--depth;
				}
			}
		}

		return result;
	}

	// Collect all xs:enumeration/@value from the *anonymous* simpleType nested
	// inside a named xs:element. Used for hdtSMP64.xsd's <shared> element, which
	// embeds its restriction inline rather than referencing a named simpleType.
	static std::unordered_set<std::string> parseElementEnumerations(
		std::string& bytes, const std::string& elementName)
	{
		std::unordered_set<std::string> result;
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
					if (localName == "enumeration" && reader.hasAttribute("value")) {
						result.insert(reader.getAttribute("value"));
					}
				}
			} else if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				if (inElement) {
					if (depth == 0) {
						break;
					}
					--depth;
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

	static PhysicsSchema g_physicsSchema;
	static std::once_flag g_schemaOnce;

	static const PhysicsSchema& getPhysicsSchema()
	{
		std::call_once(g_schemaOnce, []() {
			auto bytes = readAllFile(kPhysicsXSDPath);
			if (bytes.empty()) {
				bytes = readAllFile2(kPhysicsXSDPath);
			}

			if (bytes.empty()) {
				logger::warn(
					"[XSDValidator] Could not load physics schema from '{}'; "
					"shared/shape/constraint validation will be skipped.",
					kPhysicsXSDPath);
				return;
			}

			try {
				// shapeType — named simpleType in hdtSMP64.xsd
				g_physicsSchema.shapeTypes =
					parseNamedSimpleTypeEnumerations(bytes, "shapeType");
				// shared values — anonymous simpleType inside <element name="shared">
				g_physicsSchema.sharedValues = parseElementEnumerations(bytes, "shared");
				// constraint element names — refs inside <element name="constraint-group">
				g_physicsSchema.constraintTags = parseConstraintTypes(bytes);
				g_physicsSchema.loaded = true;

				logger::info(
					"[XSDValidator] Loaded physics schema: {} shared value(s), "
					"{} shape type(s), {} constraint type(s).",
					g_physicsSchema.sharedValues.size(), g_physicsSchema.shapeTypes.size(),
					g_physicsSchema.constraintTags.size());
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
		std::unordered_set<std::string>& definedBones;
		std::unordered_set<std::string>& definedBodies;
		std::vector<std::string>& weightThresholdBones;
		bool& hasWeightThreshold;

		void addViolation(uint64_t line, uint64_t col, const std::string& msg)
		{
			std::string path;
			for (const auto& e : elementStack) {
				path += "/" + e;
			}
			violations.push_back({ xmlPath, line, col, path, msg });
		}
	};

	// ---- forward declarations ----

	static void validateSystem(XMLReader& reader, ValidationContext& ctx);
	static void validateBone(XMLReader& reader, ValidationContext& ctx);
	static void validateConstraint(XMLReader& reader, const std::string& tag,
		ValidationContext& ctx);
	static void validateShape(XMLReader& reader, const std::string& tag,
		ValidationContext& ctx);
	static void validateWeightThreshold(XMLReader& reader, ValidationContext& ctx);

	// ---- element validators ----

	static void validateSystem(XMLReader& reader, ValidationContext& ctx)
	{
		ctx.elementStack.push_back("system");

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				ctx.elementStack.pop_back();
				return;
			}
			if (reader.GetInspected() != XMLReader::Inspected::StartTag) {
				continue;
			}

			const std::string tag = reader.GetLocalName();
			const PhysicsSchema& schema = getPhysicsSchema();

			if (tag == "bone") {
				validateBone(reader, ctx);
			} else if (schema.constraintTags.count(tag)) {
				validateConstraint(reader, tag, ctx);
			} else if (schema.shapeTypes.count(tag) || tag == "per-triangle-shape" ||
					   tag == "per-vertex-shape") {
				validateShape(reader, tag, ctx);
			} else if (tag == "weight-threshold") {
				validateWeightThreshold(reader, ctx);
			} else {
				reader.skipCurrentElement();
			}
		}

		if (!ctx.elementStack.empty() && ctx.elementStack.back() == "system") {
			ctx.elementStack.pop_back();
		}
	}

	static void validateBone(XMLReader& reader, ValidationContext& ctx)
	{
		ctx.elementStack.push_back("bone");

		if (!reader.hasAttribute("name")) {
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"<bone> is missing required attribute 'name'");
		} else {
			ctx.definedBones.insert(reader.getAttribute("name"));
		}

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				ctx.elementStack.pop_back();
				return;
			}
			if (reader.GetInspected() != XMLReader::Inspected::StartTag) {
				continue;
			}

			const std::string tag = reader.GetLocalName();
			ctx.elementStack.push_back(tag);

			const PhysicsSchema& schema = getPhysicsSchema();
			if (tag == "shared") {
				// Read text and validate against allowed values from the schema
				const std::string val = reader.readText();
				if (schema.loaded && !schema.sharedValues.count(val)) {
					ctx.addViolation(reader.GetRow(), reader.GetColumn(),
						"<shared> has invalid value '" + val +
							"' (see hdtSMP64.xsd <shared> for valid values: " +
							joinSet(schema.sharedValues) + ")");
				}
			} else if (schema.shapeTypes.count(tag) || tag == "per-triangle-shape" ||
					   tag == "per-vertex-shape") {
				ctx.elementStack.pop_back();
				validateShape(reader, tag, ctx);
				continue;  // validateShape manages its own stack entry
			} else {
				reader.skipCurrentElement();
			}

			ctx.elementStack.pop_back();
		}

		if (!ctx.elementStack.empty() && ctx.elementStack.back() == "bone") {
			ctx.elementStack.pop_back();
		}
	}

	static void validateConstraint(XMLReader& reader, const std::string& tag,
		ValidationContext& ctx)
	{
		ctx.elementStack.push_back(tag);

		if (!reader.hasAttribute("bodyA")) {
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"<" + tag + "> is missing required attribute 'bodyA'");
		} else {
			ctx.definedBodies.insert(reader.getAttribute("bodyA"));
		}

		if (!reader.hasAttribute("bodyB")) {
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"<" + tag + "> is missing required attribute 'bodyB'");
		} else {
			ctx.definedBodies.insert(reader.getAttribute("bodyB"));
		}

		reader.skipCurrentElement();
		ctx.elementStack.pop_back();
	}

	static void validateShape(XMLReader& reader, const std::string& tag,
		ValidationContext& ctx)
	{
		ctx.elementStack.push_back(tag);

		const PhysicsSchema& schema = getPhysicsSchema();
		if (schema.loaded && !schema.shapeTypes.count(tag) && tag != "per-triangle-shape" &&
			tag != "per-vertex-shape") {
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"Unknown shape type <" + tag +
					"> (see hdtSMP64.xsd shapeType for valid values: " +
					joinSet(schema.shapeTypes) + ")");
		}

		reader.skipCurrentElement();
		ctx.elementStack.pop_back();
	}

	static void validateWeightThreshold(XMLReader& reader, ValidationContext& ctx)
	{
		ctx.elementStack.push_back("weight-threshold");
		ctx.hasWeightThreshold = true;

		if (!reader.hasAttribute("bone")) {
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"<weight-threshold> is missing required attribute 'bone'");
		} else {
			ctx.weightThresholdBones.push_back(reader.getAttribute("bone"));
		}

		reader.skipCurrentElement();
		ctx.elementStack.pop_back();
	}

	// ---- public API ----

	XSDValidationResult ValidatePhysicsXML(const std::string& xmlPath)
	{
		XSDValidationResult result;

		// Try VFS first (works for mod-managed files), then fall back to direct filesystem
		auto bytes = readAllFile(xmlPath.c_str());
		if (bytes.empty()) {
			bytes = readAllFile2(xmlPath.c_str());
		}

		if (bytes.empty()) {
			result.isValid = false;
			result.violations.push_back({ xmlPath, 0, 0, "", "File not found or empty" });
			return result;
		}

		// Store original locale and switch to "C" for numeric parsing.
		// "C" locale is guaranteed available on all systems and provides
		// the same dot-as-decimal-separator behaviour needed for XML values.
		const char* current_locale = std::setlocale(LC_NUMERIC, nullptr);
		std::string saved_locale = current_locale ? current_locale : "C";
		std::setlocale(LC_NUMERIC, "C");

		ValidationContext ctx{
			xmlPath,
			result.violations,
			{},
			result.definedBones,
			result.definedBodies,
			result.weightThresholdBones,
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

		std::setlocale(LC_NUMERIC, saved_locale.c_str());

		result.isValid = result.violations.empty();
		return result;
	}

}  // namespace hdt
