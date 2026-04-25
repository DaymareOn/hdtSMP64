#include "hdtXSDValidator.h"

#include "NetImmerseUtils.h"
#include "XmlReader.h"

#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
	// Path to the physics XSD schema file, relative to the game working directory.
	// This file defines the accepted xs:enumeration values for sharedType, shapeType,
	// and constraintType — edit it without recompiling to extend or restrict validation.
	static const char* kPhysicsXSDPath =
		"data/skse/plugins/hdtSkinnedMeshConfigs/physics.xsd";

	// Parsed schema enumerations loaded once from physics.xsd.
	struct PhysicsSchema
	{
		std::unordered_set<std::string> sharedValues;
		std::unordered_set<std::string> shapeTypes;
		std::unordered_set<std::string> constraintTags;
		bool loaded = false;
	};

	// Parse all xs:enumeration/@value strings from the xs:simpleType with the given name.
	// Returns an empty set if the named type is not found.
	static std::unordered_set<std::string> parseSimpleTypeEnumerations(
		const std::vector<uint8_t>& xsdBytes, const std::string& typeName)
	{
		std::unordered_set<std::string> result;
		XMLReader reader(const_cast<uint8_t*>(xsdBytes.data()), xsdBytes.size());

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
						// End of the target simpleType element
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
				logger::warn("[XSDValidator] Could not load physics schema from '{}'; "
							 "shared/shape/constraint validation will be skipped.",
					kPhysicsXSDPath);
				return;
			}

			try {
				g_physicsSchema.sharedValues =
					parseSimpleTypeEnumerations(bytes, "sharedType");
				g_physicsSchema.shapeTypes =
					parseSimpleTypeEnumerations(bytes, "shapeType");
				g_physicsSchema.constraintTags =
					parseSimpleTypeEnumerations(bytes, "constraintType");
				g_physicsSchema.loaded = true;

				logger::info(
					"[XSDValidator] Loaded physics schema: {} shared value(s), "
					"{} shape type(s), {} constraint type(s).",
					g_physicsSchema.sharedValues.size(), g_physicsSchema.shapeTypes.size(),
					g_physicsSchema.constraintTags.size());
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

			if (tag == "shared") {
				// Read text and validate against allowed values from the schema
				const std::string val = reader.readText();
				const PhysicsSchema& schema = getPhysicsSchema();
				if (schema.loaded && !schema.sharedValues.count(val)) {
					std::string expected;
					for (const auto& v : schema.sharedValues) {
						if (!expected.empty()) expected += ", ";
						expected += v;
					}
					ctx.addViolation(reader.GetRow(), reader.GetColumn(),
						"<shared> has invalid value '" + val +
							"' (see physics.xsd sharedType for valid values: " + expected + ")");
				}
			} else if (getPhysicsSchema().shapeTypes.count(tag) || tag == "per-triangle-shape" ||
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
			std::string expected;
			for (const auto& v : schema.shapeTypes) {
				if (!expected.empty()) expected += ", ";
				expected += v;
			}
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"Unknown shape type <" + tag +
					"> (see physics.xsd shapeType for valid values: " + expected + ")");
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
		char saved_locale[64];
		strcpy_s(saved_locale, std::setlocale(LC_NUMERIC, nullptr));
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

		std::setlocale(LC_NUMERIC, saved_locale);

		result.isValid = result.violations.empty();
		return result;
	}

}  // namespace hdt

